#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GPT-4 Planner for BOTanica Agentic Controller

Sends robot state + user goal to GPT-4 and receives the next action as JSON.
Maintains conversation history across steps for multi-step goal execution.
"""
import os
import json
import logging

logger = logging.getLogger(__name__)

SYSTEM_PROMPT = """You are the controller for BOTanica, a plant-care robot built on a DJI RoboMaster S1 with mecanum wheels.
You receive a goal from the user and the robot's current sensor/pose state.
You must respond with exactly ONE action at a time as a JSON object.

## Available Actions

### High-Level (robot handles navigation automatically)
- GO_TO_WATER: Navigate to water station, water the plant, then return to idle. No params.
- GO_TO_DOCK: Navigate to charging dock. No params.
- LIGHT_SCAN: 360-degree scan to find and move toward the brightest light. No params.
- STOP: Emergency stop all movement. No params.

### Low-Level Movement
- MOVE: Drive forward/backward. Params: {"distance_m": float, "speed": float (default 0.15, max 0.3)}. Positive = forward, negative = backward.
- TURN: Rotate in place. Params: {"degrees": float, "speed": float (default 30, max 60)}. Positive = counter-clockwise (left), negative = clockwise (right).
- STRAFE: Sideways movement (mecanum wheels). Params: {"distance_m": float, "speed": float (default 0.15, max 0.3)}. Positive = left, negative = right.
- WAIT: Do nothing. Params: {"seconds": float}.
- MOVE_TO: Navigate to absolute coordinates. Params: {"x": float, "y": float}.

### Meta (no movement)
- READ_SENSORS: Get a fresh sensor snapshot. No params. Use this to check conditions before deciding.
- DONE: Goal is complete. Params: {"summary": "string describing what was accomplished"}.
- SPEAK: Log a message for the user. Params: {"message": "string"}.

## Response Format (strict JSON, no markdown, no extra text)
{"action": "ACTION_NAME", "params": {}, "reasoning": "Brief explanation of why this action"}

For DONE: {"action": "DONE", "params": {"summary": "..."}, "reasoning": "..."}
For SPEAK: {"action": "SPEAK", "params": {"message": "..."}, "reasoning": "..."}
For actions with no params: {"action": "GO_TO_WATER", "params": {}, "reasoning": "..."}

## Rules
- Respond with ONE action per message. You will be called again with the result and updated state.
- If battery_pct < 20, you MUST use GO_TO_DOCK unless the user's goal explicitly says to ignore battery.
- For geometric patterns (squares, circles, zigzags), compose them from MOVE and TURN.
- Speed limits: max linear 0.3 m/s, max rotation 60 deg/s. Stay conservative indoors.
- When you're done with the user's goal, always finish with DONE and a summary.
- If something goes wrong (action failed, unexpected state), you can retry or adjust your plan.
- You can use READ_SENSORS at any time to get fresh data before making decisions.
- WAIT is useful for patrol loops or timed checks."""


class GPTPlanner:
    """Interface to GPT-4 for agentic robot control."""

    def __init__(self, model: str = "gpt-4", api_key: str = None):
        self.model = model
        self.api_key = api_key or os.environ.get("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError(
                "OpenAI API key required. Set OPENAI_API_KEY environment variable "
                "or pass api_key parameter."
            )

        try:
            from openai import OpenAI
            self.client = OpenAI(api_key=self.api_key)
        except ImportError:
            raise ImportError("OpenAI library not installed. Run: pip3 install openai")

        logger.info(f"GPT Planner initialized with model: {self.model}")

    def get_next_action(self, messages: list) -> dict:
        """
        Call GPT-4 with conversation history and get the next action.

        Args:
            messages: List of conversation messages in OpenAI format.
                      Should include the system prompt and all prior steps.

        Returns:
            Parsed action dict with keys: action, params, reasoning
        """
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                response_format={"type": "json_object"},
                temperature=0.2,
                max_tokens=300,
            )

            content = response.choices[0].message.content
            logger.info(f"GPT-4 response: {content}")

            parsed = json.loads(content)

            # Validate required fields
            if "action" not in parsed:
                logger.error(f"Missing 'action' in response: {content}")
                return {
                    "action": "SPEAK",
                    "params": {"message": "Planning error: no action returned"},
                    "reasoning": "Error recovery",
                }

            # Ensure params exists
            if "params" not in parsed:
                parsed["params"] = {}
            if "reasoning" not in parsed:
                parsed["reasoning"] = ""

            return parsed

        except json.JSONDecodeError as e:
            logger.error(f"Failed to parse GPT-4 JSON: {e}")
            return {
                "action": "SPEAK",
                "params": {"message": f"Planning error: invalid JSON from LLM"},
                "reasoning": "JSON parse error",
            }
        except Exception as e:
            logger.error(f"GPT-4 API error: {e}")
            return {
                "action": "SPEAK",
                "params": {"message": f"Planning error: {e}"},
                "reasoning": "API error",
            }

    @staticmethod
    def build_system_message() -> dict:
        """Return the system message for the conversation."""
        return {"role": "system", "content": SYSTEM_PROMPT}

    @staticmethod
    def build_user_message(goal: str, state: dict) -> dict:
        """Build the initial user message with goal and state."""
        content = f"Goal: {goal}\n\nCurrent robot state:\n{json.dumps(state, indent=2)}"
        return {"role": "user", "content": content}

    @staticmethod
    def build_observation_message(result: dict, state: dict) -> dict:
        """Build an observation message after an action completes."""
        content = (
            f"Action result: {json.dumps(result)}\n\n"
            f"Updated robot state:\n{json.dumps(state, indent=2)}"
        )
        return {"role": "user", "content": content}

    @staticmethod
    def build_assistant_message(action: dict) -> dict:
        """Wrap an action response as an assistant message."""
        return {"role": "assistant", "content": json.dumps(action)}


# For standalone testing
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    if not os.environ.get("OPENAI_API_KEY"):
        print("Set OPENAI_API_KEY environment variable first")
        exit(1)

    planner = GPTPlanner()

    # Simulate a conversation
    messages = [GPTPlanner.build_system_message()]

    test_state = {
        "battery_pct": 85,
        "moisture_pct": 28,
        "sunlight_lux": 320,
        "temperature_c": 22.5,
        "position": {"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
        "brain_state": "IDLE",
        "force_override": False,
    }

    messages.append(GPTPlanner.build_user_message(
        "Check moisture, water if needed, then find the best light",
        test_state,
    ))

    print("=== GPT Planner Test ===")
    for step in range(5):
        action = planner.get_next_action(messages)
        messages.append(GPTPlanner.build_assistant_message(action))

        print(f"\nStep {step + 1}: {json.dumps(action, indent=2)}")

        if action["action"] == "DONE":
            break

        # Simulate state change
        if action["action"] == "GO_TO_WATER":
            test_state["moisture_pct"] = 62
            test_state["brain_state"] = "IDLE"
        elif action["action"] == "READ_SENSORS":
            pass  # No change

        result = {"success": True, "message": f"{action['action']} completed"}
        messages.append(GPTPlanner.build_observation_message(result, test_state))
