#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Command Parser for BOTanica Voice MCP Server

Parses natural language text into robot commands.
Supports "force" keyword to override safety priorities.
"""
import re
import logging

logger = logging.getLogger(__name__)


class CommandParser:
    """Parse natural language commands into robot actions."""

    def __init__(self):
        # Command patterns (order matters - more specific patterns first)
        # Format: (regex_pattern, command_name, force_override)
        self.patterns = [
            # Force override commands (highest priority)
            (r'\bforce\s+go\s+to\s+water\b', "GO_TO_WATER", True),
            (r'\bforce\s+water\b', "GO_TO_WATER", True),
            (r'\bforce\s+go\s+to\s+dock\b', "GO_TO_DOCK", True),
            (r'\bforce\s+dock\b', "GO_TO_DOCK", True),
            (r'\bforce\s+go\s+home\b', "GO_TO_DOCK", True),
            (r'\bforce\s+find\s+light\b', "LIGHT_SCAN", True),
            (r'\bforce\s+seek\s+light\b', "LIGHT_SCAN", True),
            (r'\bforce\s+light\b', "LIGHT_SCAN", True),

            # Water station commands
            (r'\bgo\s+to\s+water\s*doser\b', "GO_TO_WATER", False),
            (r'\bgo\s+to\s+water\s*station\b', "GO_TO_WATER", False),
            (r'\bgo\s+to\s+water\b', "GO_TO_WATER", False),
            (r'\bget\s+water\b', "GO_TO_WATER", False),
            (r'\bneed\s+water\b', "GO_TO_WATER", False),
            (r'\bwater\s+me\b', "GO_TO_WATER", False),
            (r'\bwater\b', "GO_TO_WATER", False),

            # Dock / charging commands
            (r'\bgo\s+to\s+dock\b', "GO_TO_DOCK", False),
            (r'\bgo\s+to\s+charging\b', "GO_TO_DOCK", False),
            (r'\bgo\s+home\b', "GO_TO_DOCK", False),
            (r'\breturn\s+to\s+dock\b', "GO_TO_DOCK", False),
            (r'\breturn\s+home\b', "GO_TO_DOCK", False),
            (r'\bdock\b', "GO_TO_DOCK", False),
            (r'\bcharge\b', "GO_TO_DOCK", False),
            (r'\bcharging\b', "GO_TO_DOCK", False),

            # Light seeking commands
            (r'\bfind\s+light\b', "LIGHT_SCAN", False),
            (r'\bseek\s+light\b', "LIGHT_SCAN", False),
            (r'\bfollow\s+light\b', "LIGHT_SCAN", False),
            (r'\blook\s+for\s+light\b', "LIGHT_SCAN", False),
            (r'\bscan\s+for\s+light\b', "LIGHT_SCAN", False),
            (r'\blight\s+scan\b', "LIGHT_SCAN", False),

            # Stop commands (always allowed)
            (r'\bstop\b', "STOP", False),
            (r'\bhalt\b', "STOP", False),
            (r'\bfreeze\b', "STOP", False),
            (r'\bemergency\s+stop\b', "STOP", False),

            # Status command
            (r'\bstatus\b', "STATUS", False),
            (r'\bhow\s+are\s+you\b', "STATUS", False),
            (r'\bwhat\'?s?\s+your\s+status\b', "STATUS", False),
        ]

    def parse(self, text: str) -> dict:
        """
        Parse natural language text into a robot command.

        Args:
            text: Input text (from voice transcription or typed command)

        Returns:
            dict with keys:
                - recognized: bool - whether a valid command was found
                - command: str - the command name (e.g., "GO_TO_WATER")
                - force: bool - whether force override was requested
                - original: str - the original input text
        """
        text_lower = text.lower().strip()

        for pattern, command, force in self.patterns:
            if re.search(pattern, text_lower):
                logger.info(f"Parsed '{text}' -> {command} (force={force})")
                return {
                    "recognized": True,
                    "command": command,
                    "force": force,
                    "original": text
                }

        logger.warning(f"Unrecognized command: '{text}'")
        return {
            "recognized": False,
            "command": None,
            "force": False,
            "original": text
        }

    def get_available_commands(self) -> list:
        """Return list of available command descriptions."""
        return [
            "go to water / water - Navigate to water station",
            "go to dock / go home / dock / charge - Navigate to charging dock",
            "find light / seek light - Start light-seeking behavior",
            "stop / halt - Stop all movement",
            "status - Get robot status",
            "force <command> - Override safety priorities (e.g., 'force go to water')"
        ]


# For standalone testing
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    parser = CommandParser()

    test_inputs = [
        "go to water",
        "Go to the water doser",
        "force go to water",
        "find light",
        "STOP",
        "go home",
        "force dock",
        "what's your status",
        "do a backflip",  # Should not recognize
    ]

    print("Command Parser Test\n" + "=" * 40)
    for text in test_inputs:
        result = parser.parse(text)
        status = "✓" if result["recognized"] else "✗"
        print(f"{status} '{text}' -> {result['command']} (force={result['force']})")
