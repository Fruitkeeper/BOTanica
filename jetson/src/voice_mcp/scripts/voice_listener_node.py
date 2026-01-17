#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BOTanica Voice Listener Node

ROS node that listens to microphone, transcribes speech with OpenAI Whisper,
and publishes commands to the robot.

Modes:
- Continuous: Always listening, detects speech via voice activity detection
- Push-to-talk: Only records when a button/service is triggered

Usage:
    rosrun voice_mcp voice_listener_node.py

Environment variables:
    OPENAI_API_KEY - Required for Whisper API
"""
import os
import sys
import io
import wave
import threading
import numpy as np

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

# Local imports
from command_parser import CommandParser
from whisper_client import WhisperClient

# Audio recording imports
try:
    import sounddevice as sd
    AUDIO_AVAILABLE = True
except ImportError:
    AUDIO_AVAILABLE = False
    rospy.logwarn("sounddevice not installed - audio recording disabled")


class VoiceListenerNode:
    """ROS node for voice command listening and processing."""

    def __init__(self):
        rospy.init_node("voice_listener")

        # Parameters
        self.sample_rate = rospy.get_param("~sample_rate", 16000)
        self.channels = rospy.get_param("~channels", 1)
        self.mode = rospy.get_param("~mode", "push_to_talk")  # "continuous" or "push_to_talk"
        self.silence_threshold = rospy.get_param("~silence_threshold", 0.01)
        self.silence_duration = rospy.get_param("~silence_duration", 1.5)  # seconds
        self.max_record_duration = rospy.get_param("~max_record_duration", 10.0)  # seconds
        self.cooldown_duration = rospy.get_param("~cooldown_duration", 1.0)  # seconds between recordings

        # Check for API key
        if not os.environ.get("OPENAI_API_KEY"):
            rospy.logerr("OPENAI_API_KEY environment variable not set!")
            rospy.logerr("Set it with: export OPENAI_API_KEY='your-key-here'")
            sys.exit(1)

        # Initialize components
        self.parser = CommandParser()
        self.whisper = WhisperClient()

        # State
        self.is_recording = False
        self.last_command_time = rospy.Time.now()

        # Publisher for voice commands
        self.cmd_pub = rospy.Publisher("/voice_command", String, queue_size=10)

        # Service for push-to-talk trigger
        self.listen_srv = rospy.Service("~listen", Trigger, self.handle_listen_trigger)

        # Status publisher
        self.status_pub = rospy.Publisher("~status", String, queue_size=1)

        rospy.loginfo("Voice Listener Node initialized")
        rospy.loginfo(f"  Mode: {self.mode}")
        rospy.loginfo(f"  Sample rate: {self.sample_rate}")
        rospy.loginfo(f"  Max record duration: {self.max_record_duration}s")

        if not AUDIO_AVAILABLE:
            rospy.logerr("Audio recording not available - install sounddevice")
            rospy.logerr("Run: pip3 install sounddevice")
            return

        # List available audio devices
        self._list_audio_devices()

        # Start listening based on mode
        if self.mode == "continuous":
            rospy.loginfo("Starting continuous listening mode...")
            self.continuous_thread = threading.Thread(target=self._continuous_listen_loop)
            self.continuous_thread.daemon = True
            self.continuous_thread.start()
        else:
            rospy.loginfo("Push-to-talk mode - call /voice_listener/listen service to record")

    def _list_audio_devices(self):
        """List available audio input devices."""
        try:
            devices = sd.query_devices()
            rospy.loginfo("Available audio devices:")
            for i, dev in enumerate(devices):
                if dev['max_input_channels'] > 0:
                    rospy.loginfo(f"  [{i}] {dev['name']} (inputs: {dev['max_input_channels']})")
        except Exception as e:
            rospy.logwarn(f"Could not list audio devices: {e}")

    def handle_listen_trigger(self, req):
        """Service handler for push-to-talk trigger."""
        if self.is_recording:
            return TriggerResponse(success=False, message="Already recording")

        # Check cooldown
        elapsed = (rospy.Time.now() - self.last_command_time).to_sec()
        if elapsed < self.cooldown_duration:
            return TriggerResponse(
                success=False,
                message=f"Cooldown - wait {self.cooldown_duration - elapsed:.1f}s"
            )

        # Record and process in a thread
        thread = threading.Thread(target=self._record_and_process)
        thread.start()

        return TriggerResponse(success=True, message="Recording started")

    def _continuous_listen_loop(self):
        """Continuous listening loop with voice activity detection."""
        rospy.loginfo("Continuous listening started")

        while not rospy.is_shutdown():
            try:
                # Wait for speech to start
                if self._detect_speech_start():
                    self._record_and_process()

                # Cooldown
                rospy.sleep(self.cooldown_duration)

            except Exception as e:
                rospy.logerr(f"Continuous listen error: {e}")
                rospy.sleep(1.0)

    def _detect_speech_start(self) -> bool:
        """Listen for speech onset (voice activity detection)."""
        try:
            # Record short chunks and check for voice activity
            chunk_duration = 0.5  # seconds
            chunk_samples = int(self.sample_rate * chunk_duration)

            with sd.InputStream(
                samplerate=self.sample_rate,
                channels=self.channels,
                dtype='float32'
            ) as stream:
                while not rospy.is_shutdown():
                    audio_chunk, _ = stream.read(chunk_samples)
                    rms = np.sqrt(np.mean(audio_chunk**2))

                    if rms > self.silence_threshold:
                        rospy.loginfo("Speech detected, starting recording...")
                        return True

                    rospy.sleep(0.1)

        except Exception as e:
            rospy.logerr(f"Speech detection error: {e}")

        return False

    def _record_and_process(self):
        """Record audio, transcribe, and publish command."""
        if self.is_recording:
            return

        self.is_recording = True
        self.status_pub.publish(String(data="recording"))

        try:
            # Record audio
            rospy.loginfo("Recording...")
            audio_data = self._record_audio()

            if audio_data is None or len(audio_data) == 0:
                rospy.logwarn("No audio recorded")
                return

            self.status_pub.publish(String(data="transcribing"))

            # Convert to WAV bytes
            wav_bytes = self._to_wav_bytes(audio_data)

            # Transcribe with Whisper
            rospy.loginfo("Transcribing with Whisper API...")
            transcription = self.whisper.transcribe(wav_bytes)

            if not transcription:
                rospy.logwarn("No transcription returned")
                return

            rospy.loginfo(f"Heard: '{transcription}'")
            self.status_pub.publish(String(data=f"heard: {transcription}"))

            # Parse command
            parsed = self.parser.parse(transcription)

            if not parsed["recognized"]:
                rospy.logwarn(f"Command not recognized: '{transcription}'")
                self.status_pub.publish(String(data="not recognized"))
                return

            # Publish command
            # Format: command|raw_text|force|source
            msg_data = f"{parsed['command']}|{transcription}|{parsed['force']}|voice"
            self.cmd_pub.publish(String(data=msg_data))

            rospy.loginfo(f"Published command: {parsed['command']} (force={parsed['force']})")
            self.status_pub.publish(String(data=f"command: {parsed['command']}"))

            self.last_command_time = rospy.Time.now()

        except Exception as e:
            rospy.logerr(f"Record and process error: {e}")
            self.status_pub.publish(String(data=f"error: {e}"))

        finally:
            self.is_recording = False
            self.status_pub.publish(String(data="idle"))

    def _record_audio(self) -> np.ndarray:
        """
        Record audio from microphone until silence or max duration.

        Returns:
            numpy array of audio samples
        """
        frames = []
        silent_frames = 0
        chunk_size = 1024
        max_silent_chunks = int(self.silence_duration * self.sample_rate / chunk_size)
        max_chunks = int(self.max_record_duration * self.sample_rate / chunk_size)

        try:
            with sd.InputStream(
                samplerate=self.sample_rate,
                channels=self.channels,
                dtype='float32',
                blocksize=chunk_size
            ) as stream:
                total_chunks = 0

                while total_chunks < max_chunks:
                    audio_chunk, _ = stream.read(chunk_size)
                    frames.append(audio_chunk)
                    total_chunks += 1

                    # Voice activity detection
                    rms = np.sqrt(np.mean(audio_chunk**2))

                    if rms < self.silence_threshold:
                        silent_frames += 1
                        if silent_frames >= max_silent_chunks and total_chunks > 10:
                            rospy.loginfo("Silence detected, stopping recording")
                            break
                    else:
                        silent_frames = 0

                duration = total_chunks * chunk_size / self.sample_rate
                rospy.loginfo(f"Recorded {duration:.1f}s of audio")

        except Exception as e:
            rospy.logerr(f"Recording error: {e}")
            return None

        if not frames:
            return None

        return np.concatenate(frames)

    def _to_wav_bytes(self, audio_data: np.ndarray) -> bytes:
        """Convert numpy audio array to WAV bytes."""
        buffer = io.BytesIO()

        with wave.open(buffer, 'wb') as wav:
            wav.setnchannels(self.channels)
            wav.setsampwidth(2)  # 16-bit
            wav.setframerate(self.sample_rate)
            # Convert float32 to int16
            wav.writeframes((audio_data * 32767).astype(np.int16).tobytes())

        return buffer.getvalue()

    def run(self):
        """Run the node."""
        rospy.spin()


if __name__ == "__main__":
    try:
        node = VoiceListenerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
