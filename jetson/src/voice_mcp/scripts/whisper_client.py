#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OpenAI Whisper API Client for BOTanica Voice Control

Handles audio transcription using OpenAI's Whisper API.
"""
import os
import io
import logging

logger = logging.getLogger(__name__)


class WhisperClient:
    """Client for OpenAI Whisper speech-to-text API."""

    def __init__(self, api_key: str = None):
        """
        Initialize Whisper client.

        Args:
            api_key: OpenAI API key. If None, reads from OPENAI_API_KEY env var.
        """
        self.api_key = api_key or os.environ.get("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError(
                "OpenAI API key required. Set OPENAI_API_KEY environment variable "
                "or pass api_key parameter."
            )

        # Import OpenAI library
        try:
            from openai import OpenAI
            self.client = OpenAI(api_key=self.api_key)
        except ImportError:
            raise ImportError("OpenAI library not installed. Run: pip3 install openai")

        self.model = "whisper-1"
        logger.info("Whisper client initialized")

    def transcribe(self, audio_bytes: bytes, language: str = "en") -> str:
        """
        Transcribe audio bytes using OpenAI Whisper API.

        Args:
            audio_bytes: WAV audio data as bytes
            language: Language code (default "en" for English)

        Returns:
            Transcribed text string, or empty string on error
        """
        try:
            # Create file-like object from bytes
            audio_file = io.BytesIO(audio_bytes)
            audio_file.name = "recording.wav"

            # Call Whisper API
            response = self.client.audio.transcriptions.create(
                model=self.model,
                file=audio_file,
                language=language,
                response_format="text"
            )

            transcription = response.strip()
            logger.info(f"Transcribed: {transcription}")
            return transcription

        except Exception as e:
            logger.error(f"Whisper transcription error: {e}")
            return ""


# For standalone testing
if __name__ == "__main__":
    import sys

    logging.basicConfig(level=logging.INFO)

    if not os.environ.get("OPENAI_API_KEY"):
        print("Set OPENAI_API_KEY environment variable first")
        sys.exit(1)

    client = WhisperClient()
    print("Whisper client initialized successfully")
    print("To test, call client.transcribe(audio_bytes)")
