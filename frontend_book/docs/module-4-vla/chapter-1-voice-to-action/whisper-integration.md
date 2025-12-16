---
sidebar_position: 2
title: "Whisper Integration"
description: "Integrating OpenAI Whisper for speech recognition in robotic systems"
---

# Whisper Integration

## Introduction to OpenAI Whisper

OpenAI Whisper is a state-of-the-art speech recognition model that can transcribe speech to text with high accuracy. In robotic systems, Whisper enables natural voice interaction by converting spoken commands into text that can be processed by the robot's control system.

## Why Whisper for Robotics?

Whisper is particularly well-suited for robotic applications because:

- **High Accuracy**: Trained on diverse datasets, providing reliable transcription
- **Multilingual Support**: Can recognize speech in multiple languages
- **Robustness**: Performs well in various acoustic conditions
- **API Availability**: Easy integration through OpenAI's API
- **Real-time Capabilities**: Can process audio streams efficiently

## Setting Up Whisper API Access

Before integrating Whisper into your robotic system, you'll need:

1. An OpenAI account
2. An API key from the OpenAI platform
3. Proper billing setup for API usage

### API Key Configuration

Store your OpenAI API key securely:

```bash
# Create a .env file in your project root
OPENAI_API_KEY=your_actual_api_key_here
```

## Basic Whisper Integration

Here's a simple example of how to use Whisper for speech recognition:

```python
import openai
import os
from pathlib import Path

# Initialize OpenAI client
openai.api_key = os.getenv("OPENAI_API_KEY")

def transcribe_audio(audio_file_path):
    """
    Transcribe audio file using OpenAI Whisper API
    """
    with open(audio_file_path, "rb") as audio_file:
        transcript = openai.Audio.transcribe("whisper-1", audio_file)
    return transcript.text

# Example usage
audio_path = "path/to/your/voice_command.wav"
command_text = transcribe_audio(audio_path)
print(f"Recognized command: {command_text}")
```

## Whisper in the Robotics Context

When integrating Whisper into a robotic system, consider these key aspects:

### Audio Quality and Preprocessing

- **Microphone Quality**: Use a clear microphone for best results
- **Audio Format**: Whisper works best with WAV, MP3, or similar formats
- **Noise Reduction**: Consider preprocessing to reduce background noise
- **Audio Length**: Keep commands concise for better accuracy

### Real-time vs. Batch Processing

For robotics, you have two main approaches:

1. **Real-time Streaming**: Process audio as it's being captured
2. **Batch Processing**: Collect audio and process after speaking stops

```python
# Example: Real-time audio capture and processing
import pyaudio
import wave
import threading

class VoiceCommandProcessor:
    def __init__(self):
        self.api_key = os.getenv("OPENAI_API_KEY")
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 44100

    def capture_and_transcribe(self):
        """
        Capture audio from microphone and transcribe using Whisper
        """
        p = pyaudio.PyAudio()

        # Open stream
        stream = p.open(format=self.format,
                        channels=self.channels,
                        rate=self.rate,
                        input=True,
                        frames_per_buffer=self.chunk)

        print("Listening for voice command...")

        # Record audio
        frames = []
        silence_threshold = 1000  # Adjust as needed
        silence_count = 0
        max_silence = 30  # Max silence frames before stopping

        while True:
            data = stream.read(self.chunk)
            frames.append(data)

            # Check for silence to stop recording
            audio_data = np.frombuffer(data, dtype=np.int16)
            if np.abs(audio_data).mean() < silence_threshold:
                silence_count += 1
                if silence_count > max_silence:
                    break
            else:
                silence_count = 0

        # Stop and close stream
        stream.stop_stream()
        stream.close()
        p.terminate()

        # Save temporary file
        temp_file = "temp_command.wav"
        wf = wave.open(temp_file, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(p.get_sample_size(self.format))
        wf.setframerate(self.rate)
        wf.writeframes(b''.join(frames))
        wf.close()

        # Transcribe the audio
        with open(temp_file, "rb") as audio_file:
            transcript = openai.Audio.transcribe("whisper-1", audio_file)

        # Clean up temp file
        os.remove(temp_file)

        return transcript.text
```

## Handling Transcription Results

Once you have the transcribed text, you need to process it for robotic action:

```python
def process_voice_command(transcript):
    """
    Process the transcribed text and extract actionable commands
    """
    # Convert to lowercase for easier processing
    command = transcript.lower().strip()

    # Define known commands
    known_commands = {
        "move forward": "move_forward",
        "move backward": "move_backward",
        "turn left": "turn_left",
        "turn right": "turn_right",
        "stop": "stop",
        "pick up object": "pick_up",
        "place object": "place",
        "go to location": "navigate_to"
    }

    # Find the best matching command
    for text, action in known_commands.items():
        if text in command:
            return action

    # If no exact match, return as unknown
    return "unknown_command"
```

## Error Handling and Fallbacks

Speech recognition isn't perfect, so implement robust error handling:

```python
def safe_transcribe_with_fallback(audio_file_path):
    """
    Transcribe audio with error handling and fallback mechanisms
    """
    try:
        # Attempt primary transcription
        with open(audio_file_path, "rb") as audio_file:
            transcript = openai.Audio.transcribe(
                "whisper-1",
                audio_file,
                response_format="verbose_json"  # Get confidence scores
            )

        # Check confidence level
        if transcript.confidence < 0.7:  # Threshold for reliability
            print("Low confidence transcription, asking for clarification")
            return handle_low_confidence(transcript.text)

        return transcript.text

    except Exception as e:
        print(f"Whisper API error: {e}")
        # Fallback: return error indicator or ask user to repeat
        return "transcription_error"
```

## Performance Considerations

When using Whisper in robotics:

- **API Costs**: Monitor usage as each transcription incurs costs
- **Latency**: Consider network delays in real-time applications
- **Bandwidth**: Audio files can be large; consider compression
- **Rate Limits**: Be aware of OpenAI's API rate limits

## Practical Exercise

Create a simple voice command system:

1. Set up your OpenAI API key
2. Create an audio capture function
3. Integrate with Whisper for transcription
4. Map recognized commands to simple robot actions
5. Test with various voice commands

## Summary

In this section, we've covered:
- How to set up OpenAI Whisper for speech recognition
- Techniques for audio capture and preprocessing
- Approaches for real-time vs. batch processing
- Error handling and fallback mechanisms
- Performance considerations for robotics applications

Next, we'll explore how to convert these transcribed commands into actionable intents for your robot.