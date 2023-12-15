#!/usr/bin/env python3

"""
Basic audio streaming example.

This example shows how to stream the audio data from the TTS engine,
and how to get the WordBoundary events from the engine (which could
be ignored if not needed).

The example streaming_with_subtitles.py shows how to use the
WordBoundary events to create subtitles using SubMaker.
"""

import asyncio
import rospy
import edge_tts

from audio_common_msgs.msg import AudioData


TEXT = "سلام خوبی چطوری"
VOICE = "Microsoft Server Speech Text to Speech Voice (fa-IR, DilaraNeural)"
OUTPUT_FILE = "test.mp3"

rospy.init_node('text_to_speech_nodee',anonymous=False)
pub = rospy.Publisher('audio/audio', AudioData, queue_size=10)
msg = AudioData()

async def amain() -> None:
    """Main function"""
    communicate = edge_tts.Communicate(TEXT, VOICE)
    with open(OUTPUT_FILE, "wb") as file:
        async for chunk in communicate.stream():
            if chunk["type"] == "audio":
                file.write(chunk["data"])
                msg.data = chunk["data"]
                pub.publish(msg)
            elif chunk["type"] == "WordBoundary":
                print(f"WordBoundary: {chunk}")


if __name__ == "__main__":
    loop = asyncio.get_event_loop_policy().get_event_loop()
    try:
        loop.run_until_complete(amain())
    finally:
        loop.close()