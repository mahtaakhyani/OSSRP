#!/usr/bin/env python3

"""
Example of dynamic voice selection using VoicesManager.
"""

import asyncio
import os
import random
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import edge_tts as edge_tts
from edge_tts import VoicesManager

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# from recognition.stt import transcript
from hazmtest import FarsGPT2

fagpt = FarsGPT2('gpt2_fa')
inputs =  "ینی واقعا شدنیه که"
output = fagpt.query({
	"inputs":inputs,
})
print(output[0]['generated_text'])

TEXT = str(output[0]['generated_text']).replace(inputs+'))). --','')
OUTPUT_FILE = "hazm.wav"
NAME = "Microsoft Server Speech Text to Speech Voice (fa-IR, DilaraNeural)"

async def _main() -> None:
    voices = await VoicesManager.create()
    # voice = voices.find(Gender="Female", Language="fa")
    voice = voices.find(Name = NAME)
    print(voice,'\n\n',TEXT)
    # Also supports Locales
    # voice = voices.find(Gender="Female", Locale="es-AR") #Automated language-based voice selection

    communicate = edge_tts.Communicate(TEXT, random.choice(voice)["Name"])
    # communicate = edge_tts.Communicate(TEXT, voice)
    await communicate.save(OUTPUT_FILE)


if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(_main())
