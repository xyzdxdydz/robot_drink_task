#!/usr/bin/env python3
from gtts import gTTS
import os

class Utils(object):
    def __init__(self) -> None:
        self.AUDIO_FORMAT = ".mp3"
        
    def speak(self, topic, document):
        language = 'th'

        myobj = gTTS(text=document, lang=language, slow=False)
        myobj.save(topic + self.AUDIO_FORMAT)

        print("----- Start speaking...")
        os.system(f'mpg123 {topic}{self.AUDIO_FORMAT}')
        print("----- Finished speaking...")