from gtts import gTTS
import os

FILENAME = "ask"
AUDIO_FORMAT = ".mp3"

# mytext = 'Welcome to AI for robotics!'
# language = 'en'

mytext = '''
    สวัสดีงับ ผมเป็นturtlebotที่โคตรฉลาด
    วันนี้ คุณอยากดื่มอะไรเป็นพิเศษมั้ย งับ 
    ถ้านึกไม่ออก ก็แดกส้นตีนแทนได้นะงับ งุงิ
'''

language = 'th'

myobj = gTTS(text=mytext, lang=language, slow=False)

myobj.save(FILENAME + AUDIO_FORMAT)

print("----- Start speaking...")
os.system(f'mpg123 {FILENAME}{AUDIO_FORMAT}')
print("----- Finished speaking...")