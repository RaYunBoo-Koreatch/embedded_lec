import RPi.GPIO as GPIO
import time
import random

pinLeds = (26, 16, 21, 20)

GPIO.setmode(GPIO.BCM) # 핀번호를 BCM(GPIO)기준으로
for pin in pinLeds: # 각 핀을
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW) # OUTPUT, LOW로 초기화

try:
    for _ in range(10): # 10번
        #pin = pinLeds[random.randrange(0, len(pinLeds))] # 랜덤한 핀을 골라
        pin = random.choice(pinLeds) # 랜덤한 핀을 골라
        GPIO.output(pin, GPIO.HIGH) # LED를 켜고
        time.sleep(0.5) # 0.5초 후에
        GPIO.output(pin, GPIO.LOW) # LED를 끈다
finally:
    GPIO.cleanup() # 핀 설정 초기화