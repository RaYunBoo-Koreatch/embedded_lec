import RPi.GPIO as GPIO
import time

pinLeds = (37, 36, 40, 38) # 시계방향으로 핀 목록을 작성

GPIO.setmode(GPIO.BOARD) # 핀번호를 BOARD기준으로
for pin in pinLeds:
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW) # 각 핀을 OUTPUT, LOW로 초기화

for pin in pinLeds: # 시계방향 순서로
    GPIO.output(pin, GPIO.HIGH) # LED를 켜고
    time.sleep(1.0) # 1초 후에
    GPIO.output(pin, GPIO.LOW) # LED를 끈다

GPIO.cleanup() # GPIO 출력 설정 초기화