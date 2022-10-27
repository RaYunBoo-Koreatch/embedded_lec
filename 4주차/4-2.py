import RPi.GPIO as GPIO
import time

pinLeds = (26, 16, 21, 20) # 시계방향으로 핀 목록을 작성

GPIO.setmode(GPIO.BCM) # 핀번호를 BCM(GPIO)기준으로
for pin in pinLeds:
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW) # 각 핀을 OUTPUT, LOW로 초기화

try:
    while True: # 계속해서
        for pin in pinLeds: # 시계방향 순서로
            GPIO.output(pin, GPIO.HIGH) # LED를 켜고
            time.sleep(1.0) # 1초 후에
            GPIO.output(pin, GPIO.LOW) # LED를 끈다
except: # 프로그램 강제 종료 시
    #for pin in pinLeds: # 모든 핀의
    #    GPIO.output(pin, GPIO.LOW) # LED를 끈다
    GPIO.cleanup() # GPIO 출력 설정 초기화