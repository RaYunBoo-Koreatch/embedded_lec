import RPi.GPIO as GPIO
import time

PWMA, AIN1, AIN2 = 18, 22, 27
PWMB, BIN1, BIN2 = 23, 25, 24
pinSwitch = (5, 6, 13, 19)


GPIO.setmode(GPIO.BCM) # 핀번호를 BCM(GPIO)기준으로
GPIO.setup(PWMA, GPIO.OUT) # OUTPUT으로 초기화
GPIO.setup(AIN1, GPIO.OUT) # OUTPUT으로 초기화
GPIO.setup(AIN2, GPIO.OUT) # OUTPUT으로 초기화
GPIO.setup(PWMB, GPIO.OUT) # OUTPUT으로 초기화
GPIO.setup(BIN1, GPIO.OUT) # OUTPUT으로 초기화
GPIO.setup(BIN2, GPIO.OUT) # OUTPUT으로 초기화
for pin in pinSwitch: # 각 핀을
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # INPUT, PULLDOWN 으로 초기화


GPIO.output(AIN1, 0)
GPIO.output(AIN2, 1)
GPIO.output(BIN1, 0)
GPIO.output(BIN2, 1)

L_Motor = GPIO.PWM(PWMA, 500)
L_Motor.start(0)
R_Motor = GPIO.PWM(PWMB, 500)
R_Motor.start(0)

def L(dutycycle):
    if(dutycycle >= 0):
        GPIO.output(AIN1, 0)
        GPIO.output(AIN2, 1)
    else:
        GPIO.output(AIN1, 1)
        GPIO.output(AIN2, 0)
    L_Motor.ChangeDutyCycle(abs(dutycycle))

def R(dutycycle):
    if(dutycycle >= 0):
        GPIO.output(BIN1, 0)
        GPIO.output(BIN2, 1)
    else:
        GPIO.output(BIN1, 1)
        GPIO.output(BIN2, 0)
    R_Motor.ChangeDutyCycle(abs(dutycycle))

def LR(ld, rd):
    L(ld)
    R(rd)

def forward():
    LR(100, 100)

def right():
    LR(100, -100)

def left():
    LR(-100, 100)

def back():
    LR(-100, -100)

def stop():
    LR(0, 0)

directions = (forward, right, left, back) # 전 우 좌 후

try:
    while True: # 무한 반복
        for i, pin in enumerate(pinSwitch): # 각 핀에 대해
            if(GPIO.input(pin) == 1): #버튼이 입력되었으면
                directions[i]() # 방향 설정
                time.sleep(0.05) # 채터링 감소
                while(GPIO.input(pin) == 1): pass # 버튼을 뗄때까지 대기
                stop() # 정지
                time.sleep(0.05) # 채터링 감소
except KeyboardInterrupt: # 인터럽트 처리
    pass

GPIO.cleanup() # 핀 설정 초기화