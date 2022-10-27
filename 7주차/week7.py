import threading
import serial
import time
import RPi.GPIO as GPIO

PWMA, AIN1, AIN2 = 18, 22, 27
PWMB, BIN1, BIN2 = 23, 25, 24


GPIO.setmode(GPIO.BCM) # 핀번호를 BCM(GPIO)기준으로
GPIO.setup(PWMA, GPIO.OUT) # OUTPUT으로 초기화
GPIO.setup(AIN1, GPIO.OUT) # OUTPUT으로 초기화
GPIO.setup(AIN2, GPIO.OUT) # OUTPUT으로 초기화
GPIO.setup(PWMB, GPIO.OUT) # OUTPUT으로 초기화
GPIO.setup(BIN1, GPIO.OUT) # OUTPUT으로 초기화
GPIO.setup(BIN2, GPIO.OUT) # OUTPUT으로 초기화

# 모터 초기화 1
GPIO.output(AIN1, 0)
GPIO.output(AIN2, 1)
GPIO.output(BIN1, 0)
GPIO.output(BIN2, 1)

# 모터 초기화 2
L_Motor = GPIO.PWM(PWMA, 500)
L_Motor.start(0)
R_Motor = GPIO.PWM(PWMB, 500)
R_Motor.start(0)

# 모터 제어 함수
def L(dutycycle):
    if(dutycycle >= 0): # 양수일 경우 전진
        GPIO.output(AIN1, 0)
        GPIO.output(AIN2, 1)
    else: # 음수일 경우 후진
        GPIO.output(AIN1, 1)
        GPIO.output(AIN2, 0)
    L_Motor.ChangeDutyCycle(abs(dutycycle))

def R(dutycycle):
    if(dutycycle >= 0): # 양수일 경우 전진
        GPIO.output(BIN1, 0)
        GPIO.output(BIN2, 1)
    else: # 음수일 경우 후진
        GPIO.output(BIN1, 1)
        GPIO.output(BIN2, 0)
    R_Motor.ChangeDutyCycle(abs(dutycycle))

def LR(ld, rd):
    L(ld)
    R(rd)

throttle = 50

def go():
    LR(throttle, throttle)

def right():
    LR(throttle, -throttle)

def left():
    LR(-throttle, throttle)

def back():
    LR(-throttle, -throttle)

def stop():
    LR(0, 0)

# 블루투스 시리얼
bleSerial = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1.0)

# 블루투스 입력 데이터
gData = ""

# 블루투스 스레드
def serial_thread():
    global gData
    while True:
        data = bleSerial.readline() # 한줄을 읽어
        data = data.decode() # 디코드 후
        gData += data # 입력

def main():
    global gData
    global throttle
    last = stop
    try:
        while True:
            if gData.find("go") >= 0: # 앞으로
                gData = ""
                last = go
                go()
            elif gData.find("back") >= 0: # 뒤로
                gData = ""
                last = back
                back()
            elif gData.find("left") >= 0: # 좌회전
                gData = ""
                last = left
                left()
            elif gData.find("right") >= 0: # 우회전
                gData = ""
                last = right
                right()
            elif gData.find("stop") >= 0: # 정지
                gData = ""
                last = stop
                stop()
            elif gData.find("slow") >= 0: # 정지
                gData = ""
                if throttle == 100:
                    throttle = 50
                else:
                    throttle = 100
                last()
    except KeyboardInterrupt: # 인터럽트 처리
        pass


if __name__ == '__main__':
    task1 = threading.Thread(target=serial_thread)
    task1.start() # 블루투스 입력 스레드 실행

    main() # 차량 이동 코드

    bleSerial.close() # 블루투스 연결 해제
    GPIO.cleanup() # 핀 설정 초기화
    print("program end")
