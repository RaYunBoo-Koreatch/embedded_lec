import RPi.GPIO as GPIO
import time

pinBuzzer = 12 # GPIO12
frequency = (262, 294, 330, 392)
pinSwitch = (5, 6, 13, 19)

GPIO.setmode(GPIO.BCM) # 핀번호를 BCM(GPIO)기준으로
for pin in pinSwitch: # 각 핀을
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # INPUT, PULLDOWN 으로 초기화
GPIO.setup(pinBuzzer, GPIO.OUT) # OUTPUT으로 초기화

p = GPIO.PWM(pinBuzzer, frequency[0]) # pwm 호출

try:
    while True: # 무한 반복
        for i, pin in enumerate(pinSwitch): # 각 핀에 대해
            if(GPIO.input(pin) == 1): #버튼이 입력되었으면
                p.ChangeFrequency(frequency[i]) # 음계를 설정
                p.start(50) # duty cycle = 50%
                time.sleep(0.05) # 채터링 감소
                while(GPIO.input(pin) == 1): pass # 버튼을 뗄때까지 대기
                time.sleep(0.05) # 채터링 감소
                p.stop()

            # if(lastSwitch[i] == 0 and swVal == 1): # Rising에만 동작
            #     p.ChangeFrequency(frequency[i]) # 음계를 설정
            #     p.start(50) # duty cycle = 50%
            # if(lastSwitch[i] == 1 and swVal == 0): # Falling에만 동작
            #     p.stop() # 부저 정지
        
        time.sleep(0.05) # 채터링 감소
except KeyboardInterrupt: # 인터럽트 처리
    pass

GPIO.cleanup() # 핀 설정 초기화