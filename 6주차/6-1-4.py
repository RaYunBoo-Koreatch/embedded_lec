import RPi.GPIO as GPIO
import time

pinSwitch = (5, 6, 13, 19)

GPIO.setmode(GPIO.BCM) # 핀번호를 BCM(GPIO)기준으로
for pin in pinSwitch: # 각 핀을
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # INPUT, PULLDOWN 으로 초기화

try:
    lastSwitch = [0] * len(pinSwitch) # 이전 상태를 보관
    counter = [0] * len(pinSwitch) # 클릭 횟수 보관
    while True: # 무한 반복
        for i, pin in enumerate(pinSwitch): # 각 핀에 대해
            swVal = GPIO.input(pin) # 버튼 입력을 받고
            
            if(lastSwitch[i] == 0 and swVal == 1): # Rising에만 동작
                counter[i] += 1 # 클릭 횟수 증가
                print("('SW%d click', %d)" % (i+1, counter[i])) # 출력
            
            lastSwitch[i] = swVal # 이전 상태 업데이트
        
        time.sleep(0.05) # 채터링 감소
except KeyboardInterrupt: # 인터럽트 처리
    pass

GPIO.cleanup() # 핀 설정 초기화