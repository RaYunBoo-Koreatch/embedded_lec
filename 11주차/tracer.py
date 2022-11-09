
import cv2 as cv
import numpy as np
import threading, time
import SDcar
import traceback
import time

enable_auto = False

def func_thread():
    i = 0
    while True:
        #print("alive!!")    
        time.sleep(1)
        i = i+1
        if is_running is False:
            break

def key_cmd(which_key):
    global enable_auto
    print('which_key', which_key)
    is_exit = False
    if which_key & 0xFF == 184:
        print('up')
        car.motor_go(speed)
    elif which_key & 0xFF == 178:
        print('down')
        car.motor_back(speed)
    elif which_key & 0xFF == 180:
        print('left')
        car.motor_left(speed)   
    elif which_key & 0xFF == 182:
        print('right')
        car.motor_right(speed)
    elif which_key & 0xFF == 181:
        car.motor_stop()
        print('stop')
    elif which_key & 0xFF == ord('q'):
        car.motor_stop()
        print('exit')
        is_exit = True
        
    elif which_key & 0xFF == ord('w'):
        enable_auto = True
        print('auto')
        
    elif which_key & 0xFF == ord('e'):
        enable_auto = False
        car.motor_stop()
        print('man')
    
    return is_exit  

def maskYellow_BGR(frame):
    B, G, R = frame[:,:,0], frame[:,:,1], frame[:,:,2] 
    #Y = np.zeros_like(B, np.uint8)
    Y = G*0.5 + R*0.5 - B*0.7
    Y = Y.astype(np.uint8) 
    Y = cv.GaussianBlur(Y, (5,5), cv.BORDER_DEFAULT)
    _, maskY = cv.threshold(Y, 100, 255, cv.THRESH_BINARY)
    return maskY

def maskYellow_HSV(frame):
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    #hsv = cv.GaussianBlur(hsv, (7,7), cv.BORDER_DEFAULT)
    maskY = cv.inRange(hsv, (25,50,100), (35, 255,255))
    maskY = cv.dilate(maskY, np.ones((3,3),np.uint8), iterations=2)
    return maskY

maskYellow = maskYellow_BGR
eps = 1e-6
speed = 30


def main():
    global enable_auto
    camera = cv.VideoCapture(0)
    camera.set(cv.CAP_PROP_FRAME_WIDTH,v_x) 
    camera.set(cv.CAP_PROP_FRAME_HEIGHT,v_y)
    x_grid = [int(v_x*i/20) for i in range(1,20)]
    
    try:
        lastframetime = time.perf_counter()
        lastCenter = (0, 0)
        lastMove = 'stop'
        while( camera.isOpened() ):
            ret, frame = camera.read()
            frame = cv.flip(frame,-1)
            cv.imshow('camera',frame)

            # ============================
            # image processing start here
            # ============================
            
            
            # yellow mask
            crop_frame = frame[120:,:]
            maskY = maskYellow_HSV(crop_frame)
            cv.imshow('maskY', maskY)
            
            # contours
            contours, _ = cv.findContours(maskY, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            contours = sorted(contours, key=cv.contourArea, reverse=True)
            contours = list(filter(lambda c:cv.contourArea(c)>20, contours))
            
            # moment
            moments = []
            for contour in contours:
                M = cv.moments(contour)
                mX = int(M['m10'] / (M['m00']+eps))
                mY = int(M['m01'] / (M['m00']+eps))
                moments.append((mX, mY))
            
            # center
            if len(moments) >= 1:
                #avgx = int((moments[0][0] + moments[1][0])/2)
                #avgy = int((moments[0][1] + moments[1][1])/2)
                avgx, avgy = moments[0][0], moments[0][1]
                lastCenter = (avgx, avgy)
            
            if enable_auto:
                if len(moments) >= 1:
                    if lastCenter[0] < x_grid[0+8]:
                        car.motor_left(speed)
                        lastMove = 'left'
                    elif lastCenter[0] > x_grid[20-8]:
                        car.motor_right(speed)
                        lastMove = 'right'
                    else:
                        car.motor_go(speed)
                        lastMove = 'go'
                    print(lastMove)
                else:
                    print(lastMove, 'missing')
            
                    
            
            # draw contours
            cv.drawContours(crop_frame, contours, -1, (0,255,0), 3)
            
            # draw moments
            for i, moment in enumerate(moments):
                cv.circle(crop_frame, moment, int(10*0.5**i), (0,0,255), -1)
            
            # draw center
            cv.circle(crop_frame, lastCenter, 10, (0,255,0), -1)
            
            # show
            cv.imshow('cropped', crop_frame)
                    
                
            
            
            frametime = time.perf_counter()
            fps = 1.0/(frametime-lastframetime)
            print("fps = %.5f" % (fps))
            lastframetime = frametime
            # ============================
            # image processing end here
            # ============================
            is_exit = False
            which_key = cv.waitKey(20)
            if which_key > 0:
                is_exit = key_cmd(which_key)    
            if is_exit is True:
                cv.destroyAllWindows()
                break
    except Exception as e:
        print(e)
        traceback.print_exc()
        global is_running
        is_running = False

if __name__ == '__main__':

    v_x = 320
    v_y = 240

    t_task1 = threading.Thread(target = func_thread)
    t_task1.start()

    car = SDcar.Drive()
    
    is_running = True
    main() 
    is_running = False
    car.clean_GPIO()

