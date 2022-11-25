import cv2 as cv
import numpy as np
import threading, time
import SDcar 
import sys
import tensorflow as tf
from tensorflow.keras.models import load_model
import traceback

speed = 80
epsilon = 0.0001

def func_thread():
    i = 0
    while True:
        #print("alive!!")    
        time.sleep(1)
        i = i+1
        if is_running is False:
            break

def collision_detect_thread():
    global frame
    global lock_frame
    global collision
    
    class_names = []
    with open('object_detection_classes_coco.txt', 'r') as f:
        class_names = f.read().split('\n')
    
    #print('class names ', class_names)
    
    COLORS = np.random.uniform(0,255, size=(len(class_names),3))
    #print('class colors ', COLORS)
    
    dnnModel = cv.dnn.readNetFromTensorflow(model='frozen_inference_graph.pb', \
                                            config='ssd_mobilenet_v2_coco_2018_03_29.pbtxt')
    
    print('dnn model ready')
    
    target_names = ['person','car','motorcycle','bus','stop sign','traffic light','bottle','cup']
    
    while True :
        if is_running is False:
            break
    
        if not enable_Collision:
            continue
            pass
        
        #print('dnn model start')
        
        lock_frame.acquire()
        dnnframe = frame.copy()
        lock_frame.release()
        
        ##
        cputime1 = time.perf_counter()
        
        blob = cv.dnn.blobFromImage(image=dnnframe, size=(300,300), swapRB=True)
        dnnModel.setInput(blob)
        output = dnnModel.forward()
        
        ##
        cputime2 = time.perf_counter()
        print('dnn time :', 1/(cputime2-cputime1), 'fps')
        
        detected_counter = 0
        detected = []
        
        collision = False
        
        for detection in output[0,0,:,:]:
            confidence = detection[2]
            if confidence > .4:
                detected_counter += 1
                
                class_id = int(detection[1])
                class_name = class_names[class_id - 1]
                color = COLORS[class_id - 1]
                
                detected.append(class_name)
                if class_name in target_names:
                    collision = True
                
                box_x = int(detection[3] * v_x)
                box_y = int(detection[4] * v_y)
                
                box_w = int(detection[5] * v_x)
                box_h = int(detection[6] * v_y)
                
                cv.rectangle(dnnframe, (box_x, box_y), (box_w, box_h), color, thickness=3)
                cv.putText(dnnframe, class_name, (box_x, box_y+10), cv.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        
        
        print('detected', detected)
        cv.imshow('collision', dnnframe)
        
    

def key_cmd(which_key):
    print('which_key', which_key)
    is_exit = False 
    global enable_AIdrive # assignment가 있는 경우는 global 키워드로 표시
    global enable_Collision
    
    if which_key & 0xFF == 184:
        print('up')
        car.motor_go(speed)
    elif which_key & 0xFF == 178:
        print('down')
        car.motor_back(speed)
    elif which_key & 0xFF == 180:
        print('left')     
        car.motor_left(30)   
    elif which_key & 0xFF == 182:
        print('right')   
        car.motor_right(30)            
    elif which_key & 0xFF == 181:
        car.motor_stop()
        enable_AIdrive = False     
        print('stop')   
    elif which_key & 0xFF == ord('q'):  
        car.motor_stop()
        print('exit')   
        enable_AIdrive = False     
        enable_Collision = False
        is_exit = True    
        print('enable_AIdrive: ', enable_AIdrive)      
            
    elif which_key & 0xFF == ord('e'):  
        enable_AIdrive = True
        print('enable_AIdrive: ', enable_AIdrive)        
    elif which_key & 0xFF == ord('w'):  
        enable_AIdrive = False
        car.motor_stop()
        print('enable_AIdrive: ', enable_AIdrive)    
             
    elif which_key & 0xFF == ord('d'):  
        enable_Collision = True
        print('enable_Collision: ', enable_Collision)        
    elif which_key & 0xFF == ord('s'):  
        enable_Collision = False
        print('enable_Collision: ', enable_Collision)  
        

    return is_exit  

def detect_maskY_HSV(frame):
    crop_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    crop_hsv = cv.GaussianBlur(crop_hsv, (5,5), cv.BORDER_DEFAULT)
    # need to tune params
    mask_Y = cv.inRange(crop_hsv, (25, 50, 100), (35, 255, 255))
    return mask_Y

def detect_maskY_BGR(frame):
    B = frame[:,:,0]
    G = frame[:,:,1]
    R = frame[:,:,2]
    Y = np.zeros_like(G, np.uint8)
    # need to tune params
    Y = G*0.5 + R*0.5 - B*0.7 # 연산 수행 시 float64로 바뀜
    Y = Y.astype(np.uint8)
    Y = cv.GaussianBlur(Y, (5,5), cv.BORDER_DEFAULT)
    # need to tune params
    _, mask_Y = cv.threshold(Y, 100, 255, cv.THRESH_BINARY)
    return mask_Y

def line_tracing(cx):
    #print('cx, ', cx)
    #print('v_x_grid', v_x_grid)
    global moment
    global v_x
    tolerance = 0.1
    diff = 0

    if moment[0] != 0 and moment[1] != 0 and moment[2] != 0:
        avg_m = np.mean(moment)
        diff = np.abs(avg_m - cx) / v_x
    
    #print('diff ={:.4f}'.format(diff))

    if diff <= tolerance:

        moment[0] = moment[1]
        moment[1] = moment[2]
        moment[2] = cx
        print('cx : ', cx)
        if v_x_grid[2] <= cx < v_x_grid[4]:
            car.motor_go(speed) 
            print('go')
        elif v_x_grid[3] >= cx:
            car.motor_left(30) 
            print('turn left')
        elif v_x_grid[1] <= cx:
            car.motor_right(30) 
            print('turn right')
        else:
            print("skip")    
        
    else:
        car.motor_go(speed) 
        print('go')    
        moment = [0,0,0]

def show_grid(img):
    h, _, _ = img.shape
    for x in v_x_grid:
        #print('show_grid', x)
        cv.line(img, (x, 0), (x, h), (0,255,0), 1, cv.LINE_4)

def test_fun(model):
    camera = cv.VideoCapture(0)
    camera.set(cv.CAP_PROP_FRAME_WIDTH,v_x) 
    camera.set(cv.CAP_PROP_FRAME_HEIGHT,v_y)
    ret, frame = camera.read()
    frame = cv.flip(frame,-1)
    cv.imshow('camera',frame)
    crop_img = frame[int(v_y/2):,:]
    crop_img = cv.resize(crop_img, (200, 66))
    crop_img = np.expand_dims(crop_img, 0)
    a = model.predict(crop_img)
    print('okey, a: ', a)

def drive_AI(img):
    global lastSteer
    
    
    global enable_Collision
    
    if enable_Collision:
        global collision
        global lastcollision
        
        if collision:
            if not lastcollision:
                print("collision stop!")
            car.motor_stop()
            return
            
        lastcollision = collision
    
    #print('id', id(model))
    img = np.expand_dims(img, 0)
    res = model.predict(img)[0]
    #print('res', res)
    steering_angle = np.argmax(np.array(res))
    #print('steering_angle', steering_angle)
            
    if steering_angle == 0:
        speedSet = 60
        car.motor_go(speedSet)
    elif steering_angle == 1:
        speedSet = 20
        car.motor_left(speedSet)          
    elif steering_angle == 2:
        speedSet = 20
        car.motor_right(speedSet)
    
    if lastSteer != steering_angle:
        if steering_angle == 0:
            print("go")
        elif steering_angle == 1:
            print("left")       
        elif steering_angle == 2:
            print("right")
        else:
            print("This cannot be entered")
    
    lastSteer = steering_angle

def main():
    global frame
    global lock_frame
    camera = cv.VideoCapture(0)
    camera.set(cv.CAP_PROP_FRAME_WIDTH,v_x) 
    camera.set(cv.CAP_PROP_FRAME_HEIGHT,v_y)
    
    try:
        while( camera.isOpened() ):
            lock_frame.acquire()
            ret, frame = camera.read()
            frame = cv.flip(frame,-1)
            lock_frame.release()
            
            cv.imshow('camera',frame)
            # image processing start here
            crop_img = frame[int(v_y/2):,:]
            crop_img = cv.resize(crop_img, (200, 66))
            cv.imshow('crop_img ', cv.resize(crop_img, dsize=(0,0), fx=2, fy=2))

            if enable_AIdrive == True:
                drive_AI(crop_img)

            # image processing end here
            is_exit = False
            which_key = cv.waitKey(20)
            if which_key > 0:
                is_exit = key_cmd(which_key)    
            if is_exit is True:
                cv.destroyAllWindows()
                break

    except Exception as e:
        traceback.print_exc()
        global is_running
        is_running = False

if __name__ == '__main__':
    global lock_frame
    lastSteer = 0
    v_x = 320
    v_y = 240
    v_x_grid = [int(v_x*i/10) for i in range(1, 10)]
    print(v_x_grid)
    moment = np.array([0, 0, 0])


    model_path = 'my_checkpoint/lane_navigation_20221117_1240.h5'     # tf1.15  
    
    model = load_model(model_path)
    '''print('id', id(model))
    print(model.summary())'''

    #test_fun(model)

    t_task1 = threading.Thread(target = func_thread)
    t_task1.start()
    
    global enable_Collision
    enable_Collision = False
    
    lock_frame = threading.Lock()
    collision_task = threading.Thread(target = collision_detect_thread)
    collision_task.start()


    global lastcollision
    lastcollision = False
    global collision
    collision = False
    

    car = SDcar.Drive()
    
    is_running = True
    enable_AIdrive = False
    main() 
    is_running = False
    car.clean_GPIO()
    print('end vis')
