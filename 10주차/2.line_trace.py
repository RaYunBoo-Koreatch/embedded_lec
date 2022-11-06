import numpy as np
import cv2


file = "imgs/3.jpg"

scale = 0.5
too_small_white = 100
too_large_white = 3000
too_small_yellow = 50
border = 10



# 파일 읽어오기
img = cv2.imread(file)
img = cv2.resize(img, dsize=(int(img.shape[1]*scale), int(img.shape[0]*scale)))

width = img.shape[1]
height = img.shape[0]


# 노란색 추출
yellow_lower = np.array([0, 192, 192])
yellow_upper = np.array([192, 255, 255])

yellow_mask = cv2.inRange(img, yellow_lower, yellow_upper)


# 노란색에서 불필요한 부분 삭제
retval, labels, stats, centroids = cv2.connectedComponentsWithStats(yellow_mask)
for i in range(retval):
    n = stats[i][4]
    if n < too_small_yellow:
        # 너무 작은 노란색 제거
        yellow_mask[labels==i] = 0
        
    else:
        #cv2.rectangle(yellow_mask, (x, y), (x + w, y + h), (255), 1)
        pass


# 하얀색 추출
white_lower = np.array([208, 208, 208])
white_upper = np.array([255, 255, 255])

white_mask = cv2.inRange(img, white_lower, white_upper)


# 하얀색에서 불필요한 부분 삭제
retval, labels, stats, centroids = cv2.connectedComponentsWithStats(white_mask)

for i in range(retval):
    x = stats[i][0]
    y = stats[i][1]
    w = stats[i][2]
    h = stats[i][3]
    n = stats[i][4]
    
    if x < border or y < border or x+w > width-border or y+h >= width-border: 
        # 경계에 붙어있는 하얀색 제거
        white_mask[labels==i] = 0
        
    elif n < too_small_white:
        # 너무 작은 하얀색 제거
        white_mask[labels==i] = 0
        
    elif n > too_large_white:
        # 너무 큰 하얀색 제거
        white_mask[labels==i] = 0
        
    else:
        #cv2.rectangle(white_mask, (x, y), (x + w, y + h), (255), 1)
        pass


# 이미지 합성
yellow_img = cv2.copyTo(img, yellow_mask)
white_img = cv2.copyTo(img, white_mask)
result = cv2.add(white_img, yellow_img)


# 이미지 출력
cv2.imshow('result',result)
cv2.imwrite("%s.trace.jpg"%(file), result)
cv2.imwrite("%s.mask.yellow.jpg"%(file), yellow_mask)
cv2.imwrite("%s.mask.white.jpg"%(file), white_mask)

cv2.waitKey(0)
cv2.destroyAllWindows()
