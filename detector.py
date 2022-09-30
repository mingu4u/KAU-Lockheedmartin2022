import cv2
import numpy as np
from pyzbar.pyzbar import decode
import os


WIDTH = 960
HEIGHT = 720


ENUM_BLACK = 0
ENUM_RED = 1
ENUM_BLUE = 2
ENUM_QR = 3
ENUM_KAU = 4
ENUM_F22 = 5
ENUM_NUMBER = 6

COLOR_TEXT = {
    ENUM_BLUE : "Blue",
    ENUM_BLACK : "Black",
    ENUM_RED : "Red"
}

def qrDetect(frame):
    data = decode(frame)
    return data

def calContourHSV(color, frame):
    colorExclude = [0,1,2]
    colorExclude.pop(color)
    # print(f"Blue : {frame[-1,-1,0]} \t Green : {frame[-1,-1,1]} \t Red : {frame[-1,-1,2]} ")


    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    

    if(color == ENUM_RED):
        lower_red1 = np.array([0//2, 255//3, 40])
        upper_red1 = np.array([15//2, 255, 200])

        lower_red2 = np.array([345//2, 255//3, 40])
        upper_red2 = np.array([360//2, 255, 200])

        rowMask = cv2.inRange(hsv, lower_red1, upper_red1)
        upperMask = cv2.inRange(hsv, lower_red2, upper_red2)

        maskedImage = cv2.bitwise_or(rowMask, upperMask)
        # maskedImage = img
    
    elif(color == ENUM_BLUE):
        lower_blue = np.array([225//2,255//3, 30])
        upper_blue = np.array([255//2, 255, 200])
        maskedImage = cv2.inRange(hsv, lower_blue, upper_blue)
        
    elif(color == ENUM_BLACK):
        lower_black1 = np.array([20//2,0, 0])
        upper_black1 = np.array([225//2, 255//2, 50])

        lower_black2 = np.array([260//2, 0, 0])
        upper_black2 = np.array([340//2, 255//2, 50])

        rowMask = cv2.inRange(hsv, lower_black1, upper_black1)
        upperMask = cv2.inRange(hsv, lower_black2, upper_black2)

        maskedImage = cv2.bitwise_or(rowMask, upperMask)


    contours, _ = cv2.findContours(maskedImage, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if(color == ENUM_BLACK):
        new_contours = []
        for c in contours:
            rect = list(cv2.boundingRect( c ))
            margin = 5
            rect [0] += margin
            rect [1] += margin
            rect [2] -= margin*2
            rect [3] -= margin*2
            (x1,y1), (x1,y2), (x2,y1), (x2,y2) = (rect[0],rect[1]), (rect[0],rect[1] + rect[3]), (rect[0]+rect[2],rect[1]), (rect[0]+rect[2],rect[1]+rect[3])
            points = np.array([(x1,y1),(x1,y2),(x2,y1),(x2,y2)])
            
            points[:,0][points[:,0] > WIDTH - 1] = WIDTH - 1
            points[:,1][points[:,1] > HEIGHT - 1] = HEIGHT - 1
            whitePoint = []
            for i in range(4):
                if(frame[points[i][1], points[i][0]].sum() > 350):
                    whitePoint.append(frame[points[i][1], points[i][0]].sum())

            if(len(whitePoint) < 4):
                # print(frame[points[0][1], points[0][0]].sum(),frame[points[1][1], points[1][0]].sum(),frame[points[2][1], points[2][0]].sum(),frame[points[3][1], points[3][0]].sum())
                continue
            new_contours.append(c)
        contours = new_contours


    if(len(contours) != 0):  
        c = max(contours, key=cv2.contourArea)
        if(cv2.contourArea(c) > 5000):
            (x,y),r = cv2.minEnclosingCircle(c)
            rect = cv2.boundingRect( c )

            return (x,y,r, cv2.contourArea(c), rect)
        else:
            return None
    else:
        return None



def getCircle(drone, frame, colors):
    frame_copy = frame.copy()
    contourDict = {}
    
    if(len(colors) == 0):
        contourDict = {ENUM_BLUE : None, ENUM_BLACK : None, ENUM_RED : None}
    else:
        for color in [ENUM_BLUE, ENUM_BLACK, ENUM_RED]:
            contourDict[color] = None

        for color in colors:
            circle = calContourHSV(color, frame)
            if(circle is None):
                contourDict[color] = None
                continue

            saveFrame = frame.copy()
            margin = 5
            (x,y,r,c) = map(int, circle[:-1])
            rect = list(map(int,circle[-1]))

            cv2.circle(frame_copy, (int(circle[0]), int(circle[1])), int(circle[2]), (0, 255, 255), 5)
            cv2.circle(saveFrame, (int(circle[0]), int(circle[1])), int(circle[2]), (0, 255, 255), 5)


            cv2.putText(frame_copy, COLOR_TEXT[color], (x,y+50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 5)
            cv2.putText(saveFrame, COLOR_TEXT[color], (x,y+50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 5)

            if(drone is None):
                contourDict[color] = (x,y,r, 0, rect)
            else:
                contourDict[color] = (x,y,r, drone.get_yaw() , rect)
            cv2.rectangle(frame_copy, (rect[0],rect[1]), (rect[0]+rect[2], rect[1]+rect[3]), (255,255,0), 5)
            
    return contourDict, frame_copy

def limitDetection(drone, frame, objectBox, colorDetectBox, color):
    if(colorDetectBox[color] is not None and objectBox[color] is None):
        if(colorDetectBox[color][0] < WIDTH/5 or colorDetectBox[color][0] > WIDTH*4/5):
            objectBox[color] = None
        else:
            print(f"{COLOR_TEXT[color]} is Detected")
            objectBox[color] = [drone.get_height(), drone.get_yaw(),colorDetectBox[color][-1]]

            count = len(os.listdir("./image"))
            cv2.imwrite(f"./image/{count//5} {COLOR_TEXT[color]}.png", frame)
            
            print(objectBox)

    return objectBox
        

def detectObjects(frame, objectBox, colorObject,drone):
    colorDetectBox, frame = getCircle(drone, frame,colorObject)
    qrdata = qrDetect(frame.copy())
    objectBox = limitDetection(drone, frame,objectBox, colorDetectBox, ENUM_RED)
    objectBox = limitDetection(drone, frame, objectBox, colorDetectBox, ENUM_BLACK)
    objectBox = limitDetection(drone, frame, objectBox, colorDetectBox, ENUM_BLUE)

    if(len(qrdata) > 0):
        if(objectBox['detailInfo']['hoverQR'] is None):
            objectBox['detailInfo']['hoverQR'] = qrdata[0].data
        objectBox[ENUM_QR] = [drone.get_height(), drone.get_yaw(), qrdata]
    else:
        objectBox[ENUM_QR] = None

    return objectBox, colorDetectBox, frame

