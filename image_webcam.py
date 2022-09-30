from detector import getCircle
import detector
import cv2
import Target_positioning as T
import torch
import time
import djitellopy as tellopy
import Target_positioning as T
import drone_action as D
import yolov5.object_detector as O
from yolov5.utils.torch_utils import select_device

WIDTH = 960
HEIGHT = 720


if __name__ == '__main__':

    # Load model
    device = select_device('')

    cap = cv2.VideoCapture(0)
    i = 0
    # model = torch.hub.load('icns-distributed-cloud/yolov5-svhn', 'svhn').fuse().eval()
    # model = model.autoshape()
    pre = time.time()
    while True:
        i+=1
        ret, frame = cap.read()
        # recPos = getCircle(None, frame, [detector.ENUM_BLACK, detector.ENUM_BLUE,detector.ENUM_RED], 5000, WIDTH, HEIGHT)

        # prediction = model(frame, size=640)
        # print(prediction)
        # for x1, y1, x2, y2, conf, clas in prediction:
        #     print('box: ({}, {}), ({}, {})'.format(x1, y1, x2, y2))
        #     print('confidence : {}'.format(conf))
        #     print('class: {}'.format(int(clas)))
        
        m1 = frame.copy()
        marker = O.main_detect(m1.transpose((2,0,1)), './yolov5/best_test.pt',device)
        if(len(marker) != 0):
            for m in marker:
                label, p = m[-1].split()
                m = list(map(int,m[:4]))
                # print(marker)
                cv2.rectangle(m1, (m[0],m[1]),(m[2],m[3]), (255,0,255),7)
                cv2.putText(m1, label, (m[0],m[1]+50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 5)
        cv2.imshow("good", m1)


        m2 = frame.copy()
        marker = O.main_detect(m2.transpose((2,0,1)), './yolov5/number_best.pt',device)
        if(len(marker) != 0):
            for m in marker:
                label, p = m[-1].split()
                m = list(map(int,m[:4]))
                # print(marker)
                cv2.rectangle(m2, (m[0],m[1]),(m[2],m[3]), (255,0,255),7)
                cv2.putText(m2, label, (m[0],m[1]+50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 5)

        m2 = m2[:720,:960]
        rect = [693, 428, 112, 110]
        cv2.rectangle(m2, (rect[0], rect[1]), (rect[0]+rect[2], rect[1]+rect[3]), (0,255,255),7)
        rect = [667, 502, 101, 146]
        cv2.rectangle(m2, (rect[0], rect[1]), (rect[0]+rect[2], rect[1]+rect[3]), (0,255,255),7)
        cv2.imshow("git", m2)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # if(i%15==0):
    post = time.time()
    print("time : ",post - pre)