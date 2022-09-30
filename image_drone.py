import sys
import traceback
import cv2
import numpy as np
import time
import detector
import djitellopy as tellopy
import Target_positioning as T
import drone_action as D
import yolov5.object_detector as O
from yolov5.utils.torch_utils import select_device

WIDTH = 960
HEIGHT = 720

def main():
    drone = tellopy.Tello()
    device = select_device('')

    # import torch
    # model = torch.hub.load('icns-distributed-cloud/yolov5-svhn', 'svhn').fuse().eval()
    # model = model.autoshape()

# Prediction


        
    global image
    try:
        drone.connect()
        frame_skip = 300
        drone.streamon()
        drone.set_video_bitrate(0)
        # drone.set_video_fps('middle')
        # drone.set_video_resolution('low')

        frame_read = drone.get_frame_read()
        objectBox = {"detailInfo" : {"baseYaw" : drone.get_yaw(), "colorSeq" : [], 'hoverQR' : None}, detector.ENUM_BLUE : None, detector.ENUM_BLACK : None, detector.ENUM_RED : None, detector.ENUM_QR : None}
        i = 0 
        # drone.takeoff()
        while True:
                time.sleep(0.01)
                i+=1
                # if(drone.get_height() > 70):
                #     drone.send_rc_control(0,0,-15,0)
                # else:
                #     drone.send_rc_control(0,0,10,0)

                frame = frame_read.frame
                print(frame.shape)
                # prediction = model(frame, size=640)
                # print(prediction)
                # for x1, y1, x2, y2, conf, clas in prediction:
                #     print('box: ({}, {}), ({}, {})'.format(x1, y1, x2, y2))
                #     print('confidence : {}'.format(conf))
                #     print('class: {}'.format(int(clas)))
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                lower_black = np.array([0//2,0, 0])
                upper_black = np.array([360//2, 2*255//3, 60])
                maskedImage = cv2.inRange(hsv, lower_black, upper_black)
                cv2.imshow("black", maskedImage)
                
                # frame_1 = frame.copy()
                # marker = O.main_detect(frame_1.transpose((2,0,1)), './yolov5/number_test.pt', device, conf_score=0.6)
                # if(len(marker) != 0):
                #     for m in marker:
                #         label, p = m[-1].split()
                #         m = list(map(int,m[:4]))
                #         if(m[2]-m[0] > 100 or m[3] - m[1] > 100):
                #             continue
                #         # print(marker)
                #         cv2.rectangle(frame_1, (m[0],m[1]),(m[2],m[3]), (255,0,255),7)
                #         cv2.putText(frame_1, label, (m[0],m[1]+50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 5)
                #         print(f"Label : {label} \t Prob : {p}")
                # cv2.imshow("test", frame_1)

                frame_2 = frame.copy()
                marker = O.main_detect(frame_2.transpose((2,0,1)), './weight/number_best.pt', device)
                if(len(marker) != 0):
                    for m in marker:
                        label, p = m[-1].split()
                        m = list(map(int,m[:4]))
                        # print(marker)
                        cv2.rectangle(frame_2, (m[0],m[1]),(m[2],m[3]), (255,0,255),7)
                        cv2.putText(frame_2, label, (m[0],m[1]+50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 5)
                        print(f"Label : {label} \t Prob : {p}")
                cv2.imshow("good", frame_2)
                
                # objectBox, colorDetectionBox, frame = detector.detectObjects(frame, objectBox, [detector.ENUM_BLACK, detector.ENUM_BLUE, detector.ENUM_RED],drone)
                # if(colorDetectionBox[detector.ENUM_BLUE] is not None):
                #     rc = colorDetectionBox[detector.ENUM_BLUE][-1]
                #     points = np.array([[rc[0],rc[1]], [rc[0]+rc[2], rc[1]+rc[3]]])
                #     points[:,0] = points[:,0]/WIDTH * 2000
                #     points[:,1] = points[:,1]/HEIGHT * 2000
                #     left_up = points[0]
                #     left_down = points[1]
                #     marker_location = T.marker_location_from_body(left_up, left_down,0.25)
                #     print('marker location: ', marker_location)

                cv2.waitKey(1)
                # if(i%15 == 0):
                if(objectBox[detector.ENUM_QR] is not None):
                    print(objectBox[detector.ENUM_QR][-1][0].rect[-1])
                    print(objectBox)

    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)
    finally:
        cv2.destroyAllWindows()
if __name__ == '__main__':
    main()