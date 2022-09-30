import sys
import traceback
import cv2
import detector
import djitellopy as tellopy
import drone_action as D
import yolov5.object_detector as O
from yolov5.utils.torch_utils import select_device
import os
import time


def main():
    drone = tellopy.Tello()
    preMission = None
    currentMission = D.MISSION_TAKEOFF
    device = select_device('')
            


    MISSION_STATE = {
        D.MISSION_TAKEOFF : None,
        D.MISSION_MOVE_CENTER : None,
        D.MISSION_DETECT : None,
        D.MISSION_MOVE_BLACK : None,
        D.MISSION_MOVE_RED : None,
        D.MISSION_MOVE_BLUE : None,

        D.MISSION_QR_BLACK : None,
        D.MISSION_QR_RED : None,
        D.MISSION_QR_BLUE : None,

        D.MISSION_TRACKING : None,
        D.MISSION_TRANS_2to3 : None,

        D.MISSION_LANDING : None,
        D.MISSION_DONE : None

    }

    try:
        drone.connect()
        drone.streamon()
        drone.set_video_fps('high')
        drone.set_video_bitrate(2)
        frame_read = drone.get_frame_read()
        objectBox = {"detailInfo" : {"baseYaw" : drone.get_yaw(), "colorSeq" : [], 'hoverQR' : None}, detector.ENUM_BLUE : None, detector.ENUM_BLACK : None, detector.ENUM_RED : None, detector.ENUM_QR : None, detector.ENUM_NUMBER : None}
        objectBox['duration'] = 0
        durationIntervalInfo = {D.MISSION_MOVE_BLACK : 60*4, D.MISSION_MOVE_RED : 60*4, D.MISSION_MOVE_BLUE : 60*4}
        durationInterval = 1
        drone.send_rc_control(0.,0.,0.,0)
        

        while True:
            # time.sleep(0.03)
            colorObject = []

            if(currentMission == D.MISSION_DETECT and preMission != D.MISSION_DETECT):
                objectBox = {"detailInfo" : {"baseYaw" : drone.get_yaw(), "colorSeq" : [], 'hoverQR' : None}, detector.ENUM_BLUE : None, detector.ENUM_BLACK : None, detector.ENUM_RED : None, detector.ENUM_QR : None, detector.ENUM_NUMBER:None}
                objectBox['duration'] = 0
            if(currentMission < D.MISSION_DETECT):
                colorObject = [detector.ENUM_BLACK,detector.ENUM_RED,detector.ENUM_BLUE]
            elif(D.MISSION_DETECT == currentMission):
                colorObject = [detector.ENUM_BLACK]
            elif(D.MISSION_MOVE_RED == currentMission):
                colorObject = [detector.ENUM_RED]
            elif(D.MISSION_MOVE_BLUE == currentMission):
                colorObject = [detector.ENUM_BLUE]
            else:
                colorObject = []
                
            frame = frame_read.frame
            
            objectBox, colorDetectionBox, frame_show = detector.detectObjects(frame, objectBox, colorObject,drone)

            if(currentMission!=preMission and currentMission in durationIntervalInfo.keys()):
                print('reset')
                objectBox['duration'] = 0
                durationInterval = durationIntervalInfo[currentMission]
            elif(currentMission == D.MISSION_TRACKING and MISSION_STATE[currentMission] is not None and MISSION_STATE[currentMission]['stage'] == 1 and MISSION_STATE[currentMission]['thread'] is not None):
                print('reset')
                objectBox['duration'] = 0
                durationInterval = 100000
            elif(currentMission!=preMission and currentMission == D.MISSION_TRANS_2to3):

                print('reset')
                objectBox['duration'] = 0
                durationInterval = 100000


            preMission = currentMission
            if(MISSION_STATE[currentMission] is None or MISSION_STATE[currentMission]["thread"] is None or (MISSION_STATE[currentMission]["thread"] is not None and not MISSION_STATE[currentMission]["thread"].is_alive())):
                currentMission, MISSION_STATE = D.droneActionManager(drone, currentMission, MISSION_STATE, objectBox, colorDetectionBox)

            if(currentMission in durationIntervalInfo.keys() and MISSION_STATE[currentMission + 3] is None):
                if(MISSION_STATE[currentMission] is not None and MISSION_STATE[currentMission]["stage"] == 2):
                    marker = O.main_detect(frame.transpose((2,0,1)), './weight/number_best.pt', device)
                    if(len(marker) != 0):
                        best = None
                        best_p = 0
                        for m in marker:
                            label, p = m[-1].split()
                            print("p:",p)
                            if(float(p) > best_p):
                                best = m
                                best_p = float(p)

                        m = best
                        label, p = m[-1].split()
                        m = list(map(int,m[:4]))
                        print(m, objectBox[currentMission - 3][-1])
                        # if(objectBox[currentMission - 3][-1][1] + objectBox[currentMission - 3][-1][3] < m[1]):
                            # print(marker)
                        cv2.rectangle(frame_show, (m[0],m[1]),(m[2],m[3]), (255,0,255),7)
                        cv2.putText(frame_show, label, (m[0],m[1]+50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 5)
                        objectBox[detector.ENUM_NUMBER] = [drone.get_height(), drone.get_yaw(), int(label)]
                        # else:
                        #     objectBox[detector.ENUM_NUMBER] = None
                    else:
                        objectBox[detector.ENUM_NUMBER] = None
                    MISSION_STATE = D.droneMissionManager(drone, frame, objectBox, currentMission, MISSION_STATE)

            if(currentMission == D.MISSION_TRACKING):
                    marker = O.main_detect(frame.transpose((2,0,1)), './weight/kau_best.pt', device)
                    if(len(marker) != 0):
                        best = None
                        best_p = 0
                        for m in marker:
                            label, p = m[-1].split()
                            print("p:",p)
                            if(float(p) > best_p):
                                best = m
                                best_p = float(p)
                            
                        m = list(map(int,best[:4]))
                        cv2.rectangle(frame_show, (m[0],m[1]),(m[2],m[3]), (255,0,255),7)
                        cv2.putText(frame_show, label, (m[0],m[1]+50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 5)
                        count = len(os.listdir("./image"))
                        cv2.imwrite(f"./image/{count//5} kau.png", frame_show)
                        objectBox[detector.ENUM_KAU] = m
                    else:
                        objectBox[detector.ENUM_KAU] = None

            if(currentMission == D.MISSION_TRANS_2to3):
                    marker = O.main_detect(frame.transpose((2,0,1)), './weight/flight_best.pt', device, conf_score=0.75)
                    if(len(marker) != 0):
                        # for m in marker:
                        best = {"APACH" : [None,0], "A380" : [None,0], "F-22" : [None,0], "KT-1" : [None,0]}
                        for m in marker:
                            label, p = m[-1].split()
                            
                            if(best[label][0] is None or  float(p) > best[label][1]):
                                best[label] = [m,float(p)]

                        for l in best.keys():
                            if(best[l][0] is None):
                                continue
                            m = list(map(int,best[l][0][:4]))
                            cv2.rectangle(frame_show, (m[0],m[1]),(m[2],m[3]), (255,0,255),7)
                            cv2.putText(frame_show, l, (m[0],m[1]+50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 5)
                            print(f"Label : {l} \t Prob : {best[l][1]}")

                        objectBox[detector.ENUM_F22] = best["F-22"][0]
                        if(objectBox[detector.ENUM_F22] is not None):
                            count = len(os.listdir("./image"))
                            cv2.imwrite(f"./image/{count//5} f-22.png", frame_show)
                            
                    else:
                        objectBox[detector.ENUM_F22] = None

            if(currentMission == D.MISSION_DONE):
                if (not MISSION_STATE[D.MISSION_LANDING].is_alive()):
                    break
            cv2.imshow("detection", frame_show)
            objectBox['duration'] += 1
            objectBox['duration']%=durationInterval

            cv2.waitKey(1)

            # if(i%15 == 0):
            #     print(objectBox)
        cv2.destroyAllWindows()

            

    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)
        drone.land()
    finally:
        cv2.destroyAllWindows()
        
if __name__ == '__main__':
        main()