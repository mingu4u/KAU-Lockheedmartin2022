from threading import Thread
import detector
import time
import numpy as np
import Target_positioning as T
import cv2


ENUM_DOWN = 0
ENUM_UP = 1

MISSION_TAKEOFF = 0
MISSION_MOVE_CENTER = 1
MISSION_DETECT = 2
MISSION_MOVE_BLACK = 3
MISSION_MOVE_RED = 4
MISSION_MOVE_BLUE = 5

MISSION_QR_BLACK = 6
MISSION_QR_RED = 7
MISSION_QR_BLUE = 8

MISSION_TRACKING = 9
MISSION_TRANS_2to3 = 10
MISSION_LANDING = 11
MISSION_DONE = 12

DRONE_HEIGHT = 30
DRONE_HEIGHT_PRESET = 10

class ActionTable():
    def __init__(self, drone):
        self.drone = drone

    @staticmethod
    def playAction(value, drone, frame):
        print("QR Mission Start")

        drone.send_rc_control(0.,0.,0.,0)
        if value == 1:
            ActionTable.back_front_30(drone)
        elif value == 2:
            ActionTable.left_right_30(drone)
        elif value == 3:
            ActionTable.cr360(drone)
        elif value == 4:
            ActionTable.rectangle(drone)
        elif value == 5:
            ActionTable.flip_back(drone)
        elif value == 6:
            ActionTable.up_flip(drone)
        elif value == 7:
            ActionTable.flip_left(drone)
        elif value == 8:
            ActionTable.up_down_30(drone)
        elif value == 9:
            ActionTable.capture(drone, frame)

        drone.send_rc_control(0.,0.,0.,0)
        drone.move_up(DRONE_HEIGHT)
        


    @staticmethod
    def back_front_30(drone):
        print('Mission 1 Start')
        drone.move_back(30)
        drone.move_forward(30)
        print('Mission 1 End')
        
    @staticmethod
    def left_right_30(drone):
        print('Mission 2 Start')
        drone.move_left(30)
        drone.move_right(30)
        print('Mission 2 End')

    
    @staticmethod
    def cr360(drone):
        print('Mission 3 Start')
        drone.rotate_clockwise(360.)
        print('Mission 3 End')

    @staticmethod    
    def rectangle(drone):
        print('Mission 4 Start')
        drone.move_right(20)
        drone.move_up(20)
        drone.move_left(20)
        drone.move_down(20)
        print('Mission 4 End')

    @staticmethod
    def flip_back(drone):
        print('Mission 5 Start')
        drone.move_forward(20)
        drone.flip_back()
        print('Mission 5 End')

    @staticmethod
    def up_flip(drone):
        print('Mission 6 Start')
        drone.move_up(30)
        drone.flip_back()
        drone.move_forward(20)
        drone.move_down(30)
        print('Mission 6 End')

    @staticmethod
    def flip_left(drone):
        print('Mission 7 Start')
        drone.move_right(20)
        drone.flip_left()
        print('Mission 7 End')

    @staticmethod
    def up_down_30(drone):
        print('Mission 8 Start')
        drone.move_up(30)
        drone.move_down(30)
        print('Mission 8 End')

    
    @staticmethod
    def capture(drone, frame):
        print('Mission 9 Start')
        cv2.imwrite("./9.jpg", frame)
        print('Mission 9 End')



def droneActionManager(drone, currentMission, MISSION_STATE, objectBox, colorDetectionBox):

    if(currentMission == MISSION_TAKEOFF):
        if(MISSION_STATE[currentMission] is None or MISSION_STATE[currentMission]['stage'] != 3): 
            if(MISSION_STATE[currentMission] is None):
                MISSION_STATE[currentMission] = {"stage" : 0, 'duration' : 0}
            thread, stage = startMission(drone, objectBox, MISSION_STATE[currentMission])
            MISSION_STATE[currentMission]['thread'] = thread
            MISSION_STATE[currentMission]["stage"] = stage
            if(thread is not None):
                thread.start()
            if(stage == 1):
                MISSION_STATE[currentMission]['duration'] += 1
            return currentMission, MISSION_STATE
        else:
            return MISSION_MOVE_CENTER, MISSION_STATE
            # return MISSION_TRACKING, MISSION_STATE

    elif(currentMission == MISSION_MOVE_CENTER):
        if(MISSION_STATE[currentMission] is None or MISSION_STATE[currentMission]['stage'] != 3):
            if MISSION_STATE[currentMission] is None:
                MISSION_STATE[currentMission] = {"stage" : 0}
            thread, stage = moveToCenter(drone, objectBox, MISSION_STATE[currentMission])
            if(thread is not None):
                MISSION_STATE[currentMission]['thread'] = thread
                MISSION_STATE[currentMission]['stage'] = stage
                thread.start()
            else:
                MISSION_STATE[currentMission] = None
            return currentMission, MISSION_STATE
        else:
            return MISSION_DETECT, MISSION_STATE

    elif(currentMission == MISSION_DETECT):
        if(MISSION_STATE[currentMission] is None or MISSION_STATE[currentMission]['stage'] != 2):
            if MISSION_STATE[currentMission] is None:
                MISSION_STATE[currentMission] = {"stage" : 0}
            thread, stage = moveToColor(drone, objectBox, MISSION_STATE[currentMission])
            if(thread is not None):
                MISSION_STATE[currentMission]['thread'] = thread
                MISSION_STATE[currentMission]['stage'] = stage
                thread.start()
            else:
                MISSION_STATE[currentMission] = None
            return currentMission, MISSION_STATE
        else:
            return MISSION_MOVE_BLACK, MISSION_STATE

    elif(currentMission == MISSION_MOVE_BLACK):

        if(MISSION_STATE[MISSION_QR_BLACK] is None):
            duration = 60
            print(objectBox['duration'] )
            if objectBox['duration'] <= duration*1:
                search_move(1,drone,13.)
            elif objectBox['duration']  <= duration*2:
                search_move(2,drone,13.)
            elif objectBox['duration']  <= duration*3:
                search_move(2,drone,13.)
            elif objectBox['duration']  <= duration*4:
                search_move(1,drone,13.)

            MISSION_STATE[currentMission] = {"thread" : None , "stage" : 2}
            return MISSION_MOVE_BLACK, MISSION_STATE
            
        elif(MISSION_STATE[MISSION_QR_BLACK].is_alive()):
            return MISSION_MOVE_BLACK, MISSION_STATE
        else:
            print("Go To Red")
            return MISSION_MOVE_RED, MISSION_STATE
            

    elif(currentMission == MISSION_MOVE_RED):
        if(MISSION_STATE[MISSION_QR_RED] is None):
            duration = 60
            
            if(objectBox[detector.ENUM_RED] is None):
                top_blade_cmd(drone.get_yaw(),objectBox[detector.ENUM_BLACK][1] + 180 + 90,drone)
                MISSION_STATE[currentMission] = {"thread" : None , "stage" : 0}

            else:
                if(MISSION_STATE[currentMission]["stage"] == 0): 
                    print("Start Red dronePosTuning")
                    temp = Thread(target=dronePosTuning, args=(drone, drone.move_back,20, 30.))
                    MISSION_STATE[currentMission] = {"thread" : temp , "stage" : 1}
                    temp.start()
                    # return MISSION_QR_RED, MISSION_STATE

                else:
                    if objectBox['duration'] <= duration*1:
                        search_move(1,drone,13.)
                    elif objectBox['duration']  <= duration*2:
                        search_move(2,drone,13.)
                    elif objectBox['duration']  <= duration*3:
                        search_move(2,drone,13.)
                    elif objectBox['duration']  <= duration*4:
                        search_move(1,drone,13.)   
                    MISSION_STATE[currentMission] = {"thread" : None , "stage" : 2}
                    # else:
                    #     drone.send_rc_control(0.,20.,0.,0) 
                    #     time.sleep(0.1)

            return MISSION_MOVE_RED, MISSION_STATE

        elif(MISSION_STATE[MISSION_QR_RED].is_alive()):
            return MISSION_MOVE_RED, MISSION_STATE
        else:
            return MISSION_MOVE_BLUE, MISSION_STATE

    elif(currentMission == MISSION_MOVE_BLUE):
        
        if(MISSION_STATE[MISSION_QR_BLUE] is None):
            duration = 60
            if(objectBox[detector.ENUM_BLUE] is None):
                top_blade_cmd(drone.get_yaw(),objectBox[detector.ENUM_RED][1] + 180 + 90,drone)
                MISSION_STATE[currentMission] = {"thread" : None , "stage" : 0}
                # top_blade_cmd(drone.get_yaw(),RED_YAW + 180 - 60 ,drone)

            else:
                if(MISSION_STATE[currentMission]['stage'] == 0):     
                    drone.send_rc_control(0.,0.,0.,0)
                    # time.sleep(0.5)
                    
                    temp = Thread(target=dronePosTuning, args=(drone, drone.move_back,20, 40.))
                    MISSION_STATE[currentMission] = {"thread" : temp , "stage" : 1}
                    temp.start()

                else:
                    if objectBox['duration'] <= duration*1:
                        search_move(1,drone,13.)
                    elif objectBox['duration']  <= duration*2:
                        search_move(2,drone,13.)
                    elif objectBox['duration']  <= duration*3:
                        search_move(2,drone,13.)
                    elif objectBox['duration']  <= duration*4:
                        search_move(1,drone,13.)  
                    MISSION_STATE[currentMission] = {"thread" : None , "stage" : 2}
                    # else:
                    #     drone.send_rc_control(0.,20.,0.,0) 
                    #     time.sleep(0.1)

            return MISSION_MOVE_BLUE, MISSION_STATE

        elif(MISSION_STATE[MISSION_QR_BLUE].is_alive()):
            return MISSION_MOVE_BLUE, MISSION_STATE
        else:
            return MISSION_TRACKING, MISSION_STATE

    elif(currentMission == MISSION_TRACKING):
        if(MISSION_STATE[currentMission] is None or MISSION_STATE[currentMission]['stage'] != 3):
            if MISSION_STATE[currentMission] is None:
                MISSION_STATE[currentMission] = {"stage" : 0, "duration" : 0, "Cruise_spd_err_before_step" : 0}
            thread, stage = transmission_1to2(drone, objectBox, MISSION_STATE[currentMission])
            MISSION_STATE[currentMission]['thread'] = thread
            MISSION_STATE[currentMission]['stage'] = stage
            if(thread is not None):
                thread.start()
            return currentMission, MISSION_STATE
        else:
            return MISSION_TRANS_2to3, MISSION_STATE

    elif(currentMission == MISSION_TRANS_2to3):
        if(MISSION_STATE[currentMission] is None or MISSION_STATE[currentMission]['stage'] != 2):
            if MISSION_STATE[currentMission] is None:
                MISSION_STATE[currentMission] = {"stage" : 0, "duration": 0,"Cruise_spd_err_before_step" : 0}
            thread, stage = transmission_2to3(drone, objectBox, MISSION_STATE[currentMission])
            MISSION_STATE[currentMission]['thread'] = thread
            MISSION_STATE[currentMission]['stage'] = stage
            if(thread is not None):
                thread.start()
            return currentMission, MISSION_STATE
        else:
            return MISSION_LANDING, MISSION_STATE
    
    elif (currentMission == MISSION_LANDING):
        temp = Thread(target=F22_landing, args=(drone,objectBox, drone.get_yaw()))
        MISSION_STATE[MISSION_LANDING] = temp
        temp.start()
        return MISSION_DONE , MISSION_STATE

    else:
        return currentMission, MISSION_STATE


def droneMissionManager(drone, frame, objectBox, currentMission, MISSION_STATE):
    if(objectBox[detector.ENUM_NUMBER] is not None and objectBox[currentMission - 3] is not None):
        print("Detected Number :", objectBox[detector.ENUM_NUMBER][-1])
        value = objectBox[detector.ENUM_NUMBER][-1]

        temp = Thread(target=ActionTable.playAction, args=(value,drone, frame))
        MISSION_STATE[currentMission + 3] = temp
        temp.start()
        
        return MISSION_STATE
    else:
        return MISSION_STATE

def startMission(drone, objectBox, STATE):
    stage = STATE['stage']
    if(stage == 0):
        return Thread(target=takeoff, args=(drone,)), 1
    elif(stage == 1):
        if(objectBox[detector.ENUM_QR] is None):
            duration = STATE['duration']
            # t_temp = duration * 0.03
            # serch_cmd = 45*np.cos(t_temp)
            t_temp = duration * 0.04
            serch_cmd = 65*np.sin(t_temp)
            drone.send_rc_control(0.,-7.,0.,int(serch_cmd))
            print(duration)
            return None, 1
        else:
            print(objectBox[detector.ENUM_QR][-1][0].data)
            drone.send_rc_control(0.,0.,0.,0)
            return None, 2
    elif(stage == 2):
        return Thread(target=time.sleep, args=(5,)), 3

def takeoff(drone):
    drone.takeoff()
    drone.move_back(30)

def moveToCenter(drone, objectBox, STATE):
    stage = STATE["stage"]
    if(stage == 0):
        if(objectBox[detector.ENUM_BLACK] is None and objectBox[detector.ENUM_RED] is None and objectBox[detector.ENUM_BLUE] is None):
            drone.send_rc_control(0.,0.,0.,40) #Edit by Sumin : to calibrate Black marker's detection Heading Yaw
            # drone.send_rc_control(0.,0.,0.,50)
            return None, 0
        else:
            drone.send_rc_control(0.,0.,0.,0)
            for color in [detector.ENUM_BLUE, detector.ENUM_RED, detector.ENUM_BLACK]:
                if(objectBox[color] is not None):
                    rect = objectBox[color][-1]
                    points = np.array([
                        (rect[0],rect[1]),(rect[0]+rect[2], rect[1]+rect[3])
                    ])
                    points[:,0] = points[:,0]/detector.WIDTH * 2000
                    points[:,1] = points[:,1]/detector.HEIGHT * 2000
                    STATE["detected"] = color
                    return Thread(target=first_move, args=([points[0],points[1]],drone)), 1
    elif(stage == 1):
        print("stage 2")
        return Thread(target=drone.rotate_clockwise, args=(360.,)), 2
    elif(stage == 2):
        detected = []
        for color in [detector.ENUM_BLUE, detector.ENUM_RED, detector.ENUM_BLACK]:
            if(objectBox[color] is not None and color != STATE["detected"]):
                detected.append(objectBox[color])
        '''
        2번째 좌상단 우하단
        yaw
        '''
        if(len(detected) == 1):
            print('detected 1')
            print(detected, STATE)
            box1 = ([detected[0][-1][0],detected[0][-1][1]], [detected[0][-1][0] + detected[0][-1][2],detected[0][-1][1]+detected[0][-1][3]])
            box1 = np.array(box1)
            box1[:,0] = box1[:,0]/detector.WIDTH * 2000
            box1[:,1] = box1[:,1]/detector.HEIGHT * 2000
            return Thread(target=second_move_case1, args=(box1, detected[0][1], drone)), 3
        else:
            print('detected 2')
            print(detected, STATE)
            box1 = np.array(([detected[0][-1][0],detected[0][-1][1]], [detected[0][-1][0] + detected[0][-1][2],detected[0][-1][1]+detected[0][-1][3]]))
            box1[:,0] = box1[:,0]/detector.WIDTH * 2000
            box1[:,1] = box1[:,1]/detector.HEIGHT * 2000
            box2 = np.array(([detected[1][-1][0],detected[1][-1][1]], [detected[1][-1][0] + detected[1][-1][2],detected[1][-1][1]+detected[1][-1][3]]))
            box2[:,0] = box2[:,0]/detector.WIDTH * 2000
            box2[:,1] = box2[:,1]/detector.HEIGHT * 2000
            print(box2)
            return Thread(target=second_move_case2, args=(box1, detected[0][1], box2, detected[1][1], drone)), 3
            
    print("Moved To Center")

def dronePosTuning(drone, moveFunc, moveDistance, clockwise):
    # drone.send_control_command("stop")
    drone.send_rc_control(0.,0.,0.,0)
    time.sleep(0.5)
    moveFunc(moveDistance)
    drone.rotate_counter_clockwise(clockwise)
    drone.move_down(20)
    drone.send_control_command("stop")
    drone.send_rc_control(13.,13.,0.,-11)
    time.sleep(2)

def moveToColor(drone, objectBox, STATE):
    stage = STATE["stage"]
    if(stage == 0):
        return Thread(target=drone.rotate_clockwise, args=(360.,)), 1
    elif(stage == 1):
        return Thread(target=rotateForYaw, args=(drone, objectBox, detector.ENUM_BLACK)), 2

def rotateForYaw(drone, objectBox, object):
    object_yaw = objectBox[object][1]
    object_height = objectBox[object][0]
    drone_yaw = drone.get_yaw()
    drone_height = drone.get_height()
    print(object_yaw, drone_yaw)
    if object_yaw<0:
            object_yaw = 360 + object_yaw
    if drone_yaw<0:    
        drone_yaw = 360 + drone_yaw
    yaw_cmd = object_yaw-drone_yaw
    height_cmd = object_height-drone_height
    if yaw_cmd<0:
        yaw_cmd = yaw_cmd + 360
    yaw_cmd = float(yaw_cmd) #Edit By Sumin _ original : to calibrate Black Marker detection heading yaw
    # yaw_cmd = float(yaw_cmd + 15) #Edit By Sumin _ optional

    if abs(height_cmd)>20:
            
        booliean = height_cmd/abs(height_cmd)
        
        drone.go_xyz_speed(0,0,height_cmd,10)
        if (0<yaw_cmd<180):
            drone.rotate_clockwise(yaw_cmd)
        else:
            drone.rotate_counter_clockwise(360-yaw_cmd) 
    elif abs(height_cmd)<=20:
        if(10<abs(height_cmd)<=20):
            booliean = height_cmd / abs(height_cmd)
            drone.go_xyz_speed(0,0,int(booliean*20),10)

            if (0<yaw_cmd<180):
                drone.rotate_clockwise(yaw_cmd)
            else:
                drone.rotate_counter_clockwise(360-yaw_cmd)
        elif(0<=abs(height_cmd)<=10):
            if (0<yaw_cmd<180):
                drone.rotate_clockwise(yaw_cmd)
            else:
                drone.rotate_counter_clockwise(360-yaw_cmd)

            # drone.move_down(DRONE_HEIGHT) #Edit by Sumin : to calibrate Number marker detection Height
            drone.move_down(DRONE_HEIGHT)
    # drone.send_control_command("stop")
    drone.send_rc_control(13.,13.,0.,-11)
    time.sleep(2)
   
def top_blade_cmd(drone_yaw,yaw_heading,drone):

    if yaw_heading<0:
        yaw_heading = 360 + yaw_heading
    if drone_yaw<0:    
        drone_yaw = 360 + drone_yaw
    yaw_cmd = drone_yaw-yaw_heading
    if yaw_cmd<0:
        yaw_cmd = yaw_cmd + 360
    yaw_cmd = (yaw_cmd+9) * np.pi / 180
    # x_cmd = 35*np.cos(yaw_cmd)
    x_cmd = 10*np.cos(yaw_cmd)
    drone.send_rc_control(0.,x_cmd,0.,100)
    time.sleep(0.05)

def search_move(mode_number, drone, forward_vel):
    x_cmd = 13.
    y_cmd = forward_vel
    yaw_vel = 27-8 - 8
    # x_cmd = 35*np.sin(yaw_cmd)
    if mode_number == 1:
        drone.send_rc_control(x_cmd,y_cmd,0.,-yaw_vel)
    elif mode_number == 2:
        drone.send_rc_control(-x_cmd,y_cmd,0.,yaw_vel)
    # time.sleep(0.1)
    # time.sleep(0.01)


def first_move(color_marker_pixel, drone):
    """ Move drone to First Flag position
    color_marker_pixel : [pixel of left up, pixel of right down]
    """
    up_left = color_marker_pixel[0]
    down_right = color_marker_pixel[1]
    target_local_position = T.marker_location_from_body(up_left, down_right, 0.25)
    target_xy_distance = np.sqrt(target_local_position[0]**2 + target_local_position[1]**2)
    print('xy_dis : ', target_xy_distance)

    # Target_yaw = np.arctan2(target_local_position[1], target_local_position[0])*180/np.pi
    # Target_yaw=Target_yaw.item(0)
    
    drone.move_up(DRONE_HEIGHT)
    "Go to Color Marker"
    if int(target_xy_distance*100) < 20:
        print("target is too close")
    else:
        drone.send_rc_control(0.,0.,0.,0)
        time.sleep(0.5)
        drone.move_forward(int(target_xy_distance*100))

def second_move_case1(color_marker_pixel,drone_yaw, drone):
    """ Move drone to Second Flag position
    case1 means drone found only one flag during the rotation above flag 1
    color_marker_pixel : [pixel of left up, pixel of right down]
    """
    up_left = color_marker_pixel[0]
    down_right = color_marker_pixel[1]
    target_local_position = T.marker_location_from_body(up_left, down_right, 0.265)
    target_xy_distance = target_local_position[0]/2
    print('xy_dis : ', target_xy_distance)

    # Target_yaw = np.arctan2(target_local_position[1], target_local_position[0])*180/np.pi
    # Target_yaw=Target_yaw.item(0)
    yaw_commander(drone, drone_yaw)

    "Go to Color Marker"
    if int(target_xy_distance*100) < 20:
        print("target is too close")
    else:
        drone.send_rc_control(0.,0.,0.,0)
        time.sleep(0.5)
        drone.move_forward(int(target_xy_distance*100))
    drone.move_down(DRONE_HEIGHT)

def second_move_case2(color_marker_pixel2,drone_yaw2,color_marker_pixel3,drone_yaw3, drone):
    """ Move drone to Second Flag position
    case2 means drone found other two flags during the rotation above flag 1
    color_marker_pixel_n : [pixel of left up, pixel of right down]
    """
    print("color_marker_pixel2: " ,color_marker_pixel2)
    print("color_marker_pixel3: ",color_marker_pixel3)
    "calculation for Flag2"
    print(color_marker_pixel2, drone_yaw2)
    print(color_marker_pixel3, drone_yaw3)
    up_left2 = color_marker_pixel2[0]
    down_right2 = color_marker_pixel2[1]
    target_local_position2 = T.marker_location_from_body(up_left2, down_right2, 0.25)
    target_xy_distance2 = np.sqrt(target_local_position2[0]**2 + target_local_position2[1]**2)
    print('flag2 xy_dis : ', target_xy_distance2)
    flag2 = vectorization(target_xy_distance2[0][0], drone_yaw2)

    "calculation for Flag3"
    up_left3 = color_marker_pixel3[0]
    down_right3 = color_marker_pixel3[1]
    target_local_position3 = T.marker_location_from_body(up_left3, down_right3, 0.25)
    print("target_local_position3: ",target_local_position3)
    target_xy_distance3 = np.sqrt(target_local_position3[0]**2 + target_local_position3[1]**2)
    print('Flag3 xy_dis : ', target_xy_distance3)
    flag3 = vectorization(target_xy_distance3[0][0], drone_yaw3)

    "found outter cirle of flag1-2-3"
    flag1 = [0.,0.,1.]
    print("f2 :", flag2, type(flag2))
    print("f3 :", flag3, type(flag3))
    center = get_center(flag1,flag2,flag3)
    print("center :",center)
    center_devec = de_vectorization(center)
    target_xy_distance = center_devec[0]
    target_yaw = center_devec[1]
    print("target_xy_distance: ", target_xy_distance)
    print("target_yaw: ",target_yaw)

    "Go to Color Marker"
    yaw_commander(drone, target_yaw)
    if int(target_xy_distance*100) < 20:
        print("target is too close")
    else:
        drone.send_rc_control(0.,0.,0.,0)
        time.sleep(0.5)
        drone.move_forward(int(target_xy_distance*100))
    drone.move_down(DRONE_HEIGHT+10)

def vectorization(length, theta):
    """
    length : [m]
    theta: [deg]
    """
    theta = theta*np.pi/180
    x = float(length*np.cos(theta))
    y = float(length*np.sin(theta))
    print("x:, ",x,type(x))
    print("y:, ",y,type(y))
    return np.array([x,y,1])

def de_vectorization(vector):
    # length = np.linalg.norm(vector)
    length = np.sqrt(vector[0]**2 + vector[1]**2)
    theta = np.arctan2(vector[1],vector[0]) * 180/np.pi
    return np.array([float(length), float(theta), 1])

def get_center(a, b, c):
    # ----------------------------------------
    # Find the normal vector from three points a, b, c
    # ----------------------------------------
    X = np.array([a, b, c])
    y = [1, 1, 1]
    n = np.linalg.inv(X.T.dot(X)).dot(X.T).dot(y)
    n = n/np.linalg.norm(n)
    # ----------------------------------------
    # Find u (v) perpendicular to vector ab (bc) and to the normal vector at the same time
    # This can be obtained by cross-product between ab and n (bc and n)
    # ----------------------------------------
    u = np.cross(b-a, n)
    v = np.cross(c-b, n)
    # ----------------------------------------
    # Find the parameter t (s) of the linear eqauation of the vertical bisector line of ab (bc) satisfying the center
    # ----------------------------------------
    X = np.array([[u[0], -v[0]], [u[1], -v[1]]])
    y = np.array([(c[0] - a[0])/2, (c[1] - a[1])/2])
    t = np.linalg.inv(X).dot(y)
    # ----------------------------------------
    # return the center of the circle and radius
    # ----------------------------------------
    center = (a+b)/2 + u*t[0]
    # r = np.linalg.norm(a-center)
    return center
    

def yaw_commander(drone, object_yaw, up_function=False):
    if(up_function):
        drone.move_up(70)
    drone_yaw = drone.get_yaw()
    if object_yaw<0:
            object_yaw = 360 + object_yaw
    if drone_yaw<0:    
        drone_yaw = 360 + drone_yaw
    yaw_cmd = object_yaw-drone_yaw
    if yaw_cmd<0:
        yaw_cmd = yaw_cmd + 360

    # yaw_cmd = round(float(yaw_cmd),3) #Edit By Sumin : prevent yaw_commander's input value near by 360 deg.
    
    # if (0<yaw_cmd<180):
    #     drone.rotate_clockwise(yaw_cmd)
    # else:
    #     drone.rotate_counter_clockwise(360-yaw_cmd) 
               
    if abs(yaw_cmd) >= 2:
        yaw_cmd = round(float(yaw_cmd),3)
        if (0<yaw_cmd<180):
            drone.rotate_clockwise(yaw_cmd)
        else:
            drone.rotate_counter_clockwise(360-yaw_cmd)                

def Side_Bearing_Guidance(STATE,Forward_velocity, R_follow_me, target_local_position, K,  drone):
    """ Forward Velocity        : drone's cruise speed [m/s]
        R_follow_me             : Drone's Following Circle Radius [m]
        Target_local Position   : Target's Pixel coordiate data [None]
        K                       : the Damping Ratio for guidance control [None]
    """
    # Cruise_spd = Forward_velocity #[m/s] to [cm/s]
    # R_ref = 0.25
    Cruise_spd_err_before_step = STATE["Cruise_spd_err_before_step"]
    eta = np.arctan2(target_local_position[1], target_local_position[0]) - np.pi/2
    target_xy_distance = np.sqrt(target_local_position[0]**2 + target_local_position[1]**2)
    Cruise_spd_err = target_xy_distance - R_follow_me # [m]
    Cruise_spd_err_diff = (Cruise_spd_err - Cruise_spd_err_before_step)/0.015
    Cruise_spd_err_int = (Cruise_spd_err + Cruise_spd_err_before_step)/2*0.015
    print( "Cruise_spd_err_diff: ", Cruise_spd_err_diff)
    print( "Cruise_spd_err_int: ", Cruise_spd_err_int)
    
    Kp = 25
    Ki = 0
    Kd = 0.1
    Cruise_spd_cmd = Cruise_spd_err*Kp + Cruise_spd_err_int*Ki + Cruise_spd_err_diff*Kd
    if Cruise_spd_cmd <= 10.0 :
        Cruise_spd_cmd = 15.0
    Cruise_spd_cmd = round(float(Cruise_spd_cmd), 2)
    print("Cruise_spd_cmd: ",Cruise_spd_cmd)

    Target_yaw = (eta + np.pi/2)*180/np.pi
    print(Target_yaw)

    
    # if target_local_position[2] < -0.15:
    #     UD_decide = 'Up'
    # elif target_local_position[2] >= 0 :
    #     UD_decide = 'Down'
    # else:
    #     UD_decide = 'Hold' #Edit By Sumin : to calibrate the sensitivity of Tracking KAU marker's Height

    if target_local_position[2] < -0.15:
        #'Target Height is Up at the drone'
        if -0.2 < target_local_position[2]:
            z_cmd = 20.
        else:
            z_cmd = round(float(-target_local_position[2]*100*1.4), 2)
    elif target_local_position[2] >= 0 :
        #'Target Height is Down at the drone'
        if target_local_position[2] < 0.1 :
            z_cmd = -20.
        else:
            z_cmd = round(float(-target_local_position[2]*100*1.4), 2)
    else:
        #'Target Height is Same at the drone'
        z_cmd = 0.
    
    target_xy_distance = np.sqrt(target_local_position[0]**2+target_local_position[1]**2)
    STATE["Cruise_spd_err_before_step"] = Cruise_spd_err
    print("Cruise_spd : ", Cruise_spd_cmd)
    if target_xy_distance > R_follow_me:
        # rc_commander(Target_yaw, UD_decide, drone, 0., Cruise_spd_cmd) #Edit By Sumin : to calibrate the sensitivity of Tracking KAU marker's Height
        rc_commander(Target_yaw, z_cmd, drone, 0., Cruise_spd_cmd)
        return 0
        
    else:
        # rc_commander(Target_yaw, UD_decide,drone,0., 0.) #Edit By Sumin : to calibrate the sensitivity of Tracking KAU marker's Height
        rc_commander(Target_yaw, z_cmd, drone,0., 0.)
        return 1

# Edit By Sumin : to calibrate the sensitivity of Tracking KAU marker's Height
# def rc_commander(Target_yaw_input, UD_decide_input, drone_input, V_cmd_input, Cruise_spd_cmd_input):
#     if Target_yaw_input >= 3:
#         print('Target is on your right side')
#         if UD_decide_input == 'Up':
#             drone_input.send_rc_control(V_cmd_input,Cruise_spd_cmd_input,20.,int(Target_yaw_input*1.4))
#         elif UD_decide_input == 'Hold':
#             drone_input.send_rc_control(V_cmd_input,Cruise_spd_cmd_input,0.,int(Target_yaw_input*1.4))
#         elif UD_decide_input == 'Down':
#             drone_input.send_rc_control(V_cmd_input,Cruise_spd_cmd_input,-20.,int(Target_yaw_input*1.4))
#     elif abs(Target_yaw_input) < 3 :
#         print('Target is on your site')
#         if UD_decide_input == 'Up':
#             drone_input.send_rc_control(0.,Cruise_spd_cmd_input,20.,0)
#         elif UD_decide_input == 'Hold':
#             drone_input.send_rc_control(0.,Cruise_spd_cmd_input,0.,0)
#         elif UD_decide_input == 'Down':
#             drone_input.send_rc_control(0.,Cruise_spd_cmd_input,-20.,0)
#     else:
#         print('Target is on your left side')
#         if UD_decide_input == 'Up':
#             drone_input.send_rc_control(-V_cmd_input,Cruise_spd_cmd_input,20.,int(Target_yaw_input*1.4))
#         elif UD_decide_input == 'Hold':
#             drone_input.send_rc_control(-V_cmd_input,Cruise_spd_cmd_input,0.,int(Target_yaw_input*1.4))
#         elif UD_decide_input == 'Down':
#             drone_input.send_rc_control(-V_cmd_input,Cruise_spd_cmd_input,-20.,int(Target_yaw_input*1.4))

def rc_commander(Target_yaw_input, z_cmd, drone_input, V_cmd_input, Cruise_spd_cmd_input):
    if Target_yaw_input >= 3:
        print('Target is on your right side')
        drone_input.send_rc_control(V_cmd_input,Cruise_spd_cmd_input,z_cmd,int(Target_yaw_input*1.4))
    elif abs(Target_yaw_input) < 3 :
        print('Target is on your site')
        drone_input.send_rc_control(0.,Cruise_spd_cmd_input,z_cmd,0)
    else:
        print('Target is on your left side')
        drone_input.send_rc_control(-V_cmd_input,Cruise_spd_cmd_input,z_cmd,int(Target_yaw_input*1.4))

def move_left_right(drone, distance):
    """positive distnace is right direction of drone
        distance : at least 20 [cm]
        """
    distance = int(distance)
    if distance >0:
        drone.move_right(distance)
    if distance <0:
        drone.move_left(abs(distance))

def F22_landing(drone, objectBox, drone_yaw):
    drone.send_rc_control(0.,0.,0.,0)
    rect = objectBox[detector.ENUM_F22]
    points = np.array([
        (rect[0],rect[1]),(rect[2], rect[3])
    ])
    points[:,0] = points[:,0]/detector.WIDTH * 2000
    points[:,1] = points[:,1]/detector.HEIGHT * 2000
    left_up = points[0]
    left_down = points[1]
    marker_location = T.marker_location_from_body(left_up, left_down, 0.65)
    # print('F22 location: ', marker_location)
    target_xy_distance = np.sqrt(marker_location[0]**2+marker_location[1]**2)
    theta = (drone_yaw - 225)*np.pi/180
    if (223 <= drone_yaw <= 227) or (-137 <= drone_yaw <= -133):
        drone.send_rc_control(0.,0.,0.,0)
    else:    
        yaw_commander(drone, 225)
    X_local_frame = float(target_xy_distance * np.cos(theta)) #[m]
    Y_local_frame = float(target_xy_distance * np.sin(theta)) #[m]
    print(f"X_local_frame : {X_local_frame}\t Y_local_frame : {Y_local_frame} \n target_xy_distance : {target_xy_distance} \t theta : {theta}")
    time.sleep(0.5)
    if abs(Y_local_frame) >= 0.2:
        move_left_right(drone, Y_local_frame*100) #[cm]
    elif 0.1 < abs(Y_local_frame) < 0.2:
        booliean = Y_local_frame/abs(Y_local_frame)
        move_left_right(drone, 25*booliean)
    drone.send_rc_control(0.,0.,0.,0)
    time.sleep(0.5)
    drone.move_forward(int(X_local_frame*100 - 25)) #[cm]
    drone.land()
    

def transmission_1to2(drone,objectBox,STATE):
    stage=STATE["stage"]
    if(stage==0):
        print("stage 1")
        return Thread(target=yaw_commander, args=(drone,225,True)), 1
    elif (stage == 1):
        if(objectBox[detector.ENUM_KAU] is None):
            duration = objectBox['duration']
            print("rotate for search / duration : ", duration)
            t_temp = duration * 0.1
            serch_cmd = 32*np.cos(t_temp) #Edit By Sumin : to calibrate the Search mode's maximum angle and speed
            # t_temp = duration * 0.05
            # serch_cmd = 40*np.sin(t_temp)
            drone.send_rc_control(0.,0.,0.,int(serch_cmd))
            return None, 1
        else:
            print("detected KAU")
            return None, 2
    elif(stage == 2):
        if(objectBox[detector.ENUM_KAU] is None):
            drone.send_rc_control(0.,0.,0.,0)
            return None, 2
        elif(STATE['duration'] > 15*1):
            return None, 3
        else:
            # drone.send_rc_control(0.,0.,0.,0)
            rect = objectBox[detector.ENUM_KAU]
            points = np.array([
                (rect[0],rect[1]),(rect[2], rect[3])
            ])
            points[:,0] = points[:,0]/detector.WIDTH * 2000
            points[:,1] = points[:,1]/detector.HEIGHT * 2000
            left_up = points[0]
            left_down = points[1]
            marker_location = T.marker_location_from_body(left_up, left_down, 0.15)
            # print('marker location: ', marker_location)
            flag = Side_Bearing_Guidance(STATE, Forward_velocity=0.2, R_follow_me=0.60, target_local_position=marker_location, K=1/30, drone=drone)
            if(flag == 0):
                STATE["duration"] = 0
                return None, 2
            else:
                print("----------------------")
                STATE["duration"] += 1
                return None, 2
                
def transmission_2to3(drone, objectBox, STATE):
    stage = STATE['stage']
    if(stage == 0):
        if(objectBox[detector.ENUM_F22] is None):
            duration = objectBox['duration']
            print("rotate for search / duration : ", duration)

            # t_temp = duration * 0.1
            # serch_cmd = 32*np.cos(t_temp) #Edit By Sumin : to calibrate the Search mode's maximum angle and speed
            t_temp = duration * 0.05
            serch_cmd = 40*np.sin(t_temp)
            drone.send_rc_control(0.,0.,0.,int(serch_cmd))
            return None, 0
        else:
            drone.send_rc_control(0.,0.,0.,0)
            return None, 1

    elif(stage == 1):
        if(objectBox[detector.ENUM_F22] is None):
            
            drone.send_rc_control(0.,0.,0.,0)
            return None, 1
        elif(STATE['duration'] > 15*1):
            return None, 2
        else:
            # print("Transmission Landing")
            rect = objectBox[detector.ENUM_F22]
            points = np.array([
                (rect[0],rect[1]),(rect[2], rect[3])
            ])
            points[:,0] = points[:,0]/detector.WIDTH * 2000
            points[:,1] = points[:,1]/detector.HEIGHT * 2000
            left_up = points[0]
            left_down = points[1]
            marker_location = T.marker_location_from_body(left_up, left_down, 0.66)
            # print('F22 location: ', marker_location)
            flag = Side_Bearing_Guidance(STATE, Forward_velocity=0.2, R_follow_me=1.2, target_local_position=marker_location, K=1/30, drone=drone)
            if(flag == 0):
                STATE['duration'] = 0
                return None, 1
            else:
                print("----------------------")
                STATE["duration"] += 1
                return None, 1

