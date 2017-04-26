from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

import ctypes
import _ctypes
import sys
import numpy as np
import time
import naoqi
from naoqi import ALProxy


if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread

# colors for drawing different bodies 

skeleton_data = None;

#robot connection settings
robotIP = "192.168.1.110"
PORT = 9559
#robotIP = "127.0.0.1"
#PORT = 51679

proxy = ALProxy("ALMotion",robotIP,PORT)
postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
NAME = 'body'
MAX_SPEED = 0.2
stiffness = 1.0
proxy.stiffnessInterpolation(NAME,stiffness,1.0)
proxy.wakeUp()

def boundary(upper,lower,angle):
   if angle < lower :
       angle = lower
   if angle > upper :
       angle = upper 
   return angle
class get_data(object):


    def __init__(self):
    
        # Loop until the user clicks the close button.
        self._done = False

        # Kinect runtime object, we want only color and body frames 
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)

        # here we will store skeleton data 
        self._bodies = None

    def run(self):
        # -------- Main Program Loop -----------
        while not self._done:
            # --- Main event loop
            # --- Woohoo! We've got a color frame! Let's fill out back buffer surface with frame's data 
            if self._kinect.has_new_color_frame():
                frame = self._kinect.get_last_color_frame()
                frame = None

            # --- Cool! We have a body frame, so can get skeletons
            if self._kinect.has_new_body_frame(): 
                self._bodies = self._kinect.get_last_body_frame()

            # --- draw skeletons to _frame_surface
            if self._bodies is not None: 
                for i in range(0, self._kinect.max_body_count):
                    body = self._bodies.bodies[i]
                    if body.is_tracked:  
                        global skeleton_data
                        skeleton_data = np.array([[body.joints[PyKinectV2.JointType_SpineBase].Position.x,body.joints[PyKinectV2.JointType_SpineBase].Position.y,body.joints[PyKinectV2.JointType_SpineBase].Position.z],
                                                  [body.joints[PyKinectV2.JointType_ShoulderLeft].Position.x,body.joints[PyKinectV2.JointType_ShoulderLeft].Position.y,body.joints[PyKinectV2.JointType_ShoulderLeft].Position.z],
                                                  [body.joints[PyKinectV2.JointType_ElbowLeft].Position.x,body.joints[PyKinectV2.JointType_ElbowLeft].Position.y,body.joints[PyKinectV2.JointType_ElbowLeft].Position.z],
                                                  [body.joints[PyKinectV2.JointType_WristLeft].Position.x,body.joints[PyKinectV2.JointType_WristLeft].Position.y ,body.joints[PyKinectV2.JointType_WristLeft].Position.z],
                                                  [body.joints[PyKinectV2.JointType_ShoulderRight].Position.x ,body.joints[PyKinectV2.JointType_ShoulderRight].Position.y, body.joints[PyKinectV2.JointType_ShoulderRight].Position.z],
                                                  [body.joints[PyKinectV2.JointType_ElbowRight].Position.x,body.joints[PyKinectV2.JointType_ElbowRight].Position.y,body.joints[PyKinectV2.JointType_ElbowRight].Position.z ],
                                                  [body.joints[PyKinectV2.JointType_WristRight].Position.x, body.joints[PyKinectV2.JointType_WristRight].Position.y, body.joints[PyKinectV2.JointType_WristRight].Position.z],
                                                  [body.joints[PyKinectV2.JointType_HipLeft].Position.x ,body.joints[PyKinectV2.JointType_HipLeft].Position.y, body.joints[PyKinectV2.JointType_HipLeft].Position.z ],
                                                  [body.joints[PyKinectV2.JointType_HipRight].Position.x,body.joints[PyKinectV2.JointType_HipRight].Position.y,body.joints[PyKinectV2.JointType_HipRight].Position.z ],
                                                  [body.joints[PyKinectV2.JointType_ThumbLeft].Position.x,body.joints[PyKinectV2.JointType_ThumbLeft].Position.y,body.joints[PyKinectV2.JointType_ThumbLeft].Position.z ],
                                                  [body.joints[PyKinectV2.JointType_ThumbRight].Position.x,body.joints[PyKinectV2.JointType_ThumbRight].Position.y,body.joints[PyKinectV2.JointType_ThumbRight].Position.z ],
                                                  [body.joints[PyKinectV2.JointType_KneeLeft].Position.x,body.joints[PyKinectV2.JointType_KneeLeft].Position.y,body.joints[PyKinectV2.JointType_KneeLeft].Position.z ],
                                                  [body.joints[PyKinectV2.JointType_KneeRight].Position.x,body.joints[PyKinectV2.JointType_KneeRight].Position.y,body.joints[PyKinectV2.JointType_KneeRight].Position.z ],
                                                  [body.joints[PyKinectV2.JointType_AnkleLeft].Position.x,body.joints[PyKinectV2.JointType_AnkleLeft].Position.y,body.joints[PyKinectV2.JointType_AnkleLeft].Position.z ],
                                                  [body.joints[PyKinectV2.JointType_AnkleRight].Position.x,body.joints[PyKinectV2.JointType_AnkleRight].Position.y,body.joints[PyKinectV2.JointType_AnkleRight].Position.z ],
                                                  [body.joints[PyKinectV2.JointType_HandRight].Position.x,body.joints[PyKinectV2.JointType_HandRight].Position.y,body.joints[PyKinectV2.JointType_HandRight].Position.z ],
                                                  [body.joints[PyKinectV2.JointType_HandLeft].Position.x,body.joints[PyKinectV2.JointType_HandLeft].Position.y,body.joints[PyKinectV2.JointType_HandLeft].Position.z ]])  
                      
                          
                                         
                    else:
                        continue
                    ######      Left     ######
                    #hip and shoulder left
                    hip_shoulder_left = np.array([skeleton_data[7][0] - skeleton_data[1][0] , skeleton_data[7][1] - skeleton_data[1][1], skeleton_data[7][2] - skeleton_data[1][2]])
                    #left upper arm
                    upper_arm_left = np.array([skeleton_data[1][0] - skeleton_data[2][0] , skeleton_data[1][1] - skeleton_data[2][1], skeleton_data[1][2] - skeleton_data[2][2]])
                    #left lower arm
                    lower_arm_left = np.array([skeleton_data[3][0] - skeleton_data[2][0] , skeleton_data[3][1] - skeleton_data[2][1], skeleton_data[3][2] - skeleton_data[2][2]])
                    #left arm plane
                    left_arm = np.cross(upper_arm_left,lower_arm_left)
                    #wrist and hand
                    wrist_hand_left = np.array([skeleton_data[3][0] - skeleton_data[16][0] , skeleton_data[3][1] - skeleton_data[16][1], skeleton_data[3][2] - skeleton_data[16][2]])

                    ######      Right     ######                    
                    #hip and shoulder right
                    hip_shoulder_right = np.array([skeleton_data[8][0] - skeleton_data[4][0] , skeleton_data[8][1] - skeleton_data[4][1], skeleton_data[8][2] - skeleton_data[4][2]])
                    #left upper arm
                    upper_arm_right = np.array([skeleton_data[4][0] - skeleton_data[5][0] , skeleton_data[4][1] - skeleton_data[5][1], skeleton_data[4][2] - skeleton_data[5][2]])
                    #left lower arm
                    lower_arm_right = np.array([skeleton_data[6][0] - skeleton_data[5][0] , skeleton_data[6][1] - skeleton_data[5][1], skeleton_data[6][2] - skeleton_data[5][2]])
                    #left arm plane
                    right_arm = np.cross(upper_arm_right,lower_arm_right)
                    #wrist and hand
                    wrist_hand_right = np.array([skeleton_data[6][0] - skeleton_data[15][0] , skeleton_data[6][1] - skeleton_data[15][1], skeleton_data[6][2] - skeleton_data[15][2]])
                    
                    
                    #body plane x0y
                    shoulder_left_right = np.array([skeleton_data[4][0] - skeleton_data[1][0] , skeleton_data[4][1] - skeleton_data[1][1], skeleton_data[4][2] - skeleton_data[1][2]])    
                    shoulder_left_hip_left = np.array([skeleton_data[7][0] - skeleton_data[1][0] , skeleton_data[7][1] - skeleton_data[1][1], skeleton_data[7][2] - skeleton_data[1][2]])
                    body_plane = np.cross(shoulder_left_hip_left,shoulder_left_right)
                    
                    
                    #left elbow roll
                    left_elbow_roll = np.arccos( np.dot(upper_arm_left,lower_arm_left) / (np.linalg.norm(upper_arm_left)*np.linalg.norm(lower_arm_left)) ) - np.pi
                    left_elbow_roll = boundary(-0.0349,-1.5446,left_elbow_roll)
                    #left elbow yaw
                    left_elbow_yaw = np.arccos(np.dot(body_plane,lower_arm_left) / (np.linalg.norm(lower_arm_left)*np.linalg.norm(body_plane))) - np.pi
                    lef_elbow_yaw = boundary(2.0857,-2.0857,left_elbow_yaw)
                    #left shoulder pitch
                    left_shoulder_pitch = np.pi/2 - np.arccos(np.dot((-1)*upper_arm_left,hip_shoulder_left) / (np.linalg.norm(upper_arm_left)*np.linalg.norm(hip_shoulder_left))) 
                    left_shoulder_pitch = boundary(2.0857,-2.0857,left_shoulder_pitch)                                       
                    #left shoulder roll
                    left_shoulder_roll = np.arccos(np.dot((-1)*upper_arm_left,shoulder_left_right) / (np.linalg.norm(upper_arm_left)*np.linalg.norm(shoulder_left_right))) - np.pi/2
                    left_shoulder_roll = boundary(1.3265,-0.314,left_shoulder_roll)
                    #left wrist yaw
                    left_wrist_yaw = np.arccos(np.dot(body_plane,lower_arm_left) / (np.linalg.norm(lower_arm_left)*np.linalg.norm(body_plane))) -np.pi/2
                    lef_wrist_yaw = boundary(1.8238,-1.8238,left_wrist_yaw)

                    #right elbow roll
                    right_elbow_roll = np.pi - np.arccos( np.dot(upper_arm_right,lower_arm_right) / (np.linalg.norm(upper_arm_right)*np.linalg.norm(lower_arm_right)) ) 
                    right_elbow_roll = boundary(1.5446, 0.0349,right_elbow_roll)
                    #right elbow yaw
                    right_elbow_yaw =np.pi - np.arccos(np.dot(body_plane,lower_arm_right) / (np.linalg.norm(lower_arm_right)*np.linalg.norm(body_plane))) 
                    right_elbow_yaw = boundary(2.0857,-2.0857,right_elbow_yaw)
                    #right shoulder pitch
                    right_shoulder_pitch = np.pi/2 - np.arccos(np.dot((-1)*upper_arm_right,hip_shoulder_right) / (np.linalg.norm(upper_arm_right)*np.linalg.norm(hip_shoulder_right))) 
                    right_shoulder_pitch = boundary(2.0857,-2.0857,right_shoulder_pitch)                                       
                    #right shoulder roll
                    right_shoulder_roll = np.pi/2 - np.arccos(np.dot((-1)*upper_arm_right,(-1)*shoulder_left_right) / (np.linalg.norm(upper_arm_right)*np.linalg.norm(shoulder_left_right))) 
                    right_shoulder_roll = boundary( 0.3142, -1.3265,right_shoulder_roll)
                    #right wrist yaw
                    right_wrist_yaw = np.arccos(np.dot(body_plane,lower_arm_right) / (np.linalg.norm(lower_arm_right)*np.linalg.norm(body_plane))) -np.pi/2
                    right_wrist_yaw = boundary(1.8238,-1.8238,right_wrist_yaw)


                    
                    #motion control
                    names      = ["LShoulderPitch","LShoulderRoll","LElbowRoll","LElbowYaw","RShoulderPitch","RShoulderRoll","RElbowRoll","RElbowYaw"]#,"LHipRoll", "LHipPitch","LKneePitch","RHipRoll", "RHipPitch","RKneePitch"]
                    angleLists = [left_shoulder_pitch,left_shoulder_roll,left_elbow_roll,left_elbow_yaw,right_shoulder_pitch,right_shoulder_roll,right_elbow_roll,right_elbow_yaw]                    
                    proxy.setAngles(names, angleLists, MAX_SPEED)

            time.sleep(0.1) 
        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()


__main__ = "Arm Movement Imitation With Nao Robot"
getData = get_data();
getData.run();