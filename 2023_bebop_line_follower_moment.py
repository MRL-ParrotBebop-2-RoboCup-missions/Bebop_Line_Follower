#!/usr/bin/env python

# import color_cursor as color
import math, numpy
from matplotlib.pyplot import contour
import rospy
import cv2, cv_bridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist 
from std_msgs.msg import Empty, String, Bool
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
from pid import PID
import time

class Follower:

    def __init__(self):              
        self.bridge = cv_bridge.CvBridge()        
        self.image_sub = rospy.Subscriber('/bebop/image_raw', Image, self.image_callback)
        rospy.Subscriber('/bebop/states/ardrone3/PilotingState/AltitudeChanged', Ardrone3PilotingStateAltitudeChanged, self.altitude_callback, queue_size=1, buff_size=2**28)  
        self.robot_control_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        self.camera_control_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=10)
        self.robot_vel = Twist()
        self.camera_vel = Twist()        
        #########image and filtering parameters 
        self.kernel = numpy.ones((3, 3), numpy.uint8)
        self.search_top = 0
        self.robot_altitude = 0
        self.altitude_error = 0
        self.altitude_goal = 0.85
        #########Robot velocity commands       
        self.robot_vel = Twist()
        self.robot_vel.angular.x = 0
        self.robot_vel.angular.y = 0
        self.robot_vel.angular.z = 0
        self.robot_vel.linear.x = 0
        self.robot_vel.linear.y = 0
        self.robot_vel.linear.z = 0        
        ##########Camera velocity commands            
        self.camera_vel = Twist()
        self.camera_vel.angular.x = 0        
        self.camera_vel.angular.y = -90
        self.camera_vel.angular.z = 0
        self.camera_vel.linear.x = 0
        self.camera_vel.linear.y = 0
        self.camera_vel.linear.z = 0
        self.camera_control_pub.publish(self.camera_vel)        
        ##########PID 
        self.sabz_ghermez = 0
        self.x_vel = 0
        self.y_vel = 0
        self.yaw_vel = 0
        self.yaw_error = 0 
        self.pitch_cut_off = 0
        self.roll_cut_off = 0.05
        # self.pid_Pitch = PID(0.0008, 0.0008,  0.007, -self.pitch_cut_off, self.pitch_cut_off, -0.02, 0.02)
        # self.pid_Roll  = PID(0.005, 0.000,  0.01, -0.08, 0.08, -0.02, 0.02)
        # self.pid_yaw = PID(0.01,   0.000,   0.025, -0.2, 0.2, -0.01, 0.01)
        # self.pid_z = PID(0.1, 0.000, 0.05, -0.2, 0.1, -0.2, 0.2)
        self.pid_Pitch = PID(0.0008, 0.0008, 0.007, -self.pitch_cut_off, self.pitch_cut_off, -0.02, 0.02)
        self.pid_Roll  = PID(0.0004, 0.000,  0.035, -self.roll_cut_off, self.roll_cut_off, -0.02, 0.02)
        self.pid_yaw = PID(0.01,   0.000,   0.05, -0.3, 0.3, -0.05, 0.05) #init
        self.pid_z = PID(0.1, 0.000, 0.05, -0.2, 0.1, -0.2, 0.2)        
           
    def robot_command(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_z=0.0):
        self.robot_vel.linear.x = linear_x
        self.robot_vel.linear.y = linear_y
        self.robot_vel.linear.z = linear_z
        self.robot_vel.angular.z = angular_z
        self.robot_control_pub.publish(self.robot_vel)
        
    def camera_command(self, angular_y):
        self.camera_vel.angular.y = angular_y      
        self.camera_control_pub.publish(self.camera_vel)    
    """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""      angle      """ 
    def angle_trunc(self, angle):
        if angle < 0.0:
            angle += math.pi * 2
        return angle            
    def get_angle(self, x_orig, y_orig, x_des, y_des):
        deltaY = y_des - y_orig
        deltaX = x_des - x_orig
        return self.angle_trunc(math.atan2(deltaY, deltaX))*180/math.pi - 270                  
     #################################################################################for altitude     
    def altitude_callback(self, altitude_msg):       
        self.robot_altitude =  round(altitude_msg.altitude, 2)
        
    """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""  color filter      """     
    def image_filter(self, original_image):             
        
        blur = cv2.blur(original_image,(10,10))        
        lower = [0, 95, 138]
        upper = [102, 255, 255]        
        lower_rop = numpy.array(lower)
        upper_rop = numpy.array(upper)
        hsv_image = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        masked_image = cv2.inRange(hsv_image, lower_rop, upper_rop)         
        masked_image = cv2.dilate(masked_image, self.kernel, iterations=3)
        # edges = cv2.Canny(masked_image,50,150,apertureSize = 3)

        # contours_blk, _ = cv2.findContours(masked_image.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # if len(contours_blk)>0:
        #     contours_blk = max(contours_blk, key=cv2.contourArea)        
        # else:
        contours_blk = 0
        return masked_image, hsv_image, contours_blk

    """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""     cordinate filter """
    def cordinate_filter(self, masked_image):
        h, w = masked_image.shape            #height width and depth of image         
        # [height. width]
        # self.search_top = int(0.3*h)            #top part of image for search
        
        masked_image[:, 0:int(w/4) ] = 0          #left part of image cancelation vertical 
        masked_image[:, int(3*w/4):w ] = 0        #right part of image cancelation vertical
   
        masked_image_top = masked_image[0 : h//3, :]
        masked_image_bot = masked_image[: ,:]     
        
        Mtop = cv2.moments(masked_image_top)        
        Mbot = cv2.moments(masked_image_bot)           
        
        if Mbot['m00'] > 0:                     
            cx_bot = int(Mbot['m10']/Mbot['m00'])
            cy_bot = int(Mbot['m01']/Mbot['m00']) 
        else:
            cx_bot , cy_bot  =  w//2 , h//2           
        if Mtop['m00'] > 0:                     
            cx_top = int(Mtop['m10']/Mtop['m00'])
            cy_top = int(Mtop['m01']/Mtop['m00'])   
        else:
            cx_top ,cy_top =  w//2 , h//3               
           
        return [cx_top, cx_bot], [cy_top, cy_bot], w, h, [masked_image_top, masked_image_bot]

    """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""     imshow      """
    def imshow_func(self, image, mask, hsv,  cx, cy, w, h, black_contour):      
        
        hbot, wbot = mask[1].shape

        # cv2.putText(image, text='xvel='+str(round(self.x_vel,3)) + '    yvel='+str(round(self.y_vel,3))+'    '+'    yawvel='+str(round(self.yaw_vel,3)), org=(50, 50),
        #     fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=(255, 0, 0),thickness=1)
        cv2.putText(image, text='FWD-BWD :  '+str( self.sabz_ghermez), org=(10, 20), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.5, color=(0, 0, 0),thickness=1)


        cv2.putText(image, text='roll :  '+str( self.roll_error), org=(int(w)-150, 20), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.5, color=(0, 0, 0),thickness=1)
        cv2.putText(image, text='yaw :  ' +str(  self.yaw_error), org=(int(w)-150, 40), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.5, color=(0, 0, 0),thickness=1)
        cv2.putText(image, text='pitch : '+str(self.pitch_error),org=(int(w)-150, 60), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.5, color=(0, 0, 0),thickness=1)

        cv2.circle(image, (cx[0], cy[0]), 20, (255,0,0), -1) # blue is top
        cv2.circle(image, (cx[1], cy[1]), 20, (0,0,255), -1) # red is bot
        cv2.circle(image, (w//2, h//2), 20, (0,255,0), -1) # green is bot
        
        cv2.line(image, (cx[1], cy[1]), (cx[0], cy[0]), (0,255, 0), 2)
        cv2.line(image, (int(wbot/2), int(hbot/2)), (int(w/2), h), (255, 0,0), 2)   # constant  line        
        
        # lines = cv2.HoughLines(black_contour,1,numpy.pi/180, 200)
        # for x1,y1,x2,y2 in lines[0]:
        #     cv2.line(image,(x1,y1),(x2,y2),(0,255,0),2)
        # # blackbox = cv2.minAreaRect(black_contour)
        # box = cv2.boxPoints(blackbox)
        # box = numpy.int0(box)
        # cv2.drawContours(image, [box], 0, (250, 0, 0), 30)
        
        # cv2.imshow("top", mask[0])
        cv2.imshow("bot", mask[1])
        cv2.imshow("window", image )  
        # cv2.imshow("hsv", black_contour )  
        
        k = cv2.waitKey(1)  
        if k == 27:  # close on ESC key
            cv2.destroyAllWindows()
            rospy.signal_shutdown('interrupt')         
    """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""     image callback      """
    def image_callback(self, msg):  
        
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')        
        mask, hsv, contour = self.image_filter(image)
        cx, cy, w, h, masked= self.cordinate_filter(mask)    
             
        """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""      errors      """       
        self.pitch_error = (cy[1]- cy[0])  # forward movement
        self.sabz_ghermez = h//2 - cy[1]    #    
        
        self.yaw_error = round(self.get_angle(cx[1], cy[1], cx[0], cy[0]), 2)   
        self.roll_error  = (w//2 - cx[1] )  # vasat safhe - red circle
        
        self.altitude_error = self.altitude_goal - self.robot_altitude
        """"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""     velocity set """
        #  if abs(self.yaw_error) >= 5:
        #     self.yaw_vel = -1 * self.pid_yaw.update(self.yaw_error)            
        #     self.y_vel =  self.pid_Roll.update(self.roll_error) * 0.1
        #     self.x_vel = 0.015
        #     print('stop and yaw')

        #     if abs(self.roll_error) >= 50:
        #         self.x_vel = 0.008
        #         self.yaw_vel = 0
        #         self.y_vel =  self.pid_Roll.update(self.roll_error)             
        #         print('stop and roll')
        
        # else:
        #     self.x_vel = 0.03
        #     self.y_vel =  self.pid_Roll.update(self.roll_error)
        #     self.yaw_vel = 0
        #     print('go forward')

        # self.z_vel = self.pid_z.update(self.altitude_error)          
        self.x_vel = 0
        self.y_vel = 0
        self.z_vel = 0
        self.yaw_vel = 0

        print(self.sabz_ghermez)
        # if abs(self.yaw_error) >= 5:
        #     self.yaw_vel = -1 * self.pid_yaw.update(self.yaw_error)       
        #     self.x_vel = 0.01
            # if self.yaw_error >= 12:
            #     self.x_vel = 0
            # elif self.yaw_error <= -10:
            #     for i in range(1,10):
            #         self.x_vel = 0.2       
            #         print('jump drift')         
            # print('stop and yaw')

        # elif abs(self.roll_error) >= 20 and  abs(self.yaw_error) <= 30:            
        #     self.y_vel =  self.pid_Roll.update(self.roll_error)   
        #     self.x_vel = 0.01            
        #     print('stop and roll')        
        
        # elif abs(self.yaw_error) < 5 and abs(self.roll_error) < 20:
        #     self.x_vel = 0.03           
        #     print('go forward')
        # else:
        #     self.x_vel = 0           
        #     print('stop')

        # self.robot_command(linear_x=self.x_vel, linear_y=self.y_vel, linear_z=self.z_vel, angular_z=self.yaw_vel) 
        # print('rollE:', self.roll_error, '   pitchE:', self.pitch_error, '   YawE:', self.yaw_error, '   ekhtelaf==', abs(cx[1]-cx[0]))      
        
        self.imshow_func(image, masked, hsv, cx, cy, w, h, contour)        

if __name__ == '__main__':
    try: 
        rospy.init_node('lineFollower_node')       
        follower = Follower()
        time.sleep(1) 
        for i in range(1,5):
            follower.camera_command(angular_y= -90)          
        time.sleep(1) 
        
        rospy.spin()
    except (KeyboardInterrupt, EOFError):
        cv2.destroyAllWindows()
        rospy.signal_shutdown('keyboard')
        exit()


