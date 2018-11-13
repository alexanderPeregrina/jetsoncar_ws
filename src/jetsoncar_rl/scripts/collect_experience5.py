#!/usr/bin/env python
from __future__ import division
import rospy
import pyrealsense2 as rs
from estimate_motion_variables2 import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import numpy as np


# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

class CollectExpNode(object):
    def __init__(self):
        # camera Parameters
        self.episode = 90
        self.frame_counter = -1
        self.pipeline = rs.pipeline()
        self.config = rs.config()
         #config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 320, 180, rs.format.bgr8, 60)
        self.color_image = np.zeros([180, 320, 3])
        self.img_shape = np.shape(self.color_image)
        self.roi = (90, 170, 0, 320)
	self.none_counter = 0

        # Start streaming
        self.pipeline.start(self.config)
        # feature extraction parameters
        self.eta = None
        self.delta_x = None
        self.experience = []
        self.vel_state = False
        self.twist = Twist()
        self.last_delta_x = None
        self.last_eta = None
        self.first_frame = True
        self.p_actions = np.linspace(-2.5, 2.5, 9)
	
	
        # self.bridge = CvBridge()

        # Node cycle rate (in Hz)
        self.rate = rospy.Rate(27)

        # Publisher
        self.pub = rospy.Publisher('jetsoncar_instructions', Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber("joy", Joy, self.joy_callback)
        # rospy.Subscriber("color_image", Image, self.image_callback)

    def joy_callback(self, data):

        # Steering angle , transform joystick commands to actions

        self.twist.angular.z = self.p_actions[np.argmin(abs(self.p_actions - 5 * data.axes[0]))]

        # Throttle Comands, while button 7 is pressed -> capture experience

        if data.buttons[6] == 1 and data.buttons[7] == 0:
            self.twist.linear.x = (5 * (data.axes[4] + 1) - 10) / 8
            self.vel_state = False
            self.first_measure = True
        elif data.buttons[6] == 0 and data.buttons[7] == 1:
            self.vel_state = True

        elif data.buttons[6] == 1 and data.buttons[7] == 1:
            self.twist.linear.x = 5 * (data.axes[4] + 1) - 10
            self.vel_state = False
        elif data.buttons[6] == 0 and data.buttons[7] == 0:
            self.twist.linear.x = (-5 * (data.axes[4] + 1) + 10) / 8
            self.vel_state = False
        else:
            self.vel_state = False

    def get_state_values(self, img, last_eta, last_delta_x):

        delta_x = None
        eta = None

        # Converting sensor_msgs.Image data type to cv2 image
        # img = self.bridge.imgmsg_to_cv2(data, "brg8")
        # Getting interesting points of the road
        points, bin_img = get_lane_points(img, thresh=190, stride=3, roi = self.roi)
        points = list(filter(lambda x: x, points))
        cv2.imwrite("images/ep" + str(self.episode) + "_frame" + str(self.frame_counter) + ".png", bin_img.astype('uint8'))
        groups = clustering_points(points)

        if self.first_frame:

            left_group, dashed_group, right_group = classify_groups(groups)
            if dashed_group:
                dashed_eq = polynomial_fitting(dashed_group)
                eta, delta_x = get_heading_angle_lateral_deviation(dashed_eq, self.roi)

            if eta and delta_x:
                self.first_frame = False
                return 180 * eta / np.pi, delta_x
            else:
                return None, None
        else:
            eta, delta_x = get_nearest_state_variables(groups, last_eta, last_delta_x, self.roi)

            if eta and delta_x:

                return eta, delta_x
            else:
                return None, None


    def get_experience(self):

        counter = 0

        #frames = self.pipeline.wait_for_frames()
        #color_frame = frames.get_color_frame()

        while not rospy.is_shutdown():

            if self.vel_state:
		#self.twist.linear.x = 1.18

                self.frame_counter += 1

                if counter < 15:
                    self.twist.linear.x = 1.18
                    counter += 1
                elif counter < 18:
                    self.twist.linear.x = 0
                    counter += 1
                else:
                    counter = 0

                # Wait for a coherent pair of frames: depth and color
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                self.color_image = np.asanyarray(color_frame.get_data())
                self.eta, self.delta_x = self.get_state_values(self.color_image, self.last_eta, self.last_delta_x)
                # self.experience = self.experience + [(self.delta_x, self.eta, self.twist.angular.z)]
                self.experience.append((self.delta_x, self.eta, self.twist.angular.z, self.frame_counter))
                if self.eta and self.delta_x:
                    self.last_eta = self.eta
                    self.last_delta_x = self.delta_x
		    self.none_counter = 0
		else:
		    self.none_counter += 1
		    if self.none_counter >= 5:
			self.first_frame = True 
                # cv2.imwrite('images/frame_'+ str(self.frame_counter) + '_' + str(self.episode) + '.jpg', self.gray_image)"""
            self.pub.publish(self.twist)
            self.rate.sleep()

        self.pipeline.stop()
	with open("experiences/exp_ep" + str(self.episode) + ".txt", "w") as exp_doc:
            for i in self.experience:
                for j in i:
                    if isinstance(j, float):
                        exp_doc.write("%.8f" % j)
                    else:
                        exp_doc.write(str(j))
                    exp_doc.write('\t')
                exp_doc.write('\n')
        exp_doc.close()
         


if __name__ == '__main__':
    try:
        rospy.init_node("collect_experience", anonymous=True)
        collect_exp = CollectExpNode()
        collect_exp.get_experience()
    except rospy.ROSInterruptException:
        pass

