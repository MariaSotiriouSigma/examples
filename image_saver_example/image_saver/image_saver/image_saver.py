#!/usr/bin/python3

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from duckietown_msgs.msg import LEDPattern, WheelsCmdStamped
from rclpy.time import Duration
from sensor_msgs.msg import Range

import cv2
import numpy as np

#test
DEFAULT_VELOCITY_RIGHT = 0.5
DEFAULT_VELOCITY_LEFT = 0.475


class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.output_dir = "/workspace/images/"
        os.makedirs(self.output_dir, exist_ok=True)
        self.vehicle_name = os.getenv('VEHICLE_NAME')
        if self.vehicle_name is None:
            self.get_logger().error("VEHICLE_NAME not set")
            return
        self.counter = 0
        self.obstacle_detected = 0
        
        self.wheels_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 10)


        self.tof_sub = self.create_subscription(Range, f'/{self.vehicle_name}/range', self.check_range, 10)
        self.create_subscription(CompressedImage, f'/{self.vehicle_name}/image/compressed', self.manager, 10)

    def check_range(self, msg):
        distance = msg.range
        #if distance<0.2:
        #    self.obstacle_detected = True
        if distance <= 0.2:
            self.stop()

    def manager(self,msg):
        self.save_the_image(msg)
        self.analyse_the_image(msg.data)
        #self.callback(self, msg)


    def move_forward(self):
        self.run_wheels('forward_callback', DEFAULT_VELOCITY_LEFT, DEFAULT_VELOCITY_RIGHT)

    def stop(self):
        self.run_wheels('stop_callback', 0.0, 0.0)


    def run_wheels(self, frame_id, vel_left, vel_right):
        wheel_msg = WheelsCmdStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        wheel_msg.header = header
        wheel_msg.vel_left = vel_left
        wheel_msg.vel_right = vel_right
        self.wheels_pub.publish(wheel_msg)

    def analyse_the_image(self,img):  # scan the surroundings for the road, 
        self.get_logger().info(f'got to analyse the image {img}')
        #if self.obstacle_detected_detected: # supposed fix, temporary added
        #    self.stop()
        #    return
        width,height = 640, 480   # then find the direction in which the road lies 
        img = cv2.imread(f"{self.output_dir}/{self.counter//5*5}.jpg") # and change the velocities of the wheels to go to the road
        RANGE = 50                     # it does not change the velocities of the wheels yet
        
        if img == None:
            return

        self.high_contrast(img)

        # yellow = [255, 255, 0]
        black = [0,0,0]
        average_x = 0
        average_y = 0 # useless comment
        counter = 0
        for i in range(0,639,1):
            for j in range(240,479):
                px = img[j,i]
                #print(i,j, px)

                if (abs(int(px[2]) - black[0]) < RANGE) and (abs(int(px[1]) - black[1]) < RANGE) and (abs(int(px[0]) - black[2]) < RANGE):
                    average_y += j
                    average_x += i
                    counter +=1 

        if counter == 0:
            return                    

        self.get_logger().info(f'the counter is {counter}')

        average_x = average_x // counter
        average_y = average_y // counter

        if average_x == 0:
            return

        average_y = 480 - average_y
        average_x = average_x - 320


        m = (average_y)/(average_x)
        #print(m)
        #print(average_x, average_y)
        self.get_logger().info(f'm, average x, average y {m,average_x,average_y}')



        if m < 0:
            if m > -1:
                self.run_wheels(self, "left_callback", DEFAULT_VELOCITY_LEFT, DEFAULT_VELOCITY_RIGHT+0.1) # hopefully this will work
                self.get_logger().info(f'turnning left strongly {m,average_x,average_y}')

            elif m < -1:
                self.run_wheels(self, "left_callback", DEFAULT_VELOCITY_LEFT, DEFAULT_VELOCITY_RIGHT+0.05)
                self.get_logger().info(f'turnning left softly {m,average_x,average_y}')

        elif m > 0:
            if m > 1:
                self.run_wheels(self, "right_callback", DEFAULT_VELOCITY_LEFT+0.1, DEFAULT_VELOCITY_RIGHT)
                self.get_logger().info(f'turnning right strongly {m,average_x,average_y}')

            elif m < 1:
                self.run_wheels(self, "right_callback", DEFAULT_VELOCITY_LEFT+0.05, DEFAULT_VELOCITY_RIGHT)
                self.get_logger().info(f'turnning right softly {m,average_x,average_y}')

        
        #self.get_clock().sleep_for(Duration(seconds=0.2)) # restore velovity to the default value
        #self.run_wheels(self, "right_callback", DEFAULT_VELOCITY_LEFT, DEFAULT_VELOCITY_RIGHT)


    def high_contrast(self,img): # make the surroundings contrasting, so road will be identified easier
        self.get_logger().info(f'We got to high contrast with the image {img}')
        width,height = 640, 480
        basic_colours = [[0, 254, 255],[0,0,0],[255,255,255],[255,0,0]] # BGR format --- yellow, black, white
        #(255, 0, 0),(0, 255, 0),(0,0,255) --- blue, green, red

        for i in range(0,639,1):
            for j in range(0,479):
                px = img[j,i]
                List = []

                for k in range(len(basic_colours)):
                    
                    value = (basic_colours[k][0] - int(px[0]))**2 + (basic_colours[k][1] - int(px[1]))**2 + (basic_colours[k][2] - int(px[2]))**2
                    List.append(value)
                Min = min(List)
                Value = List.index(Min)
                img[j,i] = basic_colours[Value]

        cv2.imwrite(f"{self.output_dir}/{self.counter//5*5}.jpg",img)



    def save_the_image(self, msg):
        if self.counter % 5 != 0:
            self.counter += 1
            return
        with open(self.output_dir + str(self.counter) + '.jpg', 'wb') as f:
            self.get_logger().info(f'Saving image {self.counter}')
            f.write(msg.data)
        self.counter += 1

    def callback(self, msg):
        # Convert compressed image to OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        blue = [0,0,255]

        # Blue mask
        #lower_blue = np.array([100, 150, 50])
        #upper_blue = np.array([140, 255, 255])
        #for i 
        
        # How much blue is present?
        #blue_pixels = cv2.countNonZero(mask)

       # if blue_pixels > 5000:  # threshold (tune this)
        #    self.get_logger().info("BLUE OBJECT DETECTED")

            # Optional: save only when blue is detected 
    
            # nope, not optional. I need it to direct the vehicle
            #filename = os.path.join(self.output_dir, f"blue_{self.counter}.jpg")
            #cv2.imwrite(filename, frame)
            #self.counter += 1


def main():
    rclpy.init()
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
