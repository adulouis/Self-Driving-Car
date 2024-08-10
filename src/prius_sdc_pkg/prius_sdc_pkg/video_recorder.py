import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image 
from .Drive_Bot import Car
from geometry_msgs.msg import Twist


class Video_get(Node):
    def __init__(self):
        super().__init__('video_subscriber') #Node name
        #Create subscriber
        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.process_data,10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.send_cmd_vel)
        #setting for writing the frames into a video
        #self.out = cv2.VideoWriter('/home/louwee/output.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 30, (1280,720))
        self.bridge = CvBridge() #converting ros images to opencv data
        self.velocity = Twist()
        self.car = Car()

    def process_data(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8') #performing conversion
        #self.out.write(frame) #write the frames to a video
        Angle, Speed, img = self.car.drive_car(frame)

        
        self.velocity.linear.x = Speed
        self.velocity.angular.z = Angle

        cv2.imshow("output", img)
        cv2.waitKey(1) #will save the video until it is interrupted

    def send_cmd_vel(self):
        self.publisher.publish(self.velocity) 


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = Video_get()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == "__main__":
    main()