import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy

class LineFollower(Node):
    def __init__(self):
        super().__init__('Line_Follower')
        self.bridge=CvBridge()
        self.qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
        self.cmd_vel_pub=self.create_publisher(Twist,'/cmd_vel',self.qos)
        self.image_sub=self.create_subscription(Image,'/camera/image_raw',self.image_callback,self.qos)
        self.declare_parameter('linear_speed',0.2)
        self.declare_parameter('angular_speed',0.5)
        self.linear_speed=self.get_parameter('linear_speed').value
        self.angular_speed=self.get_parameter('angular_speed').value
        self.image_width=None
        self.image_height=None
        self.roi_height=100
        self.error_threshold=50
        self.frame_count = 0
        self.images_received = 0
        self.last_line_center = None
        self.kp=0.05
        self.ki=0.0001
        self.kd=0.5
        self.integral=0
        self.last_error=0
        self.camera_working=False
        self.line_detected=False

        self.get_logger().info('=== DEBUG LINE FOLLOWER STARTED ===')
        self.get_logger().info(f'Subscribing to: /camera/image_raw')
        self.get_logger().info(f'Publishing to: /cmd_vel')
        self.status_timer = self.create_timer(2.0, self.status_callback)

    def status_callback(self):
      
        self.get_logger().info(f'=== STATUS ===')
        self.get_logger().info(f'Images received: {self.images_received}')
        self.get_logger().info(f'Camera working: {self.camera_working}')
        self.get_logger().info(f'Line detected: {self.line_detected}')
        if self.last_line_center:
            self.get_logger().info(f'Last line center: {self.last_line_center}')
        

    def image_callback(self,msg):   
        self.images_received+=1
        self.camera_working=True
        self.frame_count+=1

        if self.frame_count % 30 == 0:
            self.get_logger().info(f'Received frame {self.frame_count}, size: {msg.width}x{msg.height}, encoding: {msg.encoding}')

        try:
            cv_image=self.bridge.imgmsg_to_cv2(msg,"bgr8")
            line_center=self.proccess_image_simple(cv_image)
            self.simple_line_following(line_center)

        except Exception as e:
                 self.get_logger().error(f'Image processing error: {str(e)}')

    def proccess_image_simple(self,image):
        try:
            self.image_height,self.image_width=image.shape[:2]
            roi_top=self.image_height-self.roi_height
            roi=image[roi_top:self.image_height,0:self.image_width]

            gray=cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)
            _,binary=cv2.threshold(gray,100,255,cv2.THRESH_BINARY_INV)   # _ is not needed as we get a 100 as the it is threshold value given by us
            contours,_=cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour=max(contours,key=cv2.contourArea)
                area=cv2.contourArea(largest_contour)

                if area>150:
                    M=cv2.moments(largest_contour)
                    if M["m00"]!=0:
                        cx=int(M["m10"]/M["m00"])
                        self.line_detected=True
                        self.last_line_center=cx

                        if self.frame_count % 30 == 0:
                            self.get_logger().info(f'Line detected at x={cx}, area={area}')
                        return cx
                    
                self.line_detected=False
                return None 
            

        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')
            twist=Twist()
            self.cmd_vel_pub.publish(twist)
            return None
            
        
    def simple_line_following(self,line_center):
        twist=Twist()
        image_center=self.image_width//2
        if line_center is not None:
            error=line_center-image_center
            twist.linear.x=self.linear_speed

            if abs(error)<self.error_threshold:
                twist.angular.z = 0.0
                if self.frame_count % 30 == 0:
                    self.get_logger().info('Going STRAIGHT')

            elif abs(error)>0:
                self.integral+=error
                derivative=error-self.last_error
                self.integral = max(min(self.integral, 50), -50)
                twist.angular.z =-((self.kp * error) + (self.ki * self.integral) + (self.kd * derivative))
                self.last_error=error
                if self.frame_count % 40==0 and twist.angular.z<0:
                    self.get_logger().info(f'Turning RIGHT (error: {error})')
                elif self.frame_count % 40==0:
                    self.get_logger().info(f'Turning LEFT (error: {error})')


        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.3 if self.last_error > 0 else -0.3
            if self.frame_count % 40 == 0:
                self.get_logger().warning('NO LINE - Searching...')
        
        self.cmd_vel_pub.publish(twist)  



def main(args=None):
    rclpy.init(args=args)
    line_follower=LineFollower()
    try:
        rclpy.spin(line_follower)
    except KeyboardInterrupt:
        print("\nShutting down debug line follower...")
    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        line_follower.cmd_vel_pub.publish(twist)
        
        line_follower.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()