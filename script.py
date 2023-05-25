import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as Odometry
from time import sleep
from collections import deque
from turtlesim.msg import Pose as TPose

# Setting maximum difference
MAX_DIFF = 0.1

# class Pose to store the position and orientation of the turtlebot
class Pose(TPose):
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        super().__init__(x=x, y=y, theta=theta)
        
    def __repr__(self):
        return f"(x={self.x:.2f}, theta={self.y:.2f})"
    
    def __add__(self, other):

        self.x += other.x
        self.y += other.y
        return self
    
    def __sub__(self, other):
        self.x -= other.x
        self.y -= other.y
        return self
    
    def __eq__(self, other):
        return abs(self.x - other.x) < MAX_DIFF  and abs(self.y - other.y) < MAX_DIFF 


# Defining class MissionControl to store the coordinates of the turtlebot
class MissionControl(deque):    
    def __init__(self):
        super().__init__()
        coordinates = [[1.0, 0.0], [0.0, 1.0],[0.5, 0.0], [0.0,0.5],[0.5,0.0], [0.0, 0.5]]
        for coordinate in coordinates:
            new_pose = Pose()
            new_pose.x = coordinate[0]
            new_pose.y = coordinate[1]
            self.enqueue(new_pose)
                 
    def enqueue(self, x):
        super().append(x)
    
    def dequeue(self):
        return super().popleft()

# Defining class TurtleController
class TurtleController(Node):
    # Initializing the class
    def __init__(self,  mission_control, control_period=0.02):
        super().__init__('turtlecontroller')
        self.pose = Pose(x=-40.0)
        self.setpoint = Pose(x=-40.0)
        self.mission_control = mission_control

        # Creating publisher and subscriber objects
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            msg_type=Pose,
            topic="/turtle1/pose",
            callback=self.pose_callback,
            qos_profile=10
        )

        self.control_timer = self.create_timer(
            timer_period_sec = control_period, 
            callback = self.control_callback)
        
    # Function to update the setpoint
    def update_setpoint(self):
        try:
            self.setpoint = self.pose + self.mission_control.dequeue()
            self.get_logger().info(f"Turtle chegou em {self.pose} foi para {self.setpoint}")
        except IndexError:
            self.get_logger().info(f"Chegou ao destino!")
            exit()

    # Callback function for position and orientation data
    def pose_callback(self, msg):

        self.pose = Pose(x=msg.x, y=msg.y, theta=msg.theta)
        if self.setpoint.x == -40.0:
            self.update_setpoint()


    # Callback function for controlling movement
    def control_callback(self):
        if self.pose.x == -40.0:
            self.get_logger().info("Aguardando primeira pose...")
            return
        msg = Twist()
        x_diff = self.setpoint.x - self.pose.x
        y_diff = self.setpoint.y - self.pose.y

        # Printing the difference in x, y and theta
        print(f"X Difference={round(abs(x_diff), 2)}, Y Difference={round(abs(y_diff), 2)}")
       
        # Moving the turtlebot
        if self.pose == self.setpoint:
            msg.linear.x, msg.linear.y = 0.0, 0.0
            self.update_setpoint()
        if abs(y_diff) > MAX_DIFF:
            msg.linear.y = 0.5 if y_diff > 0 else -0.5
        else:
            msg.linear.y = 0.0
        if abs(x_diff) > MAX_DIFF:
            msg.linear.x = 0.5 if x_diff > 0 else -0.5
        else:
            msg.linear.x = 0.0

        # Moving the turtlebot forward
        if abs(x_diff) >= MAX_DIFF:
            msg.linear.x = 0.2

        # Publishing the movement message
        self.publisher.publish(msg)

# Define a main function that takes an optional argument 'args'
def main(args=None):
    # Initialize the ROS 2 client library with the provided arguments
    rclpy.init(args=args)
    mc = MissionControl()
    # Create an instance of the TurtleController class
    tc = TurtleController(mc)
    
    # Start spinning the node to process incoming messages
    rclpy.spin(tc)
    
    # Destroy the node and free up resources
    tc.destroy_node()
    
    # Shutdown the ROS 2 client library
    rclpy.shutdown()

# Call the main function if this script is executed directly
if __name__ == "__main__":
    main()