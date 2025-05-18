import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
import threading

class QuadrupedSpider(Node):
    def __init__(self):
        # Initialize the parent class with the node name 'joint_controller'
        super().__init__('joint_controller')

        # Create publishers for each leg joint
        # Each joint has a unique topic for publishing its position
        # 'jlf' -> front-left leg, 'jrf' -> front-right leg
        # 'jlr' -> rear-left leg, 'jrr' -> rear-right leg
        # Numbers (1, 2, 3) represent individual joints in a leg
        self.publishers_joints = {
            'jlf1': self.create_publisher(Float64, 'jlf1_topic', 100),  # Joint 1 of front-left leg
            'jlf2': self.create_publisher(Float64, 'jlf2_topic', 100),  # Joint 2 of front-left leg
            'jlf3': self.create_publisher(Float64, 'jlf3_topic', 100),  # Joint 3 of front-left leg
            'jrf1': self.create_publisher(Float64, 'jrf1_topic', 100),  # Joint 1 of front-right leg
            'jrf2': self.create_publisher(Float64, 'jrf2_topic', 100),  # Joint 2 of front-right leg
            'jrf3': self.create_publisher(Float64, 'jrf3_topic', 100),  # Joint 3 of front-right leg
            'jlr1': self.create_publisher(Float64, 'jlr1_topic', 100),  # Joint 1 of rear-left leg
            'jlr2': self.create_publisher(Float64, 'jlr2_topic', 100),  # Joint 2 of rear-left leg
            'jlr3': self.create_publisher(Float64, 'jlr3_topic', 100),  # Joint 3 of rear-left leg
            'jrr1': self.create_publisher(Float64, 'jrr1_topic', 100),  # Joint 1 of rear-right leg
            'jrr2': self.create_publisher(Float64, 'jrr2_topic', 100),  # Joint 2 of rear-right leg
            'jrr3': self.create_publisher(Float64, 'jrr3_topic', 100),  # Joint 3 of rear-right leg
        }

        # Subscribe to LIDAR data
        # The 'lidar_scan' topic provides LaserScan messages, which are processed in the callback function
        self.lidar_subscription = self.create_subscription(
            LaserScan,           # Message type for LIDAR data
            'lidar_scan',        # Topic name for LIDAR data
            self.lidar_callback, # Callback function to process incoming data
            10                   # Queue size for storing incoming messages
        )

        # Create a timer for periodic publishing of joint positions
        # The timer executes the 'publish_joint_positions' method every 0.2 seconds
        self.timer = self.create_timer(0.2, self.publish_joint_positions)

        # Initialize the robot's state variables
        self.active = False               # Indicates whether the robot is actively walking
        self.step = 0                     # Counter to track the current gait phase
        self.obstacle_detected = False    # Flag to indicate if an obstacle is detected by LIDAR
        self.obstacle_side = None         # Tracks whether the obstacle is on the left or right side


    def lidar_callback(self, msg):
        """
        Process LIDAR data to detect obstacles and determine their position (left or right).

        Parameters:
        msg (LaserScan): The LIDAR scan message containing an array of distance readings (ranges).

        Functionality:
      - Divides the LIDAR ranges into two halves: left and right.
      - Calculates the minimum distance in each half to identify obstacles.
      - Sets the `obstacle_detected` flag and the `obstacle_side` attribute based on the location of the closest obstacle.
      - Logs a message indicating obstacle detection and distance.
        """
        # Divide the LIDAR ranges into left and right sections
        # The first half (minus a small overlap) corresponds to the left side
        left_ranges = msg.ranges[:(len(msg.ranges)//2)-5]
        # The second half (including the small overlap) corresponds to the right side
        right_ranges = msg.ranges[(len(msg.ranges)//2)-5:]
        
        # Find the minimum distance in the left and right ranges
        min_left_distance = min(left_ranges)
        min_right_distance = min(right_ranges)
        
        # Check if any obstacle is within a critical distance (0.3 meters)
        if min_left_distance < 0.3 or min_right_distance < 0.3:
            self.obstacle_detected = True

             # Determine the side of the obstacle based on the closer minimum distance
            if min_left_distance < min_right_distance:
                self.obstacle_side = 'left'# Obstacle is closer on the left
                
            else:
                self.obstacle_side = 'right'# Obstacle is closer on the right
        else:
            # No obstacle detected within the critical distance
            self.obstacle_detected = False
            self.obstacle_side = None

    def toggle_active(self):
        """
        Toggle the movement state.
        Realize joint motion by releasing the angle information of the joints
        realize the forward motion and steering of the quadruped robot.
        """
        self.active = not self.active
        if self.active:
            self.get_logger().info("Motion started.")
        else:
            self.get_logger().info("Motion stopped.")

    def publish_joint_positions(self):
        if self.active:
            # Initialize messages for each joint
            joint_msgs = {name: Float64() for name in self.publishers_joints}

            if self.obstacle_detected:
                # Stop or modify movement based on obstacle position
                for msg in joint_msgs.values():
                    msg.data = 0.0

                # When an obstacle is detected to the left, execute the right turn code
                if self.obstacle_side == 'left':
                 if self.step % 4 == 0:
                    joint_msgs['jlf1'].data, joint_msgs['jlf2'].data, joint_msgs['jlf3'].data = -0.2, 0.7, 0.0
                    joint_msgs['jrf1'].data, joint_msgs['jrf2'].data, joint_msgs['jrf3'].data = 0.0, 0.0, 0.0
                    joint_msgs['jlr1'].data, joint_msgs['jlr2'].data, joint_msgs['jlr3'].data = -0.2, 0.7, 0.0
                    joint_msgs['jrr1'].data, joint_msgs['jrr2'].data, joint_msgs['jrr3'].data = 0.0, 0.0, 0.0
                 elif self.step % 4 == 1:
                    joint_msgs['jlf1'].data, joint_msgs['jlf2'].data, joint_msgs['jlf3'].data = -0.2, 0.0, 0.0
                    joint_msgs['jrf1'].data, joint_msgs['jrf2'].data, joint_msgs['jrf3'].data = 0.0, 0.0, 0.0
                    joint_msgs['jlr1'].data, joint_msgs['jlr2'].data, joint_msgs['jlr3'].data = -0.2, 0.0, 0.0
                    joint_msgs['jrr1'].data, joint_msgs['jrr2'].data, joint_msgs['jrr3'].data = 0.0, 0.0, 0.0
                 elif self.step % 4 == 2:
                    joint_msgs['jlf1'].data, joint_msgs['jlf2'].data, joint_msgs['jlf3'].data = 0.0, 0.0, 0.0
                    joint_msgs['jrf1'].data, joint_msgs['jrf2'].data, joint_msgs['jrf3'].data = -0.2, 0.7, 0.0
                    joint_msgs['jlr1'].data, joint_msgs['jlr2'].data, joint_msgs['jlr3'].data = 0.0, 0.0, 0.0
                    joint_msgs['jrr1'].data, joint_msgs['jrr2'].data, joint_msgs['jrr3'].data = -0.2, 0.7, 0.0
                 elif self.step % 4 == 3:
                    joint_msgs['jlf1'].data, joint_msgs['jlf2'].data, joint_msgs['jlf3'].data = 0.0, 0.0, 0.0
                    joint_msgs['jrf1'].data, joint_msgs['jrf2'].data, joint_msgs['jrf3'].data = -0.2, 0.0, 0.0
                    joint_msgs['jlr1'].data, joint_msgs['jlr2'].data, joint_msgs['jlr3'].data = 0.0, 0.0, 0.0
                    joint_msgs['jrr1'].data, joint_msgs['jrr2'].data, joint_msgs['jrr3'].data = -0.2, 0.0, 0.0

                 self.step += 1# Move to the next step in the gait cycle

                #When an obstacle is detected to the right, execute the left turn code
                elif self.obstacle_side == 'right':
                 if self.step % 4 == 0:
                    joint_msgs['jlf1'].data, joint_msgs['jlf2'].data, joint_msgs['jlf3'].data = 0.2, 0.7, 0.0
                    joint_msgs['jrf1'].data, joint_msgs['jrf2'].data, joint_msgs['jrf3'].data = 0.0, 0.0, 0.0
                    joint_msgs['jlr1'].data, joint_msgs['jlr2'].data, joint_msgs['jlr3'].data = 0.2, 0.7, 0.0
                    joint_msgs['jrr1'].data, joint_msgs['jrr2'].data, joint_msgs['jrr3'].data = 0.0, 0.0, 0.0
                 elif self.step % 4 == 1:
                    joint_msgs['jlf1'].data, joint_msgs['jlf2'].data, joint_msgs['jlf3'].data = 0.2, 0.0, 0.0
                    joint_msgs['jrf1'].data, joint_msgs['jrf2'].data, joint_msgs['jrf3'].data = 0.0, 0.0, 0.0
                    joint_msgs['jlr1'].data, joint_msgs['jlr2'].data, joint_msgs['jlr3'].data = 0.2, 0.0, 0.0
                    joint_msgs['jrr1'].data, joint_msgs['jrr2'].data, joint_msgs['jrr3'].data = 0.0, 0.0, 0.0
                 elif self.step % 4 == 2:
                    joint_msgs['jlf1'].data, joint_msgs['jlf2'].data, joint_msgs['jlf3'].data = 0.0, 0.0, 0.0
                    joint_msgs['jrf1'].data, joint_msgs['jrf2'].data, joint_msgs['jrf3'].data = 0.2, 0.7, 0.0
                    joint_msgs['jlr1'].data, joint_msgs['jlr2'].data, joint_msgs['jlr3'].data = 0.0, 0.0, 0.0
                    joint_msgs['jrr1'].data, joint_msgs['jrr2'].data, joint_msgs['jrr3'].data = 0.2, 0.7, 0.0
                 elif self.step % 4 == 3:
                    joint_msgs['jlf1'].data, joint_msgs['jlf2'].data, joint_msgs['jlf3'].data = 0.0, 0.0, 0.0
                    joint_msgs['jrf1'].data, joint_msgs['jrf2'].data, joint_msgs['jrf3'].data = 0.2, 0.0, 0.0
                    joint_msgs['jlr1'].data, joint_msgs['jlr2'].data, joint_msgs['jlr3'].data = 0.0, 0.0, 0.0
                    joint_msgs['jrr1'].data, joint_msgs['jrr2'].data, joint_msgs['jrr3'].data = 0.2, 0.0, 0.0

                 self.step += 1# Move to the next step in the gait cycle

            else:
                # Otherwise keep walk straightly
                if self.step % 4 == 0:
                    joint_msgs['jlf1'].data, joint_msgs['jlf2'].data, joint_msgs['jlf3'].data = 0.7, 0.7, 0.0
                    joint_msgs['jrf1'].data, joint_msgs['jrf2'].data, joint_msgs['jrf3'].data = 0.0, 0.0, 0.0
                    joint_msgs['jlr1'].data, joint_msgs['jlr2'].data, joint_msgs['jlr3'].data = -0.7, 0.7, 0.0
                    joint_msgs['jrr1'].data, joint_msgs['jrr2'].data, joint_msgs['jrr3'].data = 0.0, 0.0, 0.0
                elif self.step % 4 == 1:
                    joint_msgs['jlf1'].data, joint_msgs['jlf2'].data, joint_msgs['jlf3'].data = 0.7, 0.0, 0.0
                    joint_msgs['jrf1'].data, joint_msgs['jrf2'].data, joint_msgs['jrf3'].data = 0.0, 0.0, 0.0
                    joint_msgs['jlr1'].data, joint_msgs['jlr2'].data, joint_msgs['jlr3'].data = -0.7, 0.0, 0.0
                    joint_msgs['jrr1'].data, joint_msgs['jrr2'].data, joint_msgs['jrr3'].data = 0.0, 0.0, 0.0
                elif self.step % 4 == 2:
                    joint_msgs['jlf1'].data, joint_msgs['jlf2'].data, joint_msgs['jlf3'].data = 0.0, 0.0, 0.0
                    joint_msgs['jrf1'].data, joint_msgs['jrf2'].data, joint_msgs['jrf3'].data = -0.7, 0.7, 0.0
                    joint_msgs['jlr1'].data, joint_msgs['jlr2'].data, joint_msgs['jlr3'].data = 0.0, 0.0, 0.0
                    joint_msgs['jrr1'].data, joint_msgs['jrr2'].data, joint_msgs['jrr3'].data = 0.7, 0.7, 0.0
                elif self.step % 4 == 3:
                    joint_msgs['jlf1'].data, joint_msgs['jlf2'].data, joint_msgs['jlf3'].data = 0.0, 0.0, 0.0
                    joint_msgs['jrf1'].data, joint_msgs['jrf2'].data, joint_msgs['jrf3'].data = -0.7, 0.0, 0.0
                    joint_msgs['jlr1'].data, joint_msgs['jlr2'].data, joint_msgs['jlr3'].data = 0.0, 0.0, 0.0
                    joint_msgs['jrr1'].data, joint_msgs['jrr2'].data, joint_msgs['jrr3'].data = 0.7, 0.0, 0.0

                self.step += 1# Move to the next step in the gait cycle

            # Publish joint commands
            for joint, publisher in self.publishers_joints.items():
                publisher.publish(joint_msgs[joint])

def keyboard_listener(node):
    """
    Independent thread to listen for keyboard inputs to control motion state.
    
    """
    while rclpy.ok():
        cmd = input("start movement: enter s; quit: enter q ")
        match cmd:
            case 's':
                node.toggle_active()
            case 'q':
                rclpy.shutdown()
                break
            case _:
                print("Invalid input error. Please enter 's' to start/stop or 'q' to quit.")

def main(args=None):
    rclpy.init(args=args)
    node = QuadrupedSpider()

    # Start keyboard listener thread
    thread = threading.Thread(target=keyboard_listener, args=(node,))
    thread.start()

    # Start ROS2 event loop
    rclpy.spin(node)

    # Wait for the thread to finish
    thread.join()

if __name__ == '__main__':
    main()


