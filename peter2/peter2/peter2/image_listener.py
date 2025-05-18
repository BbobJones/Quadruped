import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageListener(Node):
    def __init__(self):
        super().__init__('image_listener')
        self.bridge = CvBridge()  # Initialize CvBridge for ROS-to-OpenCV image conversion
        
        # Subscribe to the camera image topic /boxes_image
        self.subscription = self.create_subscription(
            Image,          # Message type
            '/boxes_image', # Topic name to subscribe to
            self.image_callback, # Callback function for processing images
            10              # QoS (queue size)
        )
        self.get_logger().info("Image listener node has started.")  # Log that the node has started

    def image_callback(self, msg):
        """
        Callback function to process received image messages.
        """
        try:
            # Convert ROS 2 image message to OpenCV format (BGR image)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert the BGR image to a grayscale image
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Apply GaussianBlur to smooth the image and reduce noise
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)

            # Perform edge detection using the Canny algorithm
            edges = cv2.Canny(blurred, 50, 150)

            # Find contours in the edge-detected image
            contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                # Approximate the contour to a polygon with fewer vertices
                approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)

                # Calculate the bounding box around the approximated polygon
                x, y, w, h = cv2.boundingRect(approx)

                if len(approx) == 4:  # Check if the polygon has 4 vertices (rectangle detection)
                    # Draw the rectangle contour in green
                    cv2.drawContours(cv_image, [approx], 0, (0, 255, 0), 2)
                    # Label the detected rectangle
                    cv2.putText(cv_image, "Rectangle", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                elif len(approx) > 8:  # Check if the shape is a circle-like (smooth curve approximation)
                    # Fit a minimum enclosing circle around the contour
                    ((cx, cy), radius) = cv2.minEnclosingCircle(contour)
                    if radius > 10:  # Ignore small circles (noise)
                        # Draw the circle contour in blue
                        cv2.circle(cv_image, (int(cx), int(cy)), int(radius), (255, 0, 0), 2)
                        # Label the detected circle
                        cv2.putText(cv_image, "Circle", (int(cx) - 20, int(cy) - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Display the processed image with detected shapes
            cv2.imshow("Detected Shapes", cv_image)
            cv2.waitKey(1)  # Wait for a brief moment to update the window
        except Exception as e:
            # Log any error that occurs during image processing
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    """
    Main function to initialize the ROS 2 node and start the event loop.
    """
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    node = ImageListener()  # Create an instance of the ImageListener node

    try:
        rclpy.spin(node)  # Keep the node alive and process callbacks
    except KeyboardInterrupt:
        pass  # Handle graceful shutdown on Ctrl+C

    # Cleanup: destroy the node and close OpenCV windows
    node.destroy_node()  # Destroy the node
    rclpy.shutdown()  # Shutdown the ROS 2 client library
    cv2.destroyAllWindows()  # Close all OpenCV windows

if __name__ == '__main__':
    main()  # Entry point of the script

