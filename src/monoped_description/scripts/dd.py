import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import tf2_ros
from geometry_msgs.msg import TransformStamped

class StaticTFLookupNode(Node):
    def __init__(self):
        super().__init__('static_tf_lookup_node')
        
        # 1. Initialize the TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 2. Create a timer to regularly attempt the lookup
        self.timer = self.create_timer(0.1, self.lookup_static_transform)
        self.get_logger().info('TF Listener node started. Waiting for static transforms...')

    def lookup_static_transform(self):
        # Define the frames for the specific transform you want
        target_frame = 'upper_leg_link'
        source_frame = 'body_link'
        
        try:
            # Look up the transform at the latest available time
            t = self.tf_buffer.lookup_transform(
                target_frame, 
                source_frame, 
                rclpy.time.Time() # Use Time() for the latest time point
            )
            
            # ACCESS THE Z-VALUE HERE
            z_translation = t.transform.translation.z
            
            self.get_logger().info('-------------------------------------------')
            self.get_logger().info(f'Found static Z value for {source_frame} -> {target_frame}:')
            self.get_logger().info(f'Z = {z_translation}')
            self.get_logger().info('-------------------------------------------')
            
            # Since this is a static value, you can stop checking after the first success
            self.timer.cancel() 
            
        except tf2_ros.TransformException as ex:
            # This is common at startup until the /tf_static message is received
            self.get_logger().warn(f'Could not look up transform: {ex}', throttle_duration_sec=5.0)

# Standard main function setup
def main(args=None):
    rclpy.init(args=args)
    node = StaticTFLookupNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()