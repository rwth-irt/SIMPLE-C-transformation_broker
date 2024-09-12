import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import yaml
from ament_index_python.packages import get_package_share_directory

class FrameRemapper(Node):
    """
    A ROS2 Node that remaps PointCloud2 messages to new frame IDs and publishes them with new topic names.

    This class reads a configuration file to determine which topics should be remapped to which frame IDs 
    and with what new topic names. It creates subscribers for the input topics and publishers for the output 
    topics based on the configuration. When a PointCloud2 message is received, it modifies the frame ID and 
    republishes the message under the new topic name.
    
    Attributes:
        config (dict): The loaded configuration from the YAML file.
        subscribers (list): List of created subscribers.
        pub_list (list): List of created publishers.
        
    Methods:
        __init__(): Initializes the node, reads configuration, and sets up subscribers and publishers.
        pointcloud_callback(msg, new_frame_id, output_topic): Callback function to handle incoming PointCloud2 messages,
                                                              modify their frame ID, and republish them.
    """
    def __init__(self):
        super().__init__('frame_remapper')
        

        # Declare parameter for the configuration file path
        self.declare_parameter('config_file', 'src/config/config.yaml')  # Relative to the installed share directory
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        
        # Load the configuration from the YAML file
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Create subscribers and publishers based on the configuration file
        self.subscribers = []
        self.pub_list  = []
        
        for remap in self.config['remappings']:
            input_topic = remap['input_topic']
            output_topic = remap['output_topic']
            new_frame_id = remap['new_frame_id']

            # output information on screen
            self.get_logger().info(f'Remapping Topic {input_topic} to {output_topic} with new Frame-ID {new_frame_id}')

            subscriber = self.create_subscription(
                PointCloud2,
                input_topic,
                lambda msg, nf=new_frame_id, ot=output_topic, it=input_topic: self.pointcloud_callback(msg, nf, ot),
                10)
            
            publisher = self.create_publisher(PointCloud2, output_topic, 10)
            
            self.subscribers.append(subscriber)
            self.pub_list .append(publisher)

    def pointcloud_callback(self, msg, new_frame_id, output_topic):
        """
        Handles incoming PointCloud2 messages by modifying their frame ID and republishing them.

        This callback function is called whenever a PointCloud2 message is received on one of the subscribed topics. 
        It modifies the frame ID of the incoming message to the new frame ID specified in the configuration and republishes 
        it under the new topic name.

        Args:
            msg (PointCloud2): The received PointCloud2 message.
            new_frame_id (str): The new frame ID to set.
            output_topic (str): The new topic name to publish.

        Example:
            If a PointCloud2 message is received on '/input/pointcloud1', with an original frame ID of 'old_frame_1',
            and the configuration specifies that it should be remapped to 'new_frame_1' and published under '/output/pointcloud1',
            this function will change the frame ID to 'new_frame_1' and publish it under '/output/pointcloud1'.
        """
        # Modify the frame_id of the incoming message
        new_msg = PointCloud2()
        
        new_msg.header = Header()
        new_msg.header.stamp = msg.header.stamp  # Keep the original timestamp
        new_msg.header.frame_id = new_frame_id
        
        # Copy other fields from the original message
        new_msg.height = msg.height 
        new_msg.width = msg.width 
        new_msg.fields = msg.fields 
        new_msg.is_bigendian = msg.is_bigendian 
        new_msg.point_step = msg.point_step 
        new_msg.row_step = msg.row_step 
        new_msg.data = msg.data 
        new_msg.is_dense = msg.is_dense 

        # Find the corresponding publisher and publish the modified message 
        for pub in self.pub_list :
            if pub.topic_name == output_topic:
                pub.publish(new_msg)
                break


def main(args=None):
    rclpy.init(args=args)

    node = FrameRemapper()

    try:
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass

    node.destroy_node()   
    rclpy.shutdown()

if __name__ == '__main__':
    main()
