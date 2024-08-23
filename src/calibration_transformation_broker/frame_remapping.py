import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import yaml
from ament_index_python.packages import get_package_share_directory

class FrameRemapper(Node):
    def __init__(self):
        super().__init__('frame_remapper')
        

        # Declare parameter for the configuration file path
        self.declare_parameter('config_file', 'config/config.yaml')  # Relative to the installed share directory
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

            subscriber = self.create_subscription(
                PointCloud2,
                input_topic,
                lambda msg, nf=new_frame_id, ot=output_topic: self.pointcloud_callback(msg, nf, ot),
                10)
            
            publisher = self.create_publisher(PointCloud2, output_topic, 10)
            
            self.subscribers.append(subscriber)
            self.pub_list .append(publisher)

    def pointcloud_callback(self, msg, new_frame_id, output_topic):
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
