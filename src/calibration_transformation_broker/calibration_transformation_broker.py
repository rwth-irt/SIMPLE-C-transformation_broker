import rclpy
from rclpy.node import Node
import os
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf2_ros

class CalibrationTransformationBroker(Node):
    def __init__(self):
        super().__init__('calibration_transformation_broker')
        self.declare_parameter('transformation_file_path', '')
        transformation_file_path = self.get_parameter('transformation_file_path').get_parameter_value().string_value
        
        self.declare_parameter('parent_frame', 'lidar_imu')
        self.parent_frame_id = self.get_parameter('parent_frame').get_parameter_value().string_value

        self.declare_parameter('child_frame', 'lidar_left_front')
        self.child_frame_id = self.get_parameter('child_frame').get_parameter_value().string_value


        if not os.path.isfile(transformation_file_path):
            self.get_logger().error(f"File {transformation_file_path} does not exist")
            return
        
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        self.read_last_transformation(transformation_file_path)
    
    def read_last_transformation(self, file_path):
        with open(file_path, 'r') as file:
            lines = file.readlines()
        
        # Reverse the lines to find the last transformation from bottom
        lines.reverse()
        
        R = None
        t = None
        
        for i in range(len(lines)):
            if "New transformation for" in lines[i]:
                # Read Rotation matrix R
                R_start_index = i - 3  # The rotation matrix starts two lines after the match
                
                r_1 = lines[R_start_index].strip()[1:-1].split()
                r_2 = lines[R_start_index-1].strip()[1:-1].split()
                r_3 = lines[R_start_index-2].strip()[1:-1].split()

                R = np.array([
                    [float(r_1[8]), float(r_1[9]), float(r_1[10])],
                    [float(r_2[0]), float(r_2[1]), float(r_2[2])],
                    [float(r_3[0]), float(r_3[1]), float(r_3[2].split("]")[0])]
                ])
                
                # Read Translation vector t
                t_line_index = R_start_index - 5  # The translation vector starts four lines after the rotation matrix
                
                t_1 = lines[t_line_index][1:-2].split()
                t = np.array([float(t_1[7].split("[")[1]),
                            float(t_1[8]),
                            float(t_1[9])])
                
                break
        
        if R is not None and t is not None:
            self.get_logger().info(f'Last Transformation found:')
            self.get_logger().info(f'R: {R}')
            self.get_logger().info(f't: {t}')
            
            self.broadcast_transform(R, t, self.parent_frame_id, self.child_frame_id)
        else:
            self.get_logger().error('No valid transformation found in the file')

    def broadcast_transform(self, R, t, parent_frame_id, child_frame_id):
        transform_stamped = TransformStamped()

        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = parent_frame_id
        transform_stamped.child_frame_id = child_frame_id

        transform_stamped.transform.translation.x = t[0]
        transform_stamped.transform.translation.y = t[1]
        transform_stamped.transform.translation.z = t[2]

        # Convert rotation matrix to quaternion using scipy.spatial.transform.Rotation
        try:
            from scipy.spatial.transform import Rotation as R_
            rotation_matrix_flattened=R.flatten()
            r_= R_.from_matrix(rotation_matrix_flattened.reshape(3 ,3))
            
            q=r_.as_quat()
            
            transform_stamped.transform.rotation.x = q[0]
            transform_stamped.transform.rotation.y = q[1]
            transform_stamped.transform.rotation.z = q[2]
            transform_stamped.transform.rotation.w = q[3]
        except ImportError:
            pass

        try:
            while rclpy.ok():
                now=self.get_clock().now().to_msg()
                transform_stamped.header.stamp=now

                self.tf_static_broadcaster.sendTransform(transform_stamped)

                rclpy.spin_once(self)
        except ImportError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node=CalibrationTransformationBroker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
     main()   
