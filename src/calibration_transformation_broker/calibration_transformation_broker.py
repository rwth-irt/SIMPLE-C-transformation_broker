import yaml
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R_
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class CalibrationTransformationBroker(Node):
    """
    A ROS2 Node that publishes static transformations between frames based on a configuration file.

    This class reads a configuration file to determine the transformation chains between different frames. 
    It then reads the transformation data from specified log files, combines them if necessary, and publishes 
    the final transformations as static transforms using ROS2's StaticTransformBroadcaster.
    
    Attributes:
        config (dict): The loaded configuration from the YAML file.
        tf_static_broadcaster (StaticTransformBroadcaster): The broadcaster used to publish static transforms.
        
    Methods:
        __init__(): Initializes the node, reads configuration, and sets up the broadcasting of transformations.
        read_last_transformation(file_path, invert): Reads the last transformation from a log file and optionally inverts it.
        create_transform(R, t, parent_frame_id, child_frame_id): Creates a static transformation using the given rotation and translation.
        generate_transformations_from_config(transformations): Generates and publishes combined static transformations from a given configuration.
        create_reference_frame(): Creates a static transformation frame for a given reference frame relative to 'base_link'.
    """
    def __init__(self):
        super().__init__('calibration_transformation_broker')
        self.declare_parameter('config_file', 'src/config/config.yaml')  # Relative to the installed share directory
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        
        # Load the configuration from the YAML file
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)

        # Create static reference frame
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        # Generate the transformations from config file and publish
        self.generate_transformations_from_config(self.config["Transformations"])
    
    def read_last_transformation(self, file_path, invert):
        """
        Reads the last transformation from a log file and optionally inverts it.

        This function reads a log file containing transformation data and extracts the 
        last recorded transformation. The extracted transformation consists of a rotation 
        matrix (R) and a translation vector (t). If specified by the 'invert' parameter, 
        the transformation is inverted before being returned.

        Args:
            file_path (str): The path to the log file containing the transformation data.
            invert (int): An integer indicating whether to invert the transformation.
                        - 1: Do not invert the transformation.
                        - -1: Invert the transformation.

        Returns:
            tuple: A tuple containing two elements:
                - R (numpy.ndarray): A 3x3 rotation matrix representing the orientation.
                - t (numpy.ndarray): A 3-element translation vector representing the position.
                
                If no valid transformation is found, returns (None, None).

        Example:
            Given a log file with content indicating transformations:
            
            ```
            New transformation for frame ...
            [0.9998, 0.0175, -0.0123]
            [-0.0176, 0.9998, 0.0007]
            [0.0122, -0.0012, 0.9999]
            
            Translation vector:
            [1.2345, -0.6789, 2.3456]
            ```

            The function will read this data and return:
            
            R = [[0.9998, 0.0175, -0.0123],
                [-0.0176, 0.9998, 0.0007],
                [0.0122, -0.0012, 0.9999]]
                
            t = [1.2345, -0.6789, 2.3456]

            If 'invert' is set to -1:
            
            R_inv = np.linalg.inv(R)
            t_inv = -np.dot(R_inv ,t)
            
            The function will return these inverted values instead.
        """
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
            if invert == -1:
                R_inv = np.linalg.inv(R)
                t_inv = -np.dot(R_inv, t)

                self.get_logger().info(f'Last Transformation found (inverted):')
                self.get_logger().info(f'R: {R_inv}')
                self.get_logger().info(f't: {t_inv}')
                
                return (R_inv ,t_inv )
            
            else: 
                self.get_logger().info(f'Last Transformation found:')
                self.get_logger().info(f'R: {R}')
                self.get_logger().info(f't: {t}')
                return (R ,t)
        
        else: 
            self.get_logger().error('No valid transformation found in the file')
            return (None,None)

    def create_transform(self, R, t, source_frame_id, target_frame_id):
        """
        Creates a static transformation using the given rotation and translation.

        This function creates a TransformStamped message from the provided rotation matrix (R) 
        and translation vector (t), and publishes it as a static transform between the specified
        source and target frames. The rotation matrix is converted to a quaternion for the ROS2 message.

        Args:
            R (numpy.ndarray): A 3x3 rotation matrix representing the orientation.
            t (numpy.ndarray): A 3-element translation vector representing the position.
            source_frame_id (str): The ID of the source frame.
            target_frame_id (str): The ID of the child frame.

        Example:
            Given a rotation matrix R and translation vector t:
            
            R = [[0.9998, 0.0175, -0.0123],
                [-0.0176, 0.9998, 0.0007],
                [0.0122, -0.0012, 0.9999]]
                
            t = [1.2345, -0.6789, 2.3456]

            And frame IDs:
            
            source_frame_id = "world"
            target_frame_id = "robot_base"

            The function will publish this transformation as a static transform from 'world' to 'robot_base'.
        """
        transform_stamped = TransformStamped()

        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = target_frame_id
        transform_stamped.child_frame_id = source_frame_id

        transform_stamped.transform.translation.x = t[0]
        transform_stamped.transform.translation.y = t[1]
        transform_stamped.transform.translation.z = t[2]

        # Convert rotation matrix to quaternion using scipy.spatial.transform.Rotation
        try:
            rotation_matrix_flattened=R.flatten()
            r_= R_.from_matrix(rotation_matrix_flattened.reshape(3 ,3))
            
            q=r_.as_quat()
            
            transform_stamped.transform.rotation.x = q[0]
            transform_stamped.transform.rotation.y = q[1]
            transform_stamped.transform.rotation.z = q[2]
            transform_stamped.transform.rotation.w = q[3]
        except ImportError:
            pass

        return transform_stamped

    def create_reference_frame(self):
        """
        Creates a static transformation frame for a given reference frame relative to 'base_link'.
        """
        if self.config["Reference Frame"] is not None:
            transform_stamped = TransformStamped()

            transform_stamped.header.stamp = self.get_clock().now().to_msg()
            transform_stamped.header.frame_id = 'base_link'  # Existing parent frame
            transform_stamped.child_frame_id = self.config["Reference Frame"]  # New child frame

            # Set translation (adjust these values as needed)
            transform_stamped.transform.translation.x = 0.0
            transform_stamped.transform.translation.y = 0.0
            transform_stamped.transform.translation.z = 1.5  # Example value; adjust as needed

            try:
                from scipy.spatial.transform import Rotation as R_
                q = R_.from_euler('xyz', [0, 0, 0]).as_quat()  # No rotation
                
                transform_stamped.transform.rotation.x = q[0]
                transform_stamped.transform.rotation.y = q[1]
                transform_stamped.transform.rotation.z = q[2]
                transform_stamped.transform.rotation.w = q[3]
            except ImportError:
                pass

            self.get_logger().info(f'Created new reference frame {transform_stamped.child_frame_id}\n')

            return transform_stamped
        else: 
            self.get_logger().info(f'Reference Frame already given ({self.config["Reference Frame"]})\n')
            return None

    def generate_transformations_from_config(self, transformations):
        """
        Generates and publishes combined static transformations from a given configuration.
        Will also generate a reference frame given in the config file.

        This function reads transformation chains specified in the configuration file and 
        computes the final transformation for each chain. Each transformation chain consists 
        of multiple individual transformations that are combined sequentially. Transformation chains consisting of only one transformation (no chain) are also accepted.
        The final transformations are then published as static transforms using ROS2's StaticTransformBroadcaster.

        Args (from config-file):
            transformations (list): A list of dictionaries where each dictionary specifies a 
                                    transformation chain with the following keys:
                - frame_chain (str): Comma-separated string of frame IDs representing the chain.
                - invert_chain (str): Comma-separated string of integers (1 or -1) indicating whether 
                                    to invert each corresponding transformation.
                - path_chain (str): Comma-separated string of file paths to log files containing the 
                                    individual transformations.

        Example:
            Given a configuration entry:
            {
                "frame_chain": "lidar_left_back_frame, lidar_top_frame",
                "invert_chain": "1, 1, -1, 1",
                "path_chain": "/path/to/left_back_to_front.log,/path/to/left_front_to_front.log,/path/to/right_front_to_front.log,/path/to/right_front_to_top.log"
            }

            The function will compute the combined transformation from 'lidar_left_back_frame' to 'lidar_top_frame'
            by sequentially reading and combining the individual transformations specified in 'path_chain', applying
            inversion as indicated in 'invert_chain'. The resulting final transformation is then published as a static transform.
        """
        transformation_list = []

        # Create static frame for given master frame
        refenerece_frame_transformation = self.create_reference_frame()
        if refenerece_frame_transformation is not None: transformation_list.append(refenerece_frame_transformation)

        self.get_logger().info(f'Starting to process {len(transformations)} transformation chains\n')

        for idx, transformation in enumerate(transformations):
            self.get_logger().info(f'Processing Transformation Chain {idx + 1}/{len(transformations)}')
            
            frame_chain = transformation['frame_chain'].split(', ')
            invert_chain = transformation['invert_chain']
            path_chain = transformation['path_chain'].split(',')

            final_R = np.eye(3)
            final_t = np.zeros(3)

            source_frame_id = frame_chain[0]
            target_frame_id = frame_chain[-1]

            self.get_logger().info(f'New Transformation Chain: {frame_chain}\n')

            for i in range(len(path_chain) - 1, -1, -1):
                self.get_logger().info(f'Processing Chain-Part {i+1}\n')
                file_path = path_chain[i]
                invert = invert_chain[i]

                R, t = self.read_last_transformation(file_path, invert)
                if R is not None and t is not None:
                    # Combine the current transformation with the final transformation
                    final_t += np.dot(final_R, t)
                    final_R = np.dot(R, final_R)

            self.get_logger().info(f'Saving Transformation...\n')

            # Publish the combined (final) transformation
            transformation_chain = self.create_transform(final_R, final_t, source_frame_id, target_frame_id)
            transformation_list.append(transformation_chain)

        self.get_logger().info(f'Finished processing all transformation chains.\n')

        # Publish all transformations in parallel 
        self.tf_static_broadcaster.sendTransform(transformation_list)

        self.get_logger().info(f'Publishing all Transformations\n')


def main(args=None):
    rclpy.init(args=args)
    node=CalibrationTransformationBroker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
     main()   
