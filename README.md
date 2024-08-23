# Extrinsic Calibration Transformation Broker

This node emulates a broker that reads in the transformation from the calibration node and publishs the transformation as a `tf2` message in the ROS2 network.

## Build
Build the ROS package by executing `colcon build` and `source install/setup.bash`.

## Run
Run the node by executing
`ros2 run calibration_transformation_broker calibration_transformation_broker --ros-args -p transformation_file_path:=/path/to/your/transformations.log -p parent_frame:=frame -p child_frame:=frame`

## Frame Remapping
In case all Lidar topics from different sensors were mapped into the same frame, you can use the remapping node to remap each topic into its own frame. Please note that the topic will remain in its original frame, we duplicate the messages into the new frame. This may be sufficient in offline mode with ROS-bags but in online mode, adapt the configuration file of your LiDAR-drivers.
You can call the remapping node via
`ros2 run calibration_transformation_broker frame_remapping --ros-args -p input_topic:=/rslidar_points -p output_topic:=/remapped_rslidar_points -p new_frame_id:=rslidar_points_frame`
