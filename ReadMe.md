## gps_path_pub

This is a nice helper library made to be used with the [rviz_satellite](https://github.com/gareth-cross/rviz_satellite) RVIZ plugin.
This will take the first GPS reading given and set that as its datum.
It will then publish a path object with the ENU position for each future GPS measurements.
This was made for simple viewing of where a vehicle is during data collection.
To run, make sure you have installed the [rviz_satellite](https://github.com/gareth-cross/rviz_satellite) plugin in your ROS workspace, and then launch the program using the included launch file.
 