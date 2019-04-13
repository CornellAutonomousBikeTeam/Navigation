# TODO: 
"""   <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend> make sure these 
two lines are in package.xml and uncommented so that it knows to build the message 

Add message_generation to the CMakeLists.txt files as well 

Basically just follow the tutorial here http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
"""

float32 gps_lat
float32 gps_long
float32 gps_yaw
float32 gps_speed
float32 steer_encoder
float32 imu_yaw
float32 hall_sensor_speed
float32 hall_sensor_tick_counter
float32 hall_sensor_timestamp
