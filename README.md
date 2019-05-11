# Heron-position-controller
Python file for controlling position of Heron using GPS and ROS topics.

cmd_drive = command for thrusters
cmd_helm = Thrust, Yaw rate
Not a perfect circle but tries
cmd_wrench = Force vector, Torque Vector
Reliable and rotates in the correct position
cmd_course = yaw, speed = yaw doesn't work. Speed seems to work.


/imu/compass_heading = data = nan
/imu/declination = Nothing
/imu/raw_compass_heading = West is 0, North is 1.57, East is 3.14/-3.14, South is -1.57
/imu/data = Quaternion Orientation, Angular velocities, Linear acceleration  
/imu/mag = magnetometer data in x,y,z              
/imu/rpy = vector x,y,z
/imu/data_compass =  angular velocity x,y,z, linear acceleration x,y,z         
/imu/mag_calib            
/imu/temperature

In joystick, both the thrusters don't work together.

--------------------------------------------------------------------------------

 
Node [/um6_driver]

Publications: 
 * /imu/data [sensor_msgs/Imu]
 * /imu/mag [geometry_msgs/Vector3Stamped]
 * /imu/rpy [geometry_msgs/Vector3Stamped]
 * /imu/temperature [std_msgs/Float32]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: None

Services: 
 * /imu/reset
 * /um6_driver/get_loggers
 * /um6_driver/set_logger_level


contacting node http://cpr-m300-0017:44418/ ...
Pid: 933
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /imu/data
    * to: /imu_compass
    * direction: outbound
    * transport: TCPROS
 * topic: /imu/mag
    * to: /imu_compass
    * direction: outbound
    * transport: TCPROS
    
--------------------------------------------------------------------------------
Node [/imu_compass]
Publications: 
 * /imu/compass_heading [std_msgs/Float32]
 * /imu/data_compass [sensor_msgs/Imu]
 * /imu/mag_calib [geometry_msgs/Vector3Stamped]
 * /imu/raw_compass_heading [std_msgs/Float32]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /imu/data [sensor_msgs/Imu]
 * /imu/declination [std_msgs/Float32]
 * /imu/mag [geometry_msgs/Vector3Stamped]
 * /tf [tf2_msgs/TFMessage]
 * /tf_static [tf2_msgs/TFMessage]

Services: 
 * /imu_compass/get_loggers
 * /imu_compass/set_logger_level


contacting node http://cpr-m300-0017:43057/ ...
Pid: 938
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /imu/data_compass
    * to: /robot_pose_ekf
    * direction: outbound
    * transport: TCPROS
 * topic: /imu/data_compass
    * to: /controller
    * direction: outbound
    * transport: TCPROS
 * topic: /tf
    * to: /base_to_basefootprint_tf (http://cpr-m300-0017:45542/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tf
    * to: /robot_state_publisher (http://cpr-m300-0017:39396/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tf
    * to: /navsat_to_gps_tf (http://cpr-m300-0017:34737/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tf
    * to: /robot_pose_ekf (http://cpr-m300-0017:45073/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tf_static
    * to: /robot_state_publisher (http://cpr-m300-0017:39396/)
    * direction: inbound
    * transport: TCPROS
 * topic: /imu/data
    * to: /um6_driver (http://cpr-m300-0017:44418/)
    * direction: inbound
    * transport: TCPROS
 * topic: /imu/mag
    * to: /um6_driver (http://cpr-m300-0017:44418/)
    * direction: inbound
    * transport: TCPROS
 * topic: /imu/declination
    * to: /imu/declination_compute (http://cpr-m300-0017:46714/)
    * direction: inbound
    * transport: TCPROS

--------------------------------------------------------------------------------
Node [/controller]
Publications: 
 * /cmd_drive [heron_msgs/Drive]
 * /eff_wrench [geometry_msgs/Wrench]
 * /rosout [rosgraph_msgs/Log]
 * /yaw_debug [geometry_msgs/Vector3]
 * /yaw_rate_debug [geometry_msgs/Vector3]

Subscriptions: 
 * /cmd_course [unknown type]
 * /cmd_helm [unknown type]
 * /cmd_wrench [unknown type]
 * /imu/data_compass [sensor_msgs/Imu]

Services: 
 * /controller/get_loggers
 * /controller/set_logger_level


contacting node http://cpr-m300-0017:39290/ ...
Pid: 1000
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /cmd_drive
    * to: /rosserial_server
    * direction: outbound
    * transport: TCPROS
 * topic: /imu/data_compass
    * to: /imu_compass (http://cpr-m300-0017:43057/)
    * direction: inbound
    * transport: TCPROS

