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
