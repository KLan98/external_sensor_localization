# external_sensor_localization

Bachelor's thesis at Frankfurt UAS. Provide a more reliable localization solution for robots using external Marvelmind's ultrasonic sensors. Utilize the Extended Kalman Filter to provide more accurate readings than AMCL.

# Hardware

1. Turtlebot3 burger
2. Marvelmind starter set (including 5 ultrasonic sensors and 1 router)

# Requirements

The following packages are recommended 

1. [robot_localization](http://wiki.ros.org/robot_localization), go to my robot_localization_ekf repo and copy ekf_map.yaml and ekf_odom.yaml files and paste them to params folder of robot_localization package
2. [marvelmind_nav](https://bitbucket.org/marvelmind_robotics/ros_marvelmind_package) (optional since I have put most of what is required for the operation of Marvelmind sensors into external_sensor_localization package)

# Result

Click on the following [link](https://drive.google.com/file/d/1HUOCoFdYrak2g-GNLPjxd7Xol8M04kZN/view?usp=sharing) to see how well this system operates

# License

[MIT](https://opensource.org/licenses/MIT)
