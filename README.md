libfreenect2pclgrabber
======================

Conversion of Giacomo Dabisias ROS1 grabber to ROS2

**colcon build** the folder

then **ros2 run kinect2grabber test_ros**

If using rviz2 to visalize, remember to run the following in a separate tab:

**ros2 run tf2_ros static_transform_publisher "0" "0" "0" "0" "0" "0" "map" "kinect2"**

A grabber for the kinect2 which returns images and Point Clouds. 
There is the support for multiple kinects and a ROS interface.

For any questions don't hesitate to contact me at giacomo.dabisias@gmail.com.

If you have issues with multiple kinects execute "echo 64 > /sys/module/usbcore/parameters/usbfs_memory_mb". If this works you can apply this patch permanently as follows:

Open the /etc/default/grub file in any text editor. Find and replace:
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"
with this:

GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=1000"

Update grub with these settings:
$ sudo update-grub

REMEMBER TO EXECUTE sudo make install in the libfreenect2 build folder to copy all files in /usr/local
