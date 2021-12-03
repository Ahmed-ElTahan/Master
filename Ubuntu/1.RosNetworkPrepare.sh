# Change your linux ethernet interface ip address statically using:
sudo ifconfig enp4s0f0   192.168.1.1 netmask 255.255.255.0

# assign to ros master the same ip address using: 
export ROS_MASTER_URI=http://192.168.1.1:11311 >> ~/.bashrc
export ROS_HOSTNAME=192.168.1.1 >> ~/.bashrc
export ROS_IP=192.168.1.1 >> ~/.bashrc

# start ROS
roscore
