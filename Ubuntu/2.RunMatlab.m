clc; clear all; close all;
% assign to matlab ros, an ip address in same subnet as the master in RosNetworkPrepare.sh using :

setenv('ROS_IP','192.168.1.100:11311')
setenv('ROS_IP','192.168.1.100')

rosinit('192.168.1.1', 11311)
rosnode list
rostopic list
rosservice list

