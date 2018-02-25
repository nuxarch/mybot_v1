#!/bin/sh
sudo pkill socat
sleep 1
# sudo pkill socat
sleep 1
sudo socat pty,link=/dev/robot,ignoreeof,user=fat,group=dialout,mode=777,raw,echo=0 tcp:192.168.4.1:23 &
sleep 1
# roslaunch ev_r1 rviz_control.launch
# roslaunch ev_r1 rviz_control.launch
