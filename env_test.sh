#!/bin/bash 

# run the appropriate test bag file
test_bag="~/Bagfiles/tiffanys_bagfiles_summer_2015/2_ppl_etu_ex2.bag"

# start roscore
screen -S roscore_screen -dm bash -c "source /opt/ros/indigo/setup.bash; roscore"

# play rosbag
#gnome-terminal -e "rosbag play -l ~/Bagfiles/tiffanys_bagfiles_summer_2015/2_ppl_etu_ex2.bag"
screen -S rosbag_screen -dm bash -c "source /opt/ros/indigo/setup.bash; rosbag play -l --start=28 --duration=17 ~/Bagfiles/tiffanys_bagfiles_summer_2015/2_ppl_etu_ex2.bag"

echo "All screens started"
screen -list

echo "press Enter to quit"
read quit

# could be dangerous to kill all screens
#killall screen
screen -X -S rosbag_screen quit
screen -X -S roscore_screen quit


#echo "detached roscore screen"
#screen -R rc_screen
#roscore
#bash
#roscore

#echo "attached roscore screen"
