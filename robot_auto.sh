gnome-terminal --tab --title="Launch" -- bash -c "ros2 launch lbringup bringup.launch.py base_serial_port:=/dev/ttyACM0;
                                                 echo Press any key to close;
                                                 read -n 1"

# sleep 5
# ===================Pake RTAB-Map===================
# gnome-terminal --tab --title="Launch" -- bash -c "ros2 launch lbringup bringup.launch.py base_serial_port:=/dev/ttyACM0 run_rtabmap:=true;
#                                                  echo Press any key to close;
#                                                  read -n 1"

gnome-terminal --tab --title="Control" -- bash -c "source install/setup.bash;
                                                   ros2 run robot_main_pkg robot_main;
                                                   echo Press any key to close;
                                                   read -n 1"


gnome-terminal --tab --title="Target" -- bash -c "source install/setup.bash;
                                                 ros2 run robot_waypoint_pkg robot_waypoint;
                                                 echo Press any key to close;
                                                 read -n 1"


gnome-terminal --tab --title="Purepursuit + Spline" -- bash -c "source install/setup.bash;
                                                ros2 run  purpuresuit_pkg purpuresuit;
                                                echo Press any key to close;
                                                read -n 1"


gnome-terminal --tab --title="Gui" -- bash -c "source install/setup.bash;
                                                ros2 run robot_gui_pkg robot_gui;
                                                echo Press any key to close;
                                                read -n 1"

#
