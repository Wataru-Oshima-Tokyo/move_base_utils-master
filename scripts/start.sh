gnome-terminal --tab -t "state_machine" -- bash -c "source /opt/ros/kinetic/setup.bash; source /home/$USER/catkin_ws/devel/setup.bash; python state_machine.py; exec bash"

gnome-terminal --tab -t "int marker" -- bash -c "source /opt/ros/kinetic/setup.bash; source /home/$USER/catkin_ws/devel/setup.bash; python int_marker.py; exec bash"
