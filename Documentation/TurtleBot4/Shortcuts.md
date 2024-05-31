# Ubuntu, ROS and Turtlebot Shortcuts

By entering "gedit .bashrc" into the terminal after being in the home directery (type cd into terminal) We recommend adding following shortcuts or "aliases" so you don't have to type commands all the time, and by just typing the aliases. Put them at the bottom:


alias rviz='ros2 launch turtlebot4_viz view_model.launch.py'

alias rvizrobot='ros2 launch turtlebot4_viz view_robot.launch.py'

alias teleop='ros2 run teleop_twist_keyboard teleop_twist_keyboard'

alias pointcloud='ros2 launch depthai_ros_driver pointcloud.launch.py'


alias robot35='sshpass -p "turtlebot4" ssh ubuntu@192.168.50.35'
alias robot250='sshpass -p "turtlebot4" ssh ubuntu@192.168.50.35'

alias cameraoff='ros2 service call /oakd/stop_camera std_srvs/srv/Trigger "{}"'
alias cameraon='ros2 launch turtlebot4_bringup oakd.launch.py'

alias undock='ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"'

alias dock='ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"'

alias remoteall250='sshfs ubuntu@192.168.50.250:/ ~/remote_server_all'

alias remote250='sshfs ubuntu@192.168.50.250:/home/ubuntu/ ~/remote_server'

alias remoteall35='sshfs ubuntu@192.168.50.35:/ ~/remote_server_all'

alias remote35='sshfs ubuntu@192.168.50.35:/home/ubuntu/ ~/remote_server'

alias unremote='fusermount -u ~/remote_server'

alias build='colcon build --symlink-install'

alias nav2='ros2 launch hermes nav2.launch.py'
alias rM='ros2 launch hermes start_hermes.launch.py'
alias bcpp='colcon build --packages-select road_navigate_cpp'

alias changerobot='wget -qO - https://raw.githubusercontent.com/turtlebot/turtlebot4_setup/humble/turtlebot4_discovery/configure_discovery.sh | bash <(cat) </dev/tty'

