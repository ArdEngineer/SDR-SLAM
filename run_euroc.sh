gnome-terminal --title="enhance node" -- bash -c "cd src/dce_node; python3 img_enhance_node.py"

gnome-terminal --title="superpoint node" -- bash -c "cd src/sp_node; python3 sp_node.py"

gnome-terminal --title="rviz viewer" -- bash -c "source ./install/setup.bash && source ./install/local_setup.bash; ros2 launch vins vins_rviz.launch.xml"

gnome-terminal --title="estimator" -- bash -c "source ./install/setup.bash && source ./install/local_setup.bash; ros2 run vins vins_node src/config/euroc/euroc_mono_imu_config.yaml"
