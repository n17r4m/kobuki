How TO:  
  
0. catkin_make your workspace after first pull; source env  
1. Minimal, `roslaunch turtlebot_bringup minimal.launch`  
2. Using our own map, `roslaunch turtlebot_navigation amcl_demo.launch map_file:=./comp3/maps/robot9000_CSC2ndFlr_map.yaml` should be your full address to the map    
3. To visualize, `roslaunch turtlebot_rviz_launchers view_navigation.launch`  
4. To do circle run, `rosrun comp3 navi.py` or `./comp3/src/navi.py`  
