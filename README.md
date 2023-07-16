# chonk_pushing

Wholebody Inverse kinematics, use 5-order spline to generate the trajectory in joint space
1. roslaunch chonk_pushing gazebo_wholebody.launch
2. roslaunch chonk_pushing action_servers_wholebody.launch
4. rosrun chonk_pushing action_client_cmd_pose_wholebody.py

Without admittance control, MPC tracking joint-space trajectory
1. roslaunch chonk_pushing gazebo_planner_MPC_BC.launch
2. roslaunch chonk_pushing action_servers_MPC_BC.launch
4. rosrun chonk_pushing action_client_cmd_pose_MPC_BC_pick.py

Without admittance control, MPC tracking operational trajectory
1. roslaunch chonk_pushing gazebo_planner_MPC_BC_operational.launch
2. roslaunch chonk_pushing action_servers_MPC_BC_operational.launch
4. rosrun chonk_pushing action_client_cmd_pose_MPC_BC_operational_pick.py

With admittance control in global frame, MPC tracking operational trajectory
1. roslaunch chonk_pushing gazebo_planner_MPC_BC_operational_pick_sensorAD.launch 
2. roslaunch chonk_pushing action_servers_MPC_BC_operational_pick_sensorAD.launch 
3. roslaunch chonk_dynamics chonk_dynamics.launch
4. rosrun chonk_pushing action_client_cmd_pose_MPC_BC_operational_pick_sensorAD.py 

With admittance control in global frame and obstacle avoidance, MPC tracking operational trajectory
1. roslaunch chonk_pushing gazebo_planner_MPC_BC_operational_pick_sensorAD_obstacle_longhorizon.launch 
2. roslaunch chonk_pushing action_servers_MPC_BC_operational_pick_sensorAD_obstacle_longhorizon.launch 
3. roslaunch chonk_dynamics chonk_dynamics.launch
4. rosrun chonk_pushing action_client_cmd_pose_MPC_BC_operational_pick_sensorAD_obstacle_longhorizon.py 

With motion planning of two end-effectors' positions, MPC tracking operational trajectory with admittance control in global frame
1. roslaunch chonk_pushing gazebo_planner_MPC_BC_operational_pick_sensorAD_obstacle_wholetrajectory.launch
2. roslaunch chonk_pushing action_servers_MPC_BC_operational_pick_sensorAD_obstacle_wholetrajectory.launch 
3. roslaunch chonk_dynamics chonk_dynamics.launch
4. rosrun chonk_pushing action_client_cmd_pose_MPC_BC_operational_pick_sensorAD_obstacle_wholetrajectory.py 

With motion planning of two end-effectors' positions and orientation, MPC tracking operational trajectory with admittance control in global frame
1. roslaunch chonk_pushing gazebo_planner_MPC_BC_operational_pick_sensorAD_obstacle_wholetrajectory_withOrientation.launch
2. roslaunch chonk_pushing action_servers_MPC_BC_operational_pick_sensorAD_obstacle_wholetrajectory_withOrientation.launch 
3. roslaunch chonk_dynamics chonk_dynamics.launch
4. rosrun chonk_pushing action_client_cmd_pose_MPC_BC_operational_pick_sensorAD_obstacle_wholetrajectory_withOrientation.py 

########### Below relates to admittance control in local frame ###########################

With admittance control in local frame, MPC tracking operational trajectory
1. roslaunch chonk_pushing gazebo_planner_MPC_BC_operational_pick_localsensorAD.launch or
   roslaunch chonk_pushing gazebo_planner_MPC_BC_operational_pick_localsensorAD_turn.launch
2. roslaunch chonk_pushing action_servers_MPC_BC_operational_pick_localsensorAD.launch 
3. roslaunch chonk_dynamics chonk_dynamics.launch
4. rosrun chonk_pushing action_client_cmd_pose_MPC_BC_operational_pick_localsensorAD.py or
   rosrun chonk_pushing action_client_cmd_pose_MPC_BC_operational_pick_localsensorAD_turn.py 

With admittance control in local frame and obstacle avoidance, MPC tracking operational trajectory
1. roslaunch chonk_pushing gazebo_planner_MPC_BC_operational_pick_localsensorAD_obstacle_longhorizon.launch or
   roslaunch chonk_pushing gazebo_planner_MPC_BC_operational_pick_localsensorAD_obstacle_longhorizon_turn.launch
2. roslaunch chonk_pushing action_servers_MPC_BC_operational_pick_localsensorAD_obstacle_longhorizon.launch 
3. roslaunch chonk_dynamics chonk_dynamics.launch
4. rosrun chonk_pushing action_client_cmd_pose_MPC_BC_operational_pick_localsensorAD_obstacle_longhorizon.py or
   rosrun chonk_pushing action_client_cmd_pose_MPC_BC_operational_pick_localsensorAD_obstacle_longhorizon_turn.py

With motion planning of two end-effectors' positions, MPC tracking operational trajectory with admittance control in local frame
1. roslaunch chonk_pushing gazebo_planner_MPC_BC_operational_pick_localsensorAD_obstacle_wholetrajectory.launch or 
   roslaunch chonk_pushing gazebo_planner_MPC_BC_operational_pick_localsensorAD_obstacle_wholetrajectory_turn.launch
   roslaunch chonk_pushing gazebo_planner_MPC_BC_operational_pick_localsensorAD_obstacle_wholetrajectory_turn_experimentgazebo.launch
2. roslaunch chonk_pushing action_servers_MPC_BC_operational_pick_localsensorAD_obstacle_wholetrajectory.launch
   roslaunch chonk_pushing action_servers_MPC_BC_operational_pick_localsensorAD_obstacle_wholetrajectory_turn_experimentgazebo.launch 
3. roslaunch chonk_dynamics chonk_dynamics.launch
4. rosrun chonk_pushing action_client_cmd_config_wholetrajectory.py
5. rosrun chonk_pushing action_client_cmd_pose_MPC_BC_operational_pick_localsensorAD_obstacle_wholetrajectory.py or
   rosrun chonk_pushing action_client_cmd_pose_MPC_BC_operational_pick_localsensorAD_obstacle_wholetrajectory_turn.py 
   rosrun chonk_pushing action_client_cmd_pose_MPC_BC_operational_pick_localsensorAD_obstacle_wholetrajectory_turn_experimentgazebo.py 
   
With motion planning of two end-effectors' positions and orientation, MPC tracking operational trajectory with admittance control in local frame
1. roslaunch chonk_pushing gazebo_planner_MPC_BC_operational_pick_localsensorAD_obstacle_wholetrajectory_withOrientation.launch or
   roslaunch chonk_pushing gazebo_planner_MPC_BC_operational_pick_localsensorAD_obstacle_wholetrajectory_withOrientation_turn.launch or
   roslaunch chonk_pushing gazebo_planner_MPC_BC_operational_pick_localsensorAD_obstacle_wholetrajectory_withOrientation_turn_experimentgazebo.launch
2. roslaunch chonk_pushing action_servers_MPC_BC_operational_pick_localsensorAD_obstacle_wholetrajectory_withOrientation.launch or
   roslaunch chonk_pushing action_servers_MPC_BC_operational_pick_localsensorAD_obstacle_wholetrajectory_withOrientation_turn_experimentgazebo.launch 
3. roslaunch chonk_dynamics chonk_dynamics.launch
4. rosrun chonk_pushing action_client_cmd_config_wholetrajectory.py
5. rosrun chonk_pushing action_client_cmd_pose_MPC_BC_operational_pick_localsensorAD_obstacle_wholetrajectory_withOrientation.py or
   rosrun chonk_pushing action_client_cmd_pose_MPC_BC_operational_pick_localsensorAD_obstacle_wholetrajectory_withOrientation_turn.py or 
   rosrun chonk_pushing action_client_cmd_pose_MPC_BC_operational_pick_localsensorAD_obstacle_wholetrajectory_withOrientation_turn_experimentgazebo.py 
   
########### Below relates to 6-DOF admittance control in local frame ###########################

With 6-DOF admittance control, MPC tracking operational trajectory
1. roslaunch chonk_pushing gazebo_planner_MPC_BC_operational_pick_localsensorAD_6DOF_turn.launch 
2. roslaunch chonk_pushing action_servers_MPC_BC_operational_pick_localsensorAD_6DOF.launch 
3. roslaunch chonk_dynamics chonk_dynamics.launch
4. rosrun chonk_pushing action_client_cmd_pose_MPC_BC_operational_pick_localsensorAD_6DOF_turn.py 

With 6-DOF admittance control and obstacle avoidance, MPC tracking operational trajectory
1. roslaunch chonk_pushing gazebo_planner_MPC_BC_operational_pick_localsensorAD_6DOF_obstacle_longhorizon.launch or
   roslaunch chonk_pushing gazebo_planner_MPC_BC_operational_pick_localsensorAD_6DOF_obstacle_longhorizon_turn.launch
2. roslaunch chonk_pushing action_servers_MPC_BC_operational_pick_localsensorAD_6DOF_obstacle_longhorizon.launch 
3. roslaunch chonk_dynamics chonk_dynamics.launch
4. rosrun chonk_pushing action_client_cmd_pose_MPC_BC_operational_pick_localsensorAD_6DOF_obstacle_longhorizon.py or
   rosrun chonk_pushing action_client_cmd_pose_MPC_BC_operational_pick_localsensorAD_6DOF_obstacle_longhorizon_turn.py

With 6-DOF admittance control and obstacle avoidance, with motion planning of two end-effectors' positions and orientation, MPC tracking operational trajectory 
1. roslaunch chonk_pushing gazebo_planner_MPC_BC_operational_pick_localsensorAD_6DOF_obstacle_wholetrajectory_withOrientation.launch or
   roslaunch chonk_pushing gazebo_planner_MPC_BC_operational_pick_localsensorAD_6DOF_obstacle_wholetrajectory_withOrientation_turn.launch or
2. roslaunch chonk_pushing action_servers_MPC_BC_operational_pick_localsensorAD_6DOF_obstacle_wholetrajectory_withOrientation.launch or
   roslaunch chonk_pushing action_servers_MPC_BC_operational_pick_localsensorAD_6DOF_obstacle_wholetrajectory_withOrientation_turn.launch or
3. roslaunch chonk_dynamics chonk_dynamics.launch
4. rosrun chonk_pushing action_client_cmd_config_wholetrajectory.py
5. rosrun chonk_pushing action_client_cmd_pose_MPC_BC_operational_pick_localsensorAD_6DOF_obstacle_wholetrajectory_withOrientation.py or
   rosrun chonk_pushing action_client_cmd_pose_MPC_BC_operational_pick_localsensorAD_6DOF_obstacle_wholetrajectory_withOrientation_turn.py or 

########### Add obstacle avoidance in motion tracking ###########################

With 6-DOF admittance control and obstacle avoidance, with motion planning of two end-effectors' positions and orientation, MPC tracking operational trajectory 
1. roslaunch chonk_pushing gazebo_planner_MPC_BC_operational_pick_localsensorAD_6DOF_bothobstacle_wholetrajectory_withOrientation_turn.launch 
2. roslaunch chonk_pushing action_servers_MPC_BC_operational_pick_localsensorAD_6DOF_bothobstacle_wholetrajectory_withOrientation_turn.launch
3. roslaunch chonk_dynamics chonk_dynamics.launch
4. rosrun chonk_pushing action_client_cmd_config_wholetrajectory.py
5. rosrun chonk_pushing action_client_cmd_pose_MPC_BC_operational_pick_localsensorAD_6DOF_bothobstacle_wholetrajectory_withOrientation_turn.py


   
########### Parallel Programming ###########################
# - To launch EVA robot to pick and place a box while avoiding an obstacle using admittance control:
![Alt text](/pics/EVApicking.png "EVApicking")
Description:
- With 6-DOF admittance control and obstacle avoidance, with motion planning of two end-effectors' positions and orientation, MPC tracking operational trajectory using parallel programming 

    1. Spawn EVA Gazebo model:
    - roslaunch chonk_pushing gazebo_planner_tracking_MPC_BC_operational_pick_localsensorAD_6DOF_bothobstacle_wholetrajectory_withOrientation_turn.launch 
    2. Start motion_planning server:
    - roslaunch chonk_pushing action_servers_planner_MPC_BC_operational_pick_localsensorAD_6DOF_bothobstacle_wholetrajectory_withOrientation_turn.launch
    3. Start motion_tracking server with admittance control:
    - roslaunch chonk_pushing action_servers_tracking_MPC_BC_operational_pick_localsensorAD_6DOF_bothobstacle_wholetrajectory_withOrientation_turn.launch
    4. Start EVA_dynamics node for force signal processing:
    - roslaunch chonk_dynamics chonk_dynamics.launch
    5. Config joints to the beginning state:
    - rosrun robot_control action_client_cmd_config_wholetrajectory.py
    6. Begin to grasp and place a box with admittance control:
    - rosrun chonk_pushing action_client_planner_tracking_MPC_BC_operational_pick_localsensorAD_6DOF_bothobstacle_wholetrajectory_withOrientation_turn.py 


