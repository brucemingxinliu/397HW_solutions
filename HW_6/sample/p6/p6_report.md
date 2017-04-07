Running procedures: 


roslaunch mobot_urdf mobot_in_pen.launch

rosrun mobot_pub_des_state open_loop_controller

rosrun mobot_pub_des_state pub_des_state_path_client

rosrun mobot_pub_des_state mobot_pub_des_state 

rosrun lidar_alarm lidar_alarm

This a resubmit attempt, so I’ll state the updated materials first. I’ve improved my Lidar_alarm and it works just fine. As you can see from the movie, the robot is braking when sees a obstacle and continues when the obstacle clears. The braking is not perfect, but it does has a graceful halt, and a graceful recovery. The following contents are the previously written lab report. 

Introduction:


  The goal of this assignment is to use the lidar alarm as a action client who connects to the mobot_pub_des_state server and give it command of trajectories set in the traj.builder.cpp.


Procedure:


  First, I completed the vacant part of the braking_trajectory in the traj_builder file, so that when e-stop is triggered, the robot will move backwards and slowly halt. Then I subscribed the e-stop and clear_e_stop services in the lidar_alarm by making the lidar_alarm into a action_client for the pub_des_state server.(from line 77 to line 110) I created a if statement inside the main of lidar_alarm so that once the lidar alarm is sounded(line 103 to 110), the robot will come to a halt, whereas when the lidar alarm is cleared, the robot will continue with its rest poses(which are three poses in this case). As you can see from the video, the e_stop was triggered when the robot was near the wall(when the lidar_alarm is sounded of course), and it goes to the braking_trajectory which is backing until it gradually halts. And once the lidar_alarm is cleared, the robot triggers the clear_e_stop service and goes on with its rest poses.

