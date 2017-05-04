// acquire_block_client: 
// wsn, October, 2016

#include <ros/ros.h>
#include <mobot_pub_des_state/path.h>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <coordinator/ManipTaskAction.h>
#include <object_manipulation_properties/object_ID_codes.h>
//#include <object_manipulation_properties/object_manipulation_properties.h>
#include <object_finder/objectFinderAction.h>
#include <object_grabber/object_grabberAction.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

bool g_goal_done = true;
int g_callback_status = coordinator::ManipTaskResult::PENDING;
int g_object_grabber_return_code=0;
int g_object_finder_return_code=0;
int g_fdbk_count = 0;

geometry_msgs::PoseStamped g_des_flange_pose_stamped_wrt_torso;
geometry_msgs::PoseStamped g_object_pose;
coordinator::ManipTaskResult g_result;
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}


void doneCb(const actionlib::SimpleClientGoalState& state,
        const coordinator::ManipTaskResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    g_goal_done = true;
    g_result = *result;
    g_callback_status = result->manip_return_code;

    switch (g_callback_status) {
        case coordinator::ManipTaskResult::MANIP_SUCCESS:
            ROS_INFO("returned MANIP_SUCCESS");
            
            break;
            
        case coordinator::ManipTaskResult::FAILED_PERCEPTION:
            ROS_WARN("returned FAILED_PERCEPTION");
            g_object_finder_return_code = result->object_finder_return_code;
            break;
        case coordinator::ManipTaskResult::FAILED_PICKUP:
            ROS_WARN("returned FAILED_PICKUP");
            g_object_grabber_return_code= result->object_grabber_return_code;
            g_object_pose = result->object_pose;
            //g_des_flange_pose_stamped_wrt_torso = result->des_flange_pose_stamped_wrt_torso;
            break;
        case coordinator::ManipTaskResult::FAILED_DROPOFF:
            ROS_WARN("returned FAILED_DROPOFF");
            //g_des_flange_pose_stamped_wrt_torso = result->des_flange_pose_stamped_wrt_torso;          
            break;
    }
}

//optional feedback; output has been suppressed (commented out) below
void feedbackCb(const coordinator::ManipTaskFeedbackConstPtr& fdbk_msg) {
    g_fdbk_count++;
    if (g_fdbk_count > 1000) { //slow down the feedback publications
        g_fdbk_count = 0;
        //suppress this feedback output
        //ROS_INFO("feedback status = %d", fdbk_msg->feedback_status);
    }
    //g_fdbk = fdbk_msg->feedback_status; //make status available to "main()"
}

// Called once when the goal becomes active; not necessary, but possibly useful for diagnostics
void activeCb() {
    ROS_INFO("Goal just went active");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_client_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle
    ros::Publisher gripper = nh.advertise<std_msgs::Bool>("close_gripper", 1); //For gripper 
    std_msgs::Bool grab;
    geometry_msgs::Quaternion quat;
    grab.data = false;

    coordinator::ManipTaskGoal goal;

    actionlib::SimpleActionClient<coordinator::ManipTaskAction> action_client("manip_task_action_service", true);

    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = action_client.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }

// Uncomment below when ready to move
/*
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    geometry_msgs::Pose pose;
    pose.position.x = 0.0; // say desired x-coord is 5
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    quat = convertPlanarPhi2Quaternion(0);
    pose.orientation = quat;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);
    ROS_INFO("connected to action server"); // if here, then we connected to the server;

    pose.position.x = 3.8;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);
    ROS_INFO("connected to action server"); // if here, then we connected to the server;

    pose.position.y = -1.8;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);

    ros::Duration(40.0).sleep();
*/

    ros::Duration(60.0).sleep();

    ROS_INFO("sending a goal: move to pre-pose");
    g_goal_done = false;
    goal.action_code = coordinator::ManipTaskGoal::MOVE_TO_PRE_POSE;
    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    while (!g_goal_done) {
        ros::Duration(0.1).sleep();
    }
    if (g_callback_status!= coordinator::ManipTaskResult::MANIP_SUCCESS)
    {
        ROS_ERROR("failed to move quitting");
        return 0;
    }
    //send vision request to find table top:
    ROS_INFO("sending a goal: seeking table top");
    g_goal_done = false;
    goal.action_code = coordinator::ManipTaskGoal::FIND_TABLE_SURFACE;

    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    while (!g_goal_done) {
        ros::Duration(0.1).sleep();
    }
    
    //send vision goal to find block:
    ROS_INFO("sending a goal: find block");
    g_goal_done = false;
    goal.action_code = coordinator::ManipTaskGoal::GET_PICKUP_POSE;
    goal.object_code= ObjectIdCodes::TOY_BLOCK_ID;
    goal.perception_source = coordinator::ManipTaskGoal::PCL_VISION;
    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    while (!g_goal_done) {
        ros::Duration(0.1).sleep();
    }    
    if (g_callback_status!= coordinator::ManipTaskResult::MANIP_SUCCESS)
    {
        ROS_ERROR("failed to find block quitting");
        return 0;
    }
    g_object_pose = g_result.object_pose;
    ROS_INFO_STREAM("object pose w/rt frame-id "<<g_object_pose.header.frame_id<<endl);
    ROS_INFO_STREAM("object origin: (x,y,z) = ("<<g_object_pose.pose.position.x<<", "<<g_object_pose.pose.position.y<<", "
              <<g_object_pose.pose.position.z<<")"<<endl);
    ROS_INFO_STREAM("orientation: (qx,qy,qz,qw) = ("<<g_object_pose.pose.orientation.x<<","
              <<g_object_pose.pose.orientation.y<<","
              <<g_object_pose.pose.orientation.z<<","
              <<g_object_pose.pose.orientation.w<<")"<<endl); 
    
    
    //send command to acquire block:
    ROS_INFO("sending a goal: grab block");
    g_goal_done = false;
    goal.action_code = coordinator::ManipTaskGoal::GRAB_OBJECT;
    goal.pickup_frame = g_result.object_pose;
    goal.object_code= ObjectIdCodes::TOY_BLOCK_ID;
    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    while (!g_goal_done) {
        ros::Duration(0.1).sleep();
    }    
        if (g_callback_status!= coordinator::ManipTaskResult::MANIP_SUCCESS)
    {
        ROS_ERROR("failed to grab block; quitting");
        return 0;
    }


// Modified by gamma
    ros::Duration(3.0).sleep();
    grab.data = true;
    ros::Time start =  ros::Time::now();   
    while((ros::Time::now() - start) < ros::Duration(3.0)) {
    	gripper.publish(grab);
    	ros::spinOnce();
    }

    ros::Duration(3.0).sleep();
// End of modification
    
    ROS_INFO("sending a goal: move to pre-pose");
    g_goal_done = false;
    goal.action_code = coordinator::ManipTaskGoal::MOVE_TO_PRE_POSE;
    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    while (!g_goal_done) {
        ros::Duration(0.1).sleep();
    }
        if (g_callback_status!= coordinator::ManipTaskResult::MANIP_SUCCESS)
    {
        ROS_ERROR("failed to move to pre-pose; quitting");
        return 0;
    }

// Uncomment below when ready to move
/*
    pose.position.y = 0;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);
    
    pose.position.x = 0;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);

    client.call(path_srv);
*/
    
    return 0;
}

