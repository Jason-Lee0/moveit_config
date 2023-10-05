#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <adlink_msg/keyboard.h>


char c;


void chatterCallback(const adlink_msg::keyboard command)
{

        c = command.keyboard_command;

    
}



int main(int argc, char** argv)
{

    ros::init(argc, argv, "Command_Execute");
    ros::NodeHandle node_handle;

    ros::Subscriber sub = node_handle.subscribe("Keyboard_Input", 10, chatterCallback);
    ros::AsyncSpinner spinner(1);
    spinner.start();


    static const std::string PLANNING_GROUP = "ARM";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

 

    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


    //if no set velocity or acceleration that defult is 0.1 
    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    const std::string end_effector_link = move_group.getEndEffectorLink();

    geometry_msgs::Pose p1 = move_group.getCurrentPose(end_effector_link).pose;

    std::map<std::string, double> joints1, joints2, joints3, joints4, joints5;


   

        joints1["waist_joint"] = 0 * (M_PI / 180);
        joints1["shoulder_joint"] = 0 * (M_PI / 180);
        joints1["elbow_joint"] = 90 * (M_PI / 180);
        joints1["hand_joint"] = 90 * (M_PI / 180);
        joints1["wrist_joint"] = 0 * (M_PI / 180);

        joints2["waist_joint"] = 0 * (M_PI / 180);
        joints2["shoulder_joint"] = 64 * (M_PI / 180);
        joints2["elbow_joint"] = 72 * (M_PI / 180);
        joints2["hand_joint"] = 43 * (M_PI / 180);
        joints2["wrist_joint"] = 0 * (M_PI / 180);

        joints3["waist_joint"] = 60 * (M_PI / 180);
        joints3["shoulder_joint"] = 0 * (M_PI / 180);
        joints3["elbow_joint"] = 90 * (M_PI / 180);
        joints3["hand_joint"] = 90 * (M_PI / 180);
        joints3["wrist_joint"] = 0 * (M_PI / 180);

        joints4["waist_joint"] = 60 * (M_PI / 180);
        joints4["shoulder_joint"] = 64 * (M_PI / 180);
        joints4["elbow_joint"] = 72 * (M_PI / 180);
        joints4["hand_joint"] = 43 * (M_PI / 180);
        joints4["wrist_joint"] = 0 * (M_PI / 180);

        joints5["waist_joint"] = 0 * (M_PI / 180);
        joints5["shoulder_joint"] = 0 * (M_PI / 180);
        joints5["elbow_joint"] = 0 * (M_PI / 180);
        joints5["hand_joint"] = 0 * (M_PI / 180);
        joints5["wrist_joint"] = 0 * (M_PI / 180);



        
    move_group.setStartStateToCurrentState();


    ros::Rate rate(10);

    char input = 0;
    while (ros::ok())
    {
        
        if (input != c)
        {
            input = c;

            if (c == 'a')
            {

                //current_state->copyJointGroupPositions(joint_model_group, joints1);
                move_group.setJointValueTarget(joints1);
               
                printf("pub_time: %f\n",ros::Time::now().toSec());
                move_group.asyncMove();
            }
            else if (c == 'b')
            {
                //current_state->copyJointGroupPositions(joint_model_group, joints2);
                move_group.setJointValueTarget(joints2);
                printf("pub_time: %f\n",ros::Time::now().toSec());
                move_group.asyncMove();
            }
            else if (c == 'c')
            {
                //current_state->copyJointGroupPositions(joint_model_group, joints3);
                move_group.setJointValueTarget(joints3);
               printf("pub_time: %f\n",ros::Time::now().toSec());
                move_group.asyncMove();
            }

            else if (c == 'd')
            {
                //current_state->copyJointGroupPositions(joint_model_group, joints4);
                move_group.setJointValueTarget(joints4);
                printf("pub_time: %f\n",ros::Time::now().toSec());
                move_group.asyncMove();
            }

            else if (c == 'e')
            {
                //current_state->copyJointGroupPositions(joint_model_group, joints5);
                move_group.setJointValueTarget(joints5);
                printf("pub_time: %f\n",ros::Time::now().toSec());
                move_group.asyncMove();
            }


        }

        rate.sleep();

    }
}
