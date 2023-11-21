#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joy.hpp>

enum Button
 {
   A = 0,
   B = 1,
   X = 2,
   Y = 3,
   LEFT_BUMPER = 4,
   RIGHT_BUMPER = 5,
   CHANGE_VIEW = 6,
   MENU = 7,
   HOME = 8,
   LEFT_STICK_CLICK = 9,
   RIGHT_STICK_CLICK = 10
 };

int_fast8_t c,input;



std::map<std::string, double> joints1, joints2, joints3, joints4, joints5;


void topic_callback(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
{
   if(msg->buttons[LEFT_STICK_CLICK]){c=0;}
   if(msg->buttons[RIGHT_STICK_CLICK]){c=1;}
   if(msg->buttons[HOME]){c=2;}
   if(msg->buttons[MENU] || msg->buttons[LEFT_BUMPER]){c=3;}

    
}

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("hello_moveit", node_options);
  auto Sub = move_group_node -> create_subscription<sensor_msgs::msg::Joy>("/joy",10,topic_callback);


  

   rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();
  
  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");
  //move_group_node -> declare_parameter("scale",0.05);
  //auto scale = move_group_node -> get_parameter("scale").as_double();
  double scale = 0.05;
  
  for(int i = 1; i < argc ;i++)
  {
  	char* option = argv[i];
  	if (strcmp(option,"-s") ==0)
  	{ 
  	  if (i+1 < argc){
  	  scale = std::stod(argv[i+1]);}
  	}
  }
  
  std::cout<< "scale : "<< scale <<std::endl;

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(move_group_node, "ARM");

  move_group_interface.setMaxVelocityScalingFactor(scale);
  move_group_interface.setMaxAccelerationScalingFactor(scale);

  
  
  joints1["waist_joint"] = 0 * (M_PI / 180);
  joints1["shoulder_joint"] = 0 * (M_PI / 180);
  joints1["elbow_joint"] = 90 * (M_PI / 180);
  joints1["hand_joint"] = 90 * (M_PI / 180);
  joints1["wrist_joint"] = 0 * (M_PI / 180);

  joints2["waist_joint"] = 0 * (M_PI / 180);
  joints2["shoulder_joint"] = -64 * (M_PI / 180);
  joints2["elbow_joint"] = 72 * (M_PI / 180);
  joints2["hand_joint"] = 43 * (M_PI / 180);
  joints2["wrist_joint"] = 0 * (M_PI / 180);

  joints3["waist_joint"] = 60 * (M_PI / 180);
  joints3["shoulder_joint"] = 0 * (M_PI / 180);
  joints3["elbow_joint"] = 90 * (M_PI / 180);
  joints3["hand_joint"] = 90 * (M_PI / 180);
  joints3["wrist_joint"] = 0 * (M_PI / 180);

  joints4["waist_joint"] = 60 * (M_PI / 180);
  joints4["shoulder_joint"] = -64 * (M_PI / 180);
  joints4["elbow_joint"] = 72 * (M_PI / 180);
  joints4["hand_joint"] = 43 * (M_PI / 180);
  joints4["wrist_joint"] = 0 * (M_PI / 180);

  joints5["waist_joint"] = 0 * (M_PI / 180);
  joints5["shoulder_joint"] = 0 * (M_PI / 180);
  joints5["elbow_joint"] = 0 * (M_PI / 180);
  joints5["hand_joint"] = 0 * (M_PI / 180);
  joints5["wrist_joint"] = 0 * (M_PI / 180);
    
  move_group_interface.setStartStateToCurrentState();

  rclcpp::WallRate loop_rate(10);

  input = 0;

  auto end_link = move_group_interface.getEndEffectorLink();
  std::cout<< "end_link : " << end_link <<std::endl;

  while (rclcpp::ok())
  {
        
        if (input != c)
        {
            input = c;
            

            if (c == 0)
            {

                //current_state->copyJointGroupPositions(joint_model_group, joints1);

                move_group_interface.setJointValueTarget(joints1);
               
                printf("pub_time: %f\n",rclcpp::Clock{RCL_ROS_TIME}.now().seconds());
                move_group_interface.asyncMove();
                
            }
            else if (c == 1)
            {
                //current_state->copyJointGroupPositions(joint_model_group, joints2);

                move_group_interface.setJointValueTarget(joints2);
                printf("pub_time: %f\n",rclcpp::Clock{RCL_ROS_TIME}.now().seconds());
                move_group_interface.asyncMove();
                
            }
            /*else if (c == 'c')
            {
                //current_state->copyJointGroupPositions(joint_model_group, joints3);
                move_group_interface.setJointValueTarget(joints3);
               printf("pub_time: %f\n",rclcpp::Clock{RCL_ROS_TIME}.now().seconds());
                move_group_interface.asyncMove();
            }

            else if (c == 'd')
            {
                //current_state->copyJointGroupPositions(joint_model_group, joints4);
                move_group_interface.setJointValueTarget(joints4);
                printf("pub_time: %f\n",rclcpp::Clock{RCL_ROS_TIME}.now().seconds());
                move_group_interface.asyncMove();
            }*/

            else if (c == 2)
            {
                //current_state->copyJointGroupPositions(joint_model_group, joints5);

                move_group_interface.setJointValueTarget(joints5);
                printf("pub_time: %f\n",rclcpp::Clock{RCL_ROS_TIME}.now().seconds());
                move_group_interface.asyncMove();
                
            }
            


        }
        loop_rate.sleep();

    }
  
  rclcpp::shutdown();
  return 0;
}

