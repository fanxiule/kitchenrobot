#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_objects");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();   
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity(); 
    visual_tools.trigger();//
     
    double table_length = 62*25.4/1000; //62"
    double table_width = 36*25.4/1000; //36"
    double table_height = 35.625*25.4/1000; //35-5/8"
    double table1_xoffset = 0.3; //from robot
    double table1_yoffset = 0; //from robot
    double table1_zoffset = -0.3; //from robot
    double pastadis_base_length = 0.4;
    double pastadis_base_width = 0.08;
    double pastadis_base_height = 0.15;
    double pastadis_top_length = 0.6;
    double pastadis_top_width = 0.15;
    double pastadis_top_height = 0.3;
    double pastadis_xoffset = 0.5; //from table2
    double pastadis_yoffset = 0.2; //from table2
    double pastadistop_xoffset = 0; //from base
    double pastadistop_yoffset = 0.05; //from base
    double saucedis_height = 0.5;
    double saucedis_radius = 0.08;
    double saucedis_xoffset = 0.2;
    double saucedis_yoffset = -0.3;
    double fryer_length = 0.545;
    double fryer_width = 0.355;
    double fryer_height= 0.365;
    double fryer_xoffset = -0.4;
    double fryer_yoffset = 0.3;

    moveit_msgs::CollisionObject table1; //table 1
    table1.header.frame_id = move_group.getPlanningFrame();

    table1.id = "box1";
    shape_msgs::SolidPrimitive primitive_table1;
    primitive_table1.type = primitive_table1.BOX;
    primitive_table1.dimensions.resize(3);
    primitive_table1.dimensions[0] = table_width;
    primitive_table1.dimensions[1] = table_length;
    primitive_table1.dimensions[2] = table_height;
	
    geometry_msgs::Pose table1_pose;
    table1_pose.orientation.x = 0;
    table1_pose.orientation.y = 0;
    table1_pose.orientation.z = 0;
    table1_pose.orientation.w = 1;
    table1_pose.position.x = table_width/2+table1_xoffset;
    table1_pose.position.y = table1_yoffset;
    table1_pose.position.z = table1_zoffset;

    table1.primitives.push_back(primitive_table1);
    table1.primitive_poses.push_back(table1_pose);
    table1.operation = table1.ADD;
    
    std::vector<moveit_msgs::CollisionObject> collision_objects1;
    collision_objects1.push_back(table1);
    ros::Duration(0.1).sleep();
    planning_scene_interface.addCollisionObjects(collision_objects1);

    moveit_msgs::CollisionObject table2; //table2
    table2.header.frame_id = move_group.getPlanningFrame();

    table2.id = "box2";
    shape_msgs::SolidPrimitive primitive_table2;
    primitive_table2.type = primitive_table2.BOX;
    primitive_table2.dimensions.resize(3);
    primitive_table2.dimensions[0] = table_width;
    primitive_table2.dimensions[1] = table_length;
    primitive_table2.dimensions[2] = table_height;
	
    geometry_msgs::Pose table2_pose;
    table2_pose.orientation.x = 0;
    table2_pose.orientation.y = 0;
    table2_pose.orientation.z = 0.7071068;
    table2_pose.orientation.w = 0.7071068;
    table2_pose.position.x = table1_xoffset+table_width-table_length/2;
    table2_pose.position.y = table1_yoffset-(table_length/2+table_width/2);
    table2_pose.position.z = table1_zoffset;

    table2.primitives.push_back(primitive_table2);
    table2.primitive_poses.push_back(table2_pose);
    table2.operation = table2.ADD;
    
    std::vector<moveit_msgs::CollisionObject> collision_objects2;
    collision_objects2.push_back(table2);
    ros::Duration(0.1).sleep();
    planning_scene_interface.addCollisionObjects(collision_objects2);

    moveit_msgs::CollisionObject pastadis_base; //pasta dispenser base
    pastadis_base.header.frame_id = move_group.getPlanningFrame();

    pastadis_base.id = "box3";
    shape_msgs::SolidPrimitive primitive_pastadis_base;
    primitive_pastadis_base.type = primitive_pastadis_base.BOX;
    primitive_pastadis_base.dimensions.resize(3);
    primitive_pastadis_base.dimensions[0] = pastadis_base_width;
    primitive_pastadis_base.dimensions[1] = pastadis_base_length;
    primitive_pastadis_base.dimensions[2] = pastadis_base_height;
	
    geometry_msgs::Pose pastadis_base_pose;
    pastadis_base_pose.orientation.x = 0;
    pastadis_base_pose.orientation.y = 0;
    pastadis_base_pose.orientation.z = 0.7071068;
    pastadis_base_pose.orientation.w = 0.7071068;
    pastadis_base_pose.position.x = table1_xoffset+table_width-table_length+pastadis_xoffset;
    pastadis_base_pose.position.y = table1_yoffset-(table_length/2+table_width)+pastadis_yoffset;
    pastadis_base_pose.position.z = table1_zoffset+table_height/2+pastadis_base_height/2;

    pastadis_base.primitives.push_back(primitive_pastadis_base);
    pastadis_base.primitive_poses.push_back(pastadis_base_pose);
    pastadis_base.operation = pastadis_base.ADD;
    
    std::vector<moveit_msgs::CollisionObject> collision_objects3;
    collision_objects3.push_back(pastadis_base);
    ros::Duration(0.1).sleep();
    planning_scene_interface.addCollisionObjects(collision_objects3);

    moveit_msgs::CollisionObject pastadis_top; //pasta dispenser top
    pastadis_top.header.frame_id = move_group.getPlanningFrame();

    pastadis_top.id = "box4";
    shape_msgs::SolidPrimitive primitive_pastadis_top;
    primitive_pastadis_top.type = primitive_pastadis_top.BOX;
    primitive_pastadis_top.dimensions.resize(3);
    primitive_pastadis_top.dimensions[0] = pastadis_top_width;
    primitive_pastadis_top.dimensions[1] = pastadis_top_length;
    primitive_pastadis_top.dimensions[2] = pastadis_top_height;
	
    geometry_msgs::Pose pastadis_top_pose;
    pastadis_top_pose.orientation.x = 0;
    pastadis_top_pose.orientation.y = 0;
    pastadis_top_pose.orientation.z = 0.7071068;
    pastadis_top_pose.orientation.w = 0.7071068;
    pastadis_top_pose.position.x = table1_xoffset+table_width-table_length+pastadis_xoffset;
    pastadis_top_pose.position.y = table1_yoffset-(table_length/2+table_width)+pastadis_yoffset+(pastadis_top_width/2-pastadis_base_width/2);
    pastadis_top_pose.position.z = table1_zoffset+table_height/2+pastadis_base_height+pastadis_top_height/2;

    pastadis_top.primitives.push_back(primitive_pastadis_top);
    pastadis_top.primitive_poses.push_back(pastadis_top_pose);
    pastadis_top.operation = pastadis_top.ADD;
    
    std::vector<moveit_msgs::CollisionObject> collision_objects4;
    collision_objects4.push_back(pastadis_top);
    ros::Duration(0.1).sleep();
    planning_scene_interface.addCollisionObjects(collision_objects4);

    moveit_msgs::CollisionObject sauce_dis; //sauce dispenser
    sauce_dis.header.frame_id = move_group.getPlanningFrame();

    sauce_dis.id = "cylinder1";
    shape_msgs::SolidPrimitive primitive_sauce_dis;
    primitive_sauce_dis.type = primitive_sauce_dis.CYLINDER;
    primitive_sauce_dis.dimensions.resize(2);
    primitive_sauce_dis.dimensions[0] = saucedis_height;
    primitive_sauce_dis.dimensions[1] = saucedis_radius;
	
    geometry_msgs::Pose sauce_dis_pose;
    sauce_dis_pose.orientation.x = 0;
    sauce_dis_pose.orientation.y = 0;
    sauce_dis_pose.orientation.z = 0;
    sauce_dis_pose.orientation.w = 1;
    sauce_dis_pose.position.x = table1_xoffset+saucedis_xoffset;
    sauce_dis_pose.position.y = table1_yoffset+table_length/2+saucedis_yoffset;
    sauce_dis_pose.position.z = table1_zoffset+table_height/2+saucedis_height/2;

    sauce_dis.primitives.push_back(primitive_sauce_dis);
    sauce_dis.primitive_poses.push_back(sauce_dis_pose);
    sauce_dis.operation = sauce_dis.ADD;
    
    std::vector<moveit_msgs::CollisionObject> collision_objects5;
    collision_objects5.push_back(sauce_dis);
    ros::Duration(0.1).sleep();
    planning_scene_interface.addCollisionObjects(collision_objects5);

    moveit_msgs::CollisionObject fryer; //fryer
    fryer.header.frame_id = move_group.getPlanningFrame();

    fryer.id = "box5";
    shape_msgs::SolidPrimitive primitive_fryer;
    primitive_fryer.type = primitive_fryer.BOX;
    primitive_fryer.dimensions.resize(3);
    primitive_fryer.dimensions[0] = fryer_length;
    primitive_fryer.dimensions[1] = fryer_width;
    primitive_fryer.dimensions[2] = fryer_height;
	
    geometry_msgs::Pose fryer_pose;
    fryer_pose.orientation.x = 0;
    fryer_pose.orientation.y = 0;
    fryer_pose.orientation.z = 0.3826834;
    fryer_pose.orientation.w = 0.9238795;
    fryer_pose.position.x = table1_xoffset+table_width+fryer_xoffset;
    fryer_pose.position.y = table1_yoffset-table_length/2-table_width+fryer_yoffset;
    fryer_pose.position.z = table1_zoffset+table_height/2+fryer_height/2;

    fryer.primitives.push_back(primitive_fryer);
    fryer.primitive_poses.push_back(fryer_pose);
    fryer.operation = fryer.ADD;
    
    std::vector<moveit_msgs::CollisionObject> collision_objects6;
    collision_objects6.push_back(fryer);
    ros::Duration(0.1).sleep();
    planning_scene_interface.addCollisionObjects(collision_objects6);

    visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");
 
    ros::shutdown();
    return 0;
}
