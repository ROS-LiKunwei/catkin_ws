#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf2/LinearMath/Quaternion.h>

#include <vector>
#include <string>
#include <iostream>
#include <moveit_msgs/ObjectColor.h>
using namespace std;

class Gazebo_Process
{
public:
    Gazebo_Process(const ros::NodeHandle &nh, moveit::planning_interface::MoveGroupInterface &arm, const string &PLANNING_GROUP)
    {
        this->arm_ = &arm;
        this->nh_ = nh;

        arm_->setGoalPositionTolerance(0.1);
        arm_->setGoalOrientationTolerance(0.1);
        arm_->setGoalJointTolerance(0.1);

        arm_->setMaxAccelerationScalingFactor(0.5);
        arm_->setMaxVelocityScalingFactor(0.5);

        const moveit::core::JointModelGroup* joint_model_group = arm_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        this->end_effector_ = arm_->getEndEffectorLink();

        this->reference_frame_ = "base_link";
        arm_->setPoseReferenceFrame(reference_frame_);

        arm_->allowReplanning(true);
        arm_->setPlanningTime(5.0);
        arm_->setPlannerId("RRTConnect");

        go_home();

        create_scene();
    }

    void go_home()
    {
        arm_->setNamedTarget("init");
        arm_->move();
        sleep(1);
    }

    void create_scene()
    {
        ros::Publisher planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene",1);
        ros::WallDuration sleep_t(0.5);
        while(planning_scene_diff_publisher.getNumSubscribers() < 1)
        {
            sleep_t.sleep();
        }

        // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        moveit_msgs::PlanningScene planning_scene;
        moveit_msgs::CollisionObject collision_object1,collision_object2,collision_object3;
        
        collision_object1.header.frame_id = arm_->getPlanningFrame();
        //  id & size & type
        collision_object1.id = "table_surface";
        shape_msgs::SolidPrimitive primitive1;
        primitive1.type = primitive1.BOX;
        primitive1.dimensions.resize(3);
        primitive1.dimensions[primitive1.BOX_X] = 0.15;
        primitive1.dimensions[primitive1.BOX_Y ]= 0.8;
        primitive1.dimensions[primitive1.BOX_Z] = 0.01;
        // pose
        geometry_msgs::Pose table_surface_pose;
        table_surface_pose.orientation.w = 1.0;
        table_surface_pose.position.x = -0.25;
        table_surface_pose.position.y = 0.0;
        table_surface_pose.position.z = 0.05;

        collision_object1.primitives.push_back(primitive1);
        collision_object1.primitive_poses.push_back(table_surface_pose);
        collision_object1.operation = collision_object1.ADD;

        planning_scene.world.collision_objects.push_back(collision_object1);
        planning_scene.is_diff = true;
        planning_scene_diff_publisher.publish(planning_scene);

        ROS_INFO("Added an table into the world~");
       sleep(1);

        //collision2
        collision_object2.header.frame_id = arm_->getPlanningFrame();
        collision_object2.id = "table_baffle";
        shape_msgs::SolidPrimitive primitive2;
        primitive2.type = primitive2.BOX;
        primitive2.dimensions.resize(3);
        primitive2.dimensions[primitive2.BOX_X] = 0.15;
        primitive2.dimensions[primitive2.BOX_Y ]= 0.05;
        primitive2.dimensions[primitive2.BOX_Z] = 0.08;
        // pose
        geometry_msgs::Pose table_baffle_pose;
        table_baffle_pose.orientation.w = 1.0;
        table_baffle_pose.position.x = -0.25;
        table_baffle_pose.position.y = 0.0;
        table_baffle_pose.position.z = 0.05+  primitive2.dimensions[primitive2.BOX_Z]/2;

        // color
        moveit_msgs::ObjectColor object_color;
        object_color.color.r = 0;
        object_color.color.g = 0;
        object_color.color.b = 1;
        object_color.color.a = 1;

        collision_object2.primitives.push_back(primitive2);
        collision_object2.primitive_poses.push_back(table_baffle_pose);
        collision_object2.operation = collision_object2.ADD;

        planning_scene.world.collision_objects.push_back(collision_object2);
        planning_scene.is_diff = true;
        planning_scene.object_colors.push_back(object_color);
        planning_scene_diff_publisher.publish(planning_scene);
        ROS_INFO("Added an baffle into the world~");
        sleep(1);

    }

    void move_p_constraints(const vector<vector<double>>& poses)
    {
        vector<geometry_msgs::Pose> waypoints;
        for(int i =0;i<poses.size();++i)
        {
            geometry_msgs::Pose target_pose;
            target_pose.position.x = poses[i][0];
            target_pose.position.y = poses[i][1];
            target_pose.position.z = poses[i][2];

            tf2::Quaternion myQuaternion;
            myQuaternion.setRPY(poses[i][3],poses[i][4],poses[i][5]);
            target_pose.orientation.x = 0;//myQuaternion.getX();
            target_pose.orientation.y = 0;
            target_pose.orientation.z = 0;
            target_pose.orientation.w = 1.0;

            waypoints.push_back(target_pose);
        }

        // set constraint
        moveit_msgs::OrientationConstraint ocm;
        ocm.link_name = "link5";
        ocm.header.frame_id="base_link";
        ocm.orientation.x = 0;
        ocm.orientation.y = 0;
        ocm.orientation.z = 0;
        ocm.orientation.w = 1;
        ocm.absolute_x_axis_tolerance = 0.1;
        ocm.absolute_y_axis_tolerance = 0.1;
        ocm.absolute_z_axis_tolerance = 0.1;
        ocm.weight = 1.0;
        moveit_msgs::Constraints test_constraints;
        test_constraints.orientation_constraints.push_back(ocm);
        // arm_->setPathConstraints(test_constraints);
    
        // Now we will plan to the earlier pose target from the new start state that we have just created.
        for(int i=0;i<waypoints.size();++i)
        {
            arm_->setStartStateToCurrentState();
            arm_->setPoseTarget(waypoints[i]);

            arm_->setPlanningTime(30.0);
            
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            moveit::planning_interface::MoveItErrorCode success;
            for(int j =0;i<10;++i)
            {
                 success = arm_->plan(plan);
                 if(success)
                 {
                    arm_->execute(plan);
                    ROS_INFO("After %d 'th try.Success to move to No.%d position",j+1,i+1);
                    sleep(1);
                    break;
                 }
            }


            ROS_INFO("move_p_with_constraints :%s",success ? "Success":"Failed");

            arm_->setPlanningTime(5.0);
            arm_->clearPathConstraints();
            if(success)
            {
                // arm_->execute(plan);
                // ROS_INFO("Success to move to No.%d position",i);
                // sleep(1);
                continue;
            }
            else
            {
                ROS_ERROR("Fail to move to No.%d position",i);
            }
        }
    }

    ~Gazebo_Process()
    {
        ros::shutdown();
    }
public:
    string reference_frame_;
    string end_effector_;
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface *arm_;
};

void simulation_move_only(ros::NodeHandle& nh)
{
    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);

    Gazebo_Process GP(nh,arm,PLANNING_GROUP);

    vector<vector<double>> xyzrpys;
    vector<double> position1={-0.1,-0.1,0.15,0,0,0};
    vector<double> position2={-0.1,0.1,0.15,0,0,0};
    xyzrpys.emplace_back(position1);
    xyzrpys.emplace_back(position2);
    GP.move_p_constraints(xyzrpys);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"gazebo_grasp_simulation");
    ros::AsyncSpinner spinner(1);
    ros::NodeHandle nh;
    spinner.start();
    // static const std::string PLANNING_GROUP = "arm";
    // moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);

    // Gazebo_Process(nh,arm,PLANNING_GROUP);
    simulation_move_only(nh);
    return 0;
}