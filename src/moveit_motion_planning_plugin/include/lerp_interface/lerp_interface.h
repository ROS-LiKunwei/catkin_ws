#pragma once
/*
    自定义路径规划算法的声明类，方便其它代码调用
*/
#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>


namespace lerp_interface
{
    // 这个宏定义创建了一个 LERPInterface 类型的前向声明。通过这种方式，我们可以在代码中使用 LERPInterface 类型的指针或引用，而不需要在当前代码文件中定义 LERPInterface 类型，
    // 从而加速编译速度和减少代码复杂度。
    // 在其他代码文件中，可以使用 LERPInterface 类型的指针或引用，而不需要包含 foo.h 头文件
    MOVEIT_CLASS_FORWARD(LERPInterface);

class LERPInterface
{
public:
    LERPInterface(const ros::NodeHandle& nh= ros::NodeHandle("~"));

    bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                           const planning_interface::MotionPlanRequest& req,
                           moveit_msgs::MotionPlanDetailedResponse& res);

protected:
    ros::NodeHandle nh_;
    std::string name_;
    int num_steps_;
    int dof_;

private:
    void interpolate(const std::vector<std::string> joint_names, robot_state::RobotStatePtr& robot_state,
                                      const robot_state::JointModelGroup* joint_model_group, const std::vector<double>& start_joint_vals,
                                      const std::vector<double>& goal_joint_vals, trajectory_msgs::JointTrajectory& joint_trajectory);
};
}

