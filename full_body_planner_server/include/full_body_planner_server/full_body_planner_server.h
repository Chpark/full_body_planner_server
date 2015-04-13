#ifndef FULL_BODY_PLANNER_SERVER_H
#define FULL_BODY_PLANNER_SERVER_H

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <pluginlib/class_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <map>
#include <vector>
#include <full_body_planner_server/waypoint_2d.h>
#include <menge_3d_interface/3DInterfaceServer.h>

namespace full_body_planner
{

class FullBodyPlannerServer
{
public:
    FullBodyPlannerServer(const ros::NodeHandle& node_handle);
    virtual ~FullBodyPlannerServer();

    void init();
    void terminate();

    bool getInput(std::vector<Trajectory2D>& input);
    bool compute3DTrajectory(std::vector<Trajectory2D>& trajectories, int index);
    void sendResponse(const std::vector<Trajectory2D>& trajectory2d, bool success);

    void decreaseTrajectoryCount();

protected:
    void initServer();
    void loadStaticScene();
    void loadPlanner();

    float** getResponseMemory(int num_agents, int agent_size);
    void deallocateResponseMemory();

    void setPlanningRequest(planning_interface::MotionPlanRequest& req, const std::vector<Trajectory2D>& trajectories, int index);
    bool plan(planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res);
    void updateTrajectory2DFromPlanningResponse(Trajectory2D& trajectory2d, const planning_interface::MotionPlanResponse& res);

    void displayTrajectory(int index, const planning_interface::MotionPlanResponse& res);
    void renderState(const robot_state::RobotState& state, const std::string& topic, const std_msgs::ColorRGBA& color);
    void drawPath(const Eigen::Vector3d& from, const Eigen::Vector3d& to, int id);
    void drawPosition(const Eigen::Vector3d& position, int id, const std_msgs::ColorRGBA& color);

    void setStandingState(robot_state::RobotState& state, Waypoint2D waypoint_2d);

    ros::NodeHandle node_handle_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    robot_model::RobotModelPtr robot_model_;
    planning_scene::PlanningScenePtr planning_scene_;
    boost::shared_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader_;
    planning_interface::PlannerManagerPtr planner_instance_;

    std_msgs::ColorRGBA color_start_, color_goal_;
    ros::Publisher planning_scene_diff_publisher_;
    std::vector<ros::Publisher> display_trajectory_publishers_;
    ros::Publisher vis_marker_array_publisher_;
    std::map<std::string, ros::Publisher> state_display_publishers_;
    std::vector<moveit_msgs::DisplayTrajectory> display_trajectories_;

    Interface3D::Interface3DServer server_;

    std::string group_name_;
    int trajectory_count_;
    float** response_data_;
    int response_data_num_agents_;
    int response_data_agent_size_;
};

inline void FullBodyPlannerServer::decreaseTrajectoryCount()
{
    --trajectory_count_;
}

}

#endif
