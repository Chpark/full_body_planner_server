// Original code from pr2_moveit_tutorials::motion_planning_api_tutorial.cpp
#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PositionConstraint.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit/robot_state/conversions.h>
#include <boost/variant/get.hpp>
#include <boost/lexical_cast.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <full_body_planner_server/full_body_planner_server.h>
#include <fstream>
#include <sched.h>
#include <limits>

using namespace std;

namespace full_body_planner
{

const int NUM_WAYPOINTS = 20;

ostream& operator<<(ostream& os, const Waypoint2D& waypoint)
{
    os << waypoint.agentID << " " << waypoint.time << " " << waypoint.x << " " << waypoint.y << " " << waypoint.state << " " << waypoint.vx << " " << waypoint.vy << endl;
    return os;
}

istream& operator>>(istream& is, Waypoint2D& waypoint)
{
    double state;
    is >> waypoint.agentID >> waypoint.time >> waypoint.x >> waypoint.y >> state >> waypoint.vx >> waypoint.vy;
    waypoint.state = state;
    return is;
}

FullBodyPlannerServer::FullBodyPlannerServer(const ros::NodeHandle& node_handle) :
    node_handle_(node_handle)
{
    group_name_ = "whole_body2";

    color_start_.a = 0.5;
    color_start_.r = 0.0;
    color_start_.g = 1.0;
    color_start_.b = 0.5;

    color_goal_.a = 0.5;
    color_goal_.r = 0.0;
    color_goal_.g = 0.5;
    color_goal_.b = 1.0;
}

FullBodyPlannerServer::~FullBodyPlannerServer()
{
}

void FullBodyPlannerServer::init()
{
    vis_marker_array_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100, true);

    // scene initialization
    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
    robot_model_ = robot_model_loader_->getModel();
    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
    planning_scene_diff_publisher_ = node_handle_.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    while (planning_scene_diff_publisher_.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
        ROS_INFO("Waiting planning_scene subscribers");
    }

    loadStaticScene();
    loadPlanner();

    ros::WallDuration sleep_time(0.01);
    sleep_time.sleep();
}

bool FullBodyPlannerServer::getInput(Trajectory2D& trajectory2d)
{
    static vector<Trajectory2D> readed_trajectories;
    static vector<int> readed_indices;
    static bool initialized = false;
    if (!initialized)
    {
        // read from file
        std::ifstream trajectory_file;
        std::string file_name = "input.txt";

        trajectory_file.open(file_name.c_str());
        if (trajectory_file.is_open())
        {
            Waypoint2D waypoint;
            while (trajectory_file >> waypoint)
            {
                // TODO: for debug
                if (waypoint.agentID == 1)
                    continue;

                waypoint.x *= 0.5;
                waypoint.y *= 0.5;
                waypoint.vx *= 0.5;
                waypoint.vy *= 0.5;

                double norm = std::sqrt(waypoint.vx * waypoint.vx + waypoint.vy * waypoint.vy);
                if (norm < 1e-7)
                    norm = 1e-7;
                waypoint.orientation = std::atan2(waypoint.vy / norm, waypoint.vx / norm) - M_PI_2;
                if (waypoint.orientation < -M_PI)
                    waypoint.orientation += 2.0 * M_PI;

                if (readed_trajectories.size() <= waypoint.agentID)
                {
                    readed_trajectories.resize(waypoint.agentID + 1);
                    readed_indices.resize(waypoint.agentID + 1, 0);
                }

                readed_trajectories[waypoint.agentID].push_back(waypoint);

                if (agent_trajectory_count_.size() <= waypoint.agentID)
                    agent_trajectory_count_.resize(waypoint.agentID + 1, 0);
            }
        }
        initialized = true;
    }
    static int current_agent = 0;

    if (readed_indices[current_agent] + 1 >= readed_trajectories[current_agent].size())
        return false;

    trajectory2d.resize(NUM_WAYPOINTS + 1);
    if (readed_indices[current_agent] + NUM_WAYPOINTS + 1 <= readed_trajectories[current_agent].size())
    {
        std::copy(readed_trajectories[current_agent].begin() + readed_indices[current_agent],
                  readed_trajectories[current_agent].begin() + readed_indices[current_agent] + NUM_WAYPOINTS + 1,
                  trajectory2d.begin());
    }
    else
    {
        std::copy(readed_trajectories[current_agent].begin() + readed_indices[current_agent],
                  readed_trajectories[current_agent].end(),
                  trajectory2d.begin());
        for (int i = readed_indices[current_agent] + NUM_WAYPOINTS + 1 - readed_trajectories[current_agent].size(); i < NUM_WAYPOINTS + 1; ++i)
            trajectory2d[i] = readed_trajectories[current_agent].back();
    }

    ROS_INFO("Read %d trajectory %d - %d", current_agent, readed_indices[current_agent], readed_indices[current_agent] + NUM_WAYPOINTS);

    readed_indices[current_agent] += NUM_WAYPOINTS;
    ++agent_trajectory_count_[current_agent];

    current_agent = (current_agent + 1) % readed_trajectories.size();

    return true;
}

void FullBodyPlannerServer::compute3DTrajectory(Trajectory2D& trajectory2d)
{
    node_handle_.setParam("/itomp_planner/agent_id", trajectory2d.front().agentID);
    node_handle_.setParam("/itomp_planner/agent_trajectory_index", agent_trajectory_count_[trajectory2d.front().agentID] - 1);

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    setPlanningRequest(req, trajectory2d);

    // call planner
    plan(req, res);

    updateTrajectory2DFromPlanningResponse(trajectory2d, res);

    displayTrajectory(trajectory2d.front().agentID, res);
}

void FullBodyPlannerServer::sendResponse(const Trajectory2D& trajectory2d)
{

}

void FullBodyPlannerServer::terminate()
{
    // clean up
    planner_instance_.reset();
    planner_plugin_loader_.reset();
    planning_scene_.reset();
    robot_model_.reset();
    robot_model_loader_.reset();
}

void FullBodyPlannerServer::loadPlanner()
{
    // planner initialization
    std::string planner_plugin_name;
    if (!node_handle_.getParam("/move_itomp/planning_plugin", planner_plugin_name))
        ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
        planner_plugin_loader_.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
                                         "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }

    try
    {
        cpu_set_t mask;
        if (sched_getaffinity(0, sizeof(cpu_set_t), &mask) != 0)
            ROS_ERROR("sched_getaffinity failed");
        planner_instance_.reset(planner_plugin_loader_->createUnmanagedInstance(planner_plugin_name));
        if (sched_setaffinity(0, sizeof(cpu_set_t), &mask) != 0)
            ROS_ERROR("sched_setaffinity failed");

        if (!planner_instance_->initialize(robot_model_, node_handle_.getNamespace()))
            ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planning interface '" << planner_instance_->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
        const std::vector<std::string> &classes = planner_plugin_loader_->getDeclaredClasses();
        std::stringstream ss;
        for (std::size_t i = 0; i < classes.size(); ++i)
            ss << classes[i] << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl << "Available plugins: " << ss.str());
    }
}

void FullBodyPlannerServer::setPlanningRequest(planning_interface::MotionPlanRequest& req, const Trajectory2D& trajectory2d)
{
    robot_state::RobotState start_state = planning_scene_->getCurrentStateNonConst();
    robot_state::RobotState goal_state = start_state;

    setStandingState(start_state, trajectory2d.front());
    setStandingState(goal_state, trajectory2d.back());

    robot_state::robotStateToRobotStateMsg(start_state, req.start_state);

    const robot_state::JointModelGroup* joint_model_group = goal_state.getJointModelGroup(group_name_);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal);

    renderState(start_state, "/move_itomp/display_start_state", color_start_);
    renderState(goal_state, "/move_itomp/display_goal_state", color_goal_);
}

void FullBodyPlannerServer::updateTrajectory2DFromPlanningResponse(Trajectory2D& trajectory2d, const planning_interface::MotionPlanResponse& res)
{

}

void FullBodyPlannerServer::plan(planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res)
{
    req.group_name = group_name_;
    req.allowed_planning_time = 300.0;
    req.num_planning_attempts = 1;

    planning_interface::PlanningContextPtr context = planner_instance_->getPlanningContext(planning_scene_, req, res.error_code_);

    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
        return;
    }
}

void FullBodyPlannerServer::loadStaticScene()
{
    moveit_msgs::PlanningScene planning_scene_msg;
    std::string environment_file;
    std::vector<double> environment_position;

    node_handle_.param<std::string>("/itomp_planner/environment_model", environment_file, "");

    if (!environment_file.empty())
    {
        double scale;
        node_handle_.param("/itomp_planner/environment_model_scale", scale, 1.0);
        environment_position.resize(3, 0);
        if (node_handle_.hasParam("/itomp_planner/environment_model_position"))
        {
            XmlRpc::XmlRpcValue segment;
            node_handle_.getParam("/itomp_planner/environment_model_position", segment);
            if (segment.getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                int size = segment.size();
                for (int i = 0; i < size; ++i)
                {
                    double value = segment[i];
                    environment_position[i] = value;
                }
            }
        }

        // Collision object
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = robot_model_->getModelFrame();
        collision_object.id = "environment";
        geometry_msgs::Pose pose;
        pose.position.x = environment_position[0];
        pose.position.y = environment_position[1];
        pose.position.z = environment_position[2];
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;

        shapes::Mesh* shape = shapes::createMeshFromResource(environment_file, Eigen::Vector3d(scale, scale, scale));
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(shape, mesh_msg);
        shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

        collision_object.meshes.push_back(mesh);
        collision_object.mesh_poses.push_back(pose);

        collision_object.operation = collision_object.ADD;
        planning_scene_msg.world.collision_objects.push_back(collision_object);
        planning_scene_msg.is_diff = true;
        planning_scene_->setPlanningSceneDiffMsg(planning_scene_msg);
    }

    planning_scene_diff_publisher_.publish(planning_scene_msg);
}

void FullBodyPlannerServer::displayTrajectory(int index, const planning_interface::MotionPlanResponse& res)
{
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    if (index >= display_trajectory_publishers_.size())
    {
        display_trajectory_publishers_.resize(index + 1);
        display_trajectories_.resize(index + 1);

        stringstream ss;
        ss << "/move_group/display_planned_path_" << index;
        display_trajectory_publishers_[index] = node_handle_.advertise<moveit_msgs::DisplayTrajectory>(ss.str(), 1, true);

        display_trajectories_[index].trajectory_start = response.trajectory_start;
        display_trajectories_[index].trajectory.push_back(response.trajectory);
    }
    else
    {
        moveit_msgs::RobotTrajectory trajectory = response.trajectory;
        trajectory.joint_trajectory.points.erase(trajectory.joint_trajectory.points.begin());
        display_trajectories_[index].trajectory.push_back(trajectory);
    }


    display_trajectory_publishers_[index].publish(display_trajectories_[index]);
}

void FullBodyPlannerServer::renderState(const robot_state::RobotState& state, const std::string& topic, const std_msgs::ColorRGBA& color)
{
    if (state_display_publishers_.find(topic) == state_display_publishers_.end())
        state_display_publishers_[topic] = node_handle_.advertise<moveit_msgs::DisplayRobotState>(topic, 1, true);
    ros::Publisher& state_display_publisher = state_display_publishers_[topic];

    int num_variables = state.getVariableNames().size();
    moveit_msgs::DisplayRobotState disp_state;
    disp_state.state.joint_state.header.frame_id = robot_model_->getModelFrame();
    disp_state.state.joint_state.name = state.getVariableNames();
    disp_state.state.joint_state.position.resize(num_variables);
    memcpy(&disp_state.state.joint_state.position[0], state.getVariablePositions(), sizeof(double) * num_variables);
    disp_state.highlight_links.clear();
    const std::vector<std::string>& link_model_names = robot_model_->getLinkModelNames();
    for (unsigned int i = 0; i < link_model_names.size(); ++i)
    {
        moveit_msgs::ObjectColor obj_color;
        obj_color.id = link_model_names[i];
        obj_color.color = color;
        disp_state.highlight_links.push_back(obj_color);
    }
    state_display_publisher.publish(disp_state);
}

void FullBodyPlannerServer::drawPosition(const Eigen::Vector3d& position, int id, const std_msgs::ColorRGBA& color)
{
    const double scale = 0.02;

    visualization_msgs::Marker msg;
    msg.header.frame_id = robot_model_->getModelFrame();
    msg.header.stamp = ros::Time::now();
    msg.ns = "position";
    msg.type = visualization_msgs::Marker::CUBE_LIST;
    msg.action = visualization_msgs::Marker::ADD;

    msg.scale.x = scale;
    msg.scale.y = scale;
    msg.scale.z = scale;

    msg.id = id;
    msg.color = color;

    msg.points.resize(0);
    geometry_msgs::Point point;
    point.x = position(0);
    point.y = position(1);
    point.z = position(2);
    msg.points.push_back(point);

    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(msg);
    vis_marker_array_publisher_.publish(ma);
}

void FullBodyPlannerServer::drawPath(const Eigen::Vector3d& from, const Eigen::Vector3d& to, int id)
{
    const double scale = 0.005;

    visualization_msgs::Marker::_color_type BLUE, LIGHT_YELLOW;
    visualization_msgs::Marker::_color_type RED, LIGHT_RED;
    RED.a = 1.0;
    RED.r = 1.0;
    RED.g = 0.0;
    RED.b = 0.0;
    BLUE.a = 1.0;
    BLUE.r = 0.5;
    BLUE.g = 0.5;
    BLUE.b = 1.0;
    LIGHT_RED = RED;
    LIGHT_RED.g = 0.5;
    LIGHT_RED.b = 0.5;
    LIGHT_YELLOW = BLUE;
    LIGHT_YELLOW.b = 0.5;

    visualization_msgs::Marker msg;
    msg.header.frame_id = robot_model_->getModelFrame();
    msg.header.stamp = ros::Time::now();
    msg.ns = "path";
    msg.type = visualization_msgs::Marker::LINE_LIST;
    msg.action = visualization_msgs::Marker::ADD;

    msg.scale.x = scale;
    msg.scale.y = scale;
    msg.scale.z = scale;

    msg.id = id;
    msg.color = BLUE;

    msg.points.resize(0);
    geometry_msgs::Point point;
    point.x = from(0) - 0.001;
    point.y = from(1);
    point.z = from(2);
    msg.points.push_back(point);
    point.x = to(0) - 0.001;
    point.y = to(1);
    point.z = to(2);
    msg.points.push_back(point);

    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(msg);
    vis_marker_array_publisher_.publish(ma);
}

void FullBodyPlannerServer::setStandingState(robot_state::RobotState& state, Waypoint2D waypoint_2d)
{
    const robot_state::JointModelGroup* joint_model_group = state.getJointModelGroup("whole_body");

    std::map<std::string, double> values;
    joint_model_group->getVariableDefaultPositions("idle", values);
    state.setVariablePositions(values);

    state.setVariablePosition("base_prismatic_joint_x", waypoint_2d.x);
    state.setVariablePosition("base_prismatic_joint_y", waypoint_2d.y);
    state.setVariablePosition("base_revolute_joint_z", waypoint_2d.orientation);
}

}

int main(int argc, char **argv)
{
    // for debug
    setbuf(stdout, NULL);

    ros::init(argc, argv, "full_body_planner");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    full_body_planner::FullBodyPlannerServer* full_body_planner = new full_body_planner::FullBodyPlannerServer(node_handle);

    full_body_planner->init();

    full_body_planner::Trajectory2D trajectory2d;

    // loop
    while(full_body_planner->getInput(trajectory2d))
    {
        full_body_planner->compute3DTrajectory(trajectory2d);
        full_body_planner->sendResponse(trajectory2d);
    }

    full_body_planner->terminate();
    delete full_body_planner;

    return 0;
}
