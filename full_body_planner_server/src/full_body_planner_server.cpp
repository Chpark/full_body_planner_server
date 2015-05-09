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
#include <menge_3d_interface/3DInterfaceMessageTypes.h>

using namespace std;

namespace full_body_planner
{

const int NUM_WAYPOINTS = 20;

FullBodyPlannerServer::FullBodyPlannerServer(const ros::NodeHandle& node_handle) :
    node_handle_(node_handle), trajectory_count_(0), response_data_(0), response_data_num_agents_(0), response_data_agent_size_(0)
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
    initServer();

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

bool FullBodyPlannerServer::getInput(std::vector<Trajectory2D>& trajectories)
{
    unsigned int msg = 0, numAgents=0, numFrames=0, agentSize=0;
    bool test = false;
    float **agentData=0x0;

    while (true)
    {
        test = server_.receiveMessage(msg);
        if (test && msg == Interface3D::CONNECT)
        {
            server_.handleConnectMessage();
            std::cout << "New Connection!" <<std::endl;
        }
        else if (test && msg == Interface3D::AGENT_DATA)
        {
            std::cout << "got agent data message. Handle it." << std::endl;
            server_.handleAgentDataMessage(numFrames, numAgents, agentSize, agentData);

            trajectories.resize(numAgents);
            for (unsigned int agent = 0; agent < numAgents; ++agent)
            {
                trajectories[agent].resize(numFrames);
                for (unsigned int frame = 0; frame < numFrames; ++frame)
                {
                    Waypoint2D& waypoint = trajectories[agent][frame];

                    /*
                    waypoint.frame = agentData[frame * numAgents + agent][0];
                    waypoint.agent_id = agentData[frame * numAgents + agent][1];
                    waypoint.radius = agentData[frame * numAgents + agent][2];
                    waypoint.x = agentData[frame * numAgents + agent][3];
                    waypoint.y = agentData[frame * numAgents + agent][4];
                    waypoint.orientation = agentData[frame * numAgents + agent][5];
                    waypoint.state = agentData[frame * numAgents + agent][6];
                    waypoint.pvx = agentData[frame * numAgents + agent][7];
                    waypoint.pvy = agentData[frame * numAgents + agent][8];
                    waypoint.vx = agentData[frame * numAgents + agent][9];
                    waypoint.vy = agentData[frame * numAgents + agent][10];
                    waypoint.neighbors.resize(agentData[frame * numAgents + agent][11]);
                    */

                    waypoint.frame = agentData[frame * numAgents + agent][0];
                    waypoint.agent_id = agentData[frame * numAgents + agent][1];
                    waypoint.state = agentData[frame * numAgents + agent][2];
                    waypoint.x = agentData[frame * numAgents + agent][3];
                    waypoint.y = agentData[frame * numAgents + agent][4];
                    waypoint.vx = agentData[frame * numAgents + agent][5];
                    waypoint.vy = agentData[frame * numAgents + agent][6];
                    waypoint.pvx = agentData[frame * numAgents + agent][7];
                    waypoint.pvy = agentData[frame * numAgents + agent][8];
                    waypoint.orientation = agentData[frame * numAgents + agent][9];
                    waypoint.radius = agentData[frame * numAgents + agent][10];
                    waypoint.neighbors.resize(agentData[frame * numAgents + agent][11]);

                    for (unsigned int neighbor = 0; neighbor < waypoint.neighbors.size(); ++neighbor)
                        waypoint.neighbors[neighbor] = agentData[frame * numAgents + agent][12 + neighbor];

                    cout << waypoint;
                }
            }
            break;
        }
        sleep(0);
    }

    ++trajectory_count_;

    return true;
}

bool FullBodyPlannerServer::compute3DTrajectory(std::vector<Trajectory2D>& trajectories, int index)
{
    Trajectory2D& trajectory2d = trajectories[index];

    node_handle_.setParam("/itomp_planner/agent_id", trajectory2d.front().agent_id);
    node_handle_.setParam("/itomp_planner/agent_trajectory_index", trajectory_count_ - 1);

    node_handle_.setParam("/itomp_planner/agent_vel_x_0", trajectory2d.front().vx);
    node_handle_.setParam("/itomp_planner/agent_vel_y_0", trajectory2d.front().vy);
    node_handle_.setParam("/itomp_planner/agent_vel_x_10", trajectory2d[trajectory2d.size() / 2].vx);
    node_handle_.setParam("/itomp_planner/agent_vel_y_10", trajectory2d[trajectory2d.size() / 2].vy);
    node_handle_.setParam("/itomp_planner/agent_vel_x_20", trajectory2d.back().vx);
    node_handle_.setParam("/itomp_planner/agent_vel_y_20", trajectory2d.back().vy);

    node_handle_.setParam("/itomp_planner/agent_pref_vel_x_0", trajectory2d.front().pvx);
    node_handle_.setParam("/itomp_planner/agent_pref_vel_y_0", trajectory2d.front().pvy);
    node_handle_.setParam("/itomp_planner/agent_pref_vel_x_10", trajectory2d[trajectory2d.size() / 2].pvx);
    node_handle_.setParam("/itomp_planner/agent_pref_vel_y_10", trajectory2d[trajectory2d.size() / 2].pvy);
    node_handle_.setParam("/itomp_planner/agent_pref_vel_x_20", trajectory2d.back().pvx);
    node_handle_.setParam("/itomp_planner/agent_pref_vel_y_20", trajectory2d.back().pvy);

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    setPlanningRequest(req, trajectories, index);

    // call planner
    bool success = plan(req, res);

    updateTrajectory2DFromPlanningResponse(trajectory2d, res);

    if (success)
        displayTrajectory(trajectory2d.front().agent_id, res);

    ROS_INFO("Planning of agent %d trajectory %d : %s", trajectory2d.front().agent_id, trajectory_count_ - 1, (success ? "Success" : "Fail"));

    return success;
}

void FullBodyPlannerServer::sendResponse(const std::vector<Trajectory2D>& trajectories, bool success)
{
    if (success)
    {
        const unsigned int agentSize = 5;

        unsigned int numAgents = trajectories.size();
        float **response = getResponseMemory(numAgents, agentSize);

        for (unsigned int agent = 0; agent < numAgents; ++agent)
        {
            const Waypoint2D& waypoint = trajectories[agent].back();

            double orientation = waypoint.orientation + M_PI_2;
            if (orientation > M_PI)
                orientation -= 2.0 * M_PI;

            response[agent][0] = waypoint.agent_id;
            response[agent][1] = waypoint.x;
            response[agent][2] = waypoint.y;
            response[agent][3] = orientation;
            response[agent][4] = waypoint.locked;
        }

        server_.sendSuccessMessage(numAgents, agentSize, response);
    }
    else
    {
        const unsigned int agentSize = 1;

        std::vector<int> failed_agent_ids;
        for (unsigned int agent = 0; agent < trajectories.size(); ++agent)
        {
            const Waypoint2D& waypoint = trajectories[agent].back();
            if (waypoint.failed == 1)
                failed_agent_ids.push_back(waypoint.agent_id);
        }
        unsigned int numAgents = failed_agent_ids.size();

        float **response = getResponseMemory(numAgents, agentSize);

        for (unsigned int agent = 0; agent < numAgents; ++agent)
        {
            response[agent][0] = failed_agent_ids[agent];
        }

        server_.sendFailMessage(numAgents, agentSize, response);
    }
}

void FullBodyPlannerServer::terminate()
{
    // clean up
    planner_instance_.reset();
    planner_plugin_loader_.reset();
    planning_scene_.reset();
    robot_model_.reset();
    robot_model_loader_.reset();

    deallocateResponseMemory();
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

void FullBodyPlannerServer::setPlanningRequest(planning_interface::MotionPlanRequest& req, const std::vector<Trajectory2D>& trajectories, int index)
{
    const Trajectory2D& trajectory2d = trajectories[index];

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

    int max_neighbors = 0;
    for (int i = 0; i < trajectory2d.size(); ++i)
    {
        if (trajectory2d[i].neighbors.size() > max_neighbors)
            max_neighbors = trajectory2d[i].neighbors.size();
    }

    // set robot trajectories as constraints
    req.trajectory_constraints.constraints.reserve(max_neighbors + 1);
    for (int i = 0; i < max_neighbors + 1; ++i)
    {
        moveit_msgs::Constraints c;
        c.position_constraints.reserve(trajectories[index].size());
        for (int j = 0; j < trajectories[index].size(); ++j)
        {
            moveit_msgs::PositionConstraint pc;
            if (i == 0)
            {
                pc.weight = index; // robot index
                pc.target_point_offset.z = trajectories[index][j].radius; // is it ok?
            }
            else
            {
                int neighbor_id = -1;

                if (trajectory2d[j].neighbors.size() >= i)
                {
                    neighbor_id = trajectory2d[j].neighbors[i - 1];
                    pc.weight = neighbor_id;
                    pc.target_point_offset.x = trajectories[neighbor_id][j].x;
                    pc.target_point_offset.y = trajectories[neighbor_id][j].y;
                    pc.target_point_offset.z = trajectories[neighbor_id][j].radius; // is it ok?
                }
            }
            c.position_constraints.push_back(pc);
        }

        req.trajectory_constraints.constraints.push_back(c);
    }
}

void FullBodyPlannerServer::updateTrajectory2DFromPlanningResponse(Trajectory2D& trajectory2d, const planning_interface::MotionPlanResponse& res)
{
    trajectory2d.back().failed = (res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS) ? 0 : 1;
    trajectory2d.back().locked = 0;

    if (!trajectory2d.back().failed)
    {
        trajectory2d.back().x = res.trajectory_->getLastWayPoint().getVariablePosition("base_prismatic_joint_x");
        trajectory2d.back().y = res.trajectory_->getLastWayPoint().getVariablePosition("base_prismatic_joint_y");
        trajectory2d.back().orientation = res.trajectory_->getLastWayPoint().getVariablePosition("base_revolute_joint_z");
    }
}

bool FullBodyPlannerServer::plan(planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res)
{
    req.group_name = group_name_;
    req.allowed_planning_time = 300.0;
    req.num_planning_attempts = 1;

    planning_interface::PlanningContextPtr context = planner_instance_->getPlanningContext(planning_scene_, req, res.error_code_);

    if (context->solve(res) == false)
        return false;

    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
        return false;
    }
    return true;
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
    }

    void* pub = display_trajectory_publishers_[index];
    if (!pub)
    {
        while (!pub)
        {
            stringstream ss;
            ss << "/move_group/display_planned_path_" << index;
            display_trajectory_publishers_[index] = node_handle_.advertise<moveit_msgs::DisplayTrajectory>(ss.str(), 1, true);

            pub = display_trajectory_publishers_[index];

            ROS_INFO("Reinitialize display trajectory publisher %d", index);
        }

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

void FullBodyPlannerServer::initServer()
{
    server_.initSocket("*", "5557");
    std::cout << "Server started" << std::endl;
}

float** FullBodyPlannerServer::getResponseMemory(int num_agents, int agent_size)
{
    if (num_agents <= response_data_num_agents_)
        return response_data_;
    if (agent_size <= response_data_agent_size_)
        return response_data_;

    deallocateResponseMemory();

    response_data_num_agents_ = num_agents;
    response_data_agent_size_ = agent_size;

    response_data_ = new float*[num_agents];
    for (int i = 0; i < num_agents; ++i)
        response_data_[i] = new float[agent_size];

    return response_data_;
}

void FullBodyPlannerServer::deallocateResponseMemory()
{
    for (int i = 0; i < response_data_num_agents_; ++i)
        delete[] response_data_[i];
    delete[] response_data_;

    response_data_num_agents_ = 0;
    response_data_agent_size_ = 0;
    response_data_ = 0;
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

    vector<full_body_planner::Trajectory2D> trajectories;

    // loop
    while(full_body_planner->getInput(trajectories))
    {
        bool success = true;
        for (int i = 0; i < trajectories.size(); ++i)
            success &= full_body_planner->compute3DTrajectory(trajectories, i);
        full_body_planner->sendResponse(trajectories, success);

        if (!success)
            full_body_planner->decreaseTrajectoryCount();
    }

    full_body_planner->terminate();
    delete full_body_planner;

    return 0;
}
