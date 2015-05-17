#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/PlanningScene.h>
#include <map>
#include <cmath>
#include <boost/variant/get.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <sched.h>

//file handling
#include <string>
#include <sstream>
#include <fstream>

using namespace std;

const double INV_SQRT_2 = 1.0 / std::sqrt((long double) 2.0);

void loadStaticScene(ros::NodeHandle& node_handle,
                     planning_scene::PlanningScenePtr& planning_scene,
                     robot_model::RobotModelPtr& robot_model,
                     ros::Publisher& planning_scene_diff_publisher)
{
    std::string environment_file;
    std::vector<double> environment_position;

    node_handle.param<std::string>("/itomp_planner/environment_model", environment_file, "");

    //environment_file = "package://boeing_app/meshes/BoeingSeats.dae";

    if (!environment_file.empty())
    {
        environment_position.resize(3, 0);
        if (node_handle.hasParam("/itomp_planner/environment_model_position"))
        {
            XmlRpc::XmlRpcValue segment;
            node_handle.getParam("/itomp_planner/environment_model_position", segment);
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
        collision_object.header.frame_id = robot_model->getModelFrame();
        collision_object.id = "environment";
        geometry_msgs::Pose pose;
        pose.position.x = environment_position[0];
        pose.position.y = environment_position[1];
        pose.position.z = environment_position[2];
        ROS_INFO("Env col pos : (%f %f %f)", environment_position[0], environment_position[1], environment_position[2]);
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;

        shapes::Mesh* shape = shapes::createMeshFromResource(environment_file);
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(shape, mesh_msg);
        shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

        collision_object.meshes.push_back(mesh);
        collision_object.mesh_poses.push_back(pose);

        collision_object.operation = collision_object.ADD;
        moveit_msgs::PlanningScene planning_scene_msg;
        planning_scene_msg.world.collision_objects.push_back(collision_object);
        planning_scene_msg.is_diff = true;
        planning_scene->setPlanningSceneDiffMsg(planning_scene_msg);

        planning_scene_diff_publisher.publish(planning_scene_msg);
    }
}


void readTrajectory(moveit_msgs::DisplayTrajectory& display_trajectory, const string& filename, int num_points, planning_scene::PlanningScenePtr& planning_scene,
                    std::vector<robot_state::RobotState>& robot_states)
{
    int start = robot_states.size();

    robot_state::RobotState state(planning_scene->getCurrentState());

    std::ifstream trajectory_file;
    trajectory_file.open(filename.c_str());
    if (trajectory_file.is_open())
    {
        std::string temp;
        for (int i = 0; i < 4; ++i)
            std::getline(trajectory_file, temp);

        int num_joints = state.getVariableCount();

        int num_lines = 0;
        while (trajectory_file >> temp && num_lines < num_points)
        {
            trajectory_file >> temp;
            for (int j = 0; j < num_joints; ++j)
            {
                trajectory_file >> *(state.getVariablePositions() + j) ;
                std::cout << *(state.getVariablePositions() + j) << " ";
            }
            std::cout << std::endl;

            state.update(true);
            robot_states.push_back(state);

            ++num_lines;
        }

        trajectory_file.close();
    }

    // convert to moveit_msg
    moveit_msgs::MotionPlanResponse response;
    planning_interface::MotionPlanResponse res;
    res.error_code_.val = 1;

    res.trajectory_.reset(new robot_trajectory::RobotTrajectory(robot_states[0].getRobotModel(), ""));
    for (int i = start; i < robot_states.size() - 1; ++i)
    {
        res.trajectory_->addSuffixWayPoint(robot_states[i], 0.05);
    }
    res.getMessage(response);

    display_trajectory.trajectory.push_back(response.trajectory);
}

void visualizeTrajectory(moveit_msgs::DisplayTrajectory& display_trajectory, ros::NodeHandle& node_handle)
{
    static ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    display_publisher.publish(display_trajectory);
    ros::WallDuration timer(0.5);
    timer.sleep();
}

using namespace Eigen;
template<typename _Scalar> class EulerAngles
{
public:
    enum { Dim = 3 };
    typedef _Scalar Scalar;
    typedef Matrix<Scalar,3,3> Matrix3;
    typedef Matrix<Scalar,3,1> Vector3;
    typedef Quaternion<Scalar> QuaternionType;

protected:
    Vector3 m_angles;

public:
    EulerAngles() {}
    inline EulerAngles(Scalar a0, Scalar a1, Scalar a2) : m_angles(a0, a1, a2) {}
    inline EulerAngles(const QuaternionType& q)
    {
        *this = q;
    }

    const Vector3& coeffs() const
    {
        return m_angles;
    }
    Vector3& coeffs()
    {
        return m_angles;
    }

    EulerAngles& operator=(const QuaternionType& q)
    {
        Matrix3 m = q.toRotationMatrix();
        return *this = m;
    }

    EulerAngles& operator=(const Matrix3& m)
    {
        // mat = cy*cz -cy*sz sy
        // cz*sx*sy+cx*sz cx*cz-sx*sy*sz -cy*sx
        // -cx*cz*sy+sx*sz cz*sx+cx*sy*sz cx*cy

        m_angles.coeffRef(1) = std::asin(m.coeff(0,2));
        m_angles.coeffRef(0) = std::atan2(-m.coeff(1,2),m.coeff(2,2));
        m_angles.coeffRef(2) = std::atan2(-m.coeff(0,1),m.coeff(0,0));

        /*
        double eps = 1e-7;
        if (std::abs(std::abs(m.coeff(2, 0)) - 1.0) > eps)
        {
            m_angles.coeffRef(1) = -std::asin(m.coeff(0,2));
            m_angles.coeffRef(0) = std::atan2(m.coeff(2,1)/std::cos(m_angles.coeffRef(1)),
                                              m.coeff(2,2)/std::cos(m_angles.coeffRef(1)));
            m_angles.coeffRef(2) = std::atan2(m.coeff(1,0)/std::cos(m_angles.coeffRef(1)),
                                              m.coeff(0,0)/std::cos(m_angles.coeffRef(1)));
        }
        else
        {
            m_angles.coeffRef(2) = 0.0;
            double delta = std::atan2(m.coeff(0,1), m.coeff(0, 2));
            if (std::abs(m.coeff(2, 0) + 1.0) < eps)
            {
                m_angles.coeffRef(1) = M_PI_2;
                m_angles.coeffRef(0) = m_angles.coeffRef(2) + delta;
            }
            else
            {
                m_angles.coeffRef(1) = -M_PI_2;
                m_angles.coeffRef(0) = -m_angles.coeffRef(2) + delta;
            }
        }
        */





        return *this;
    }

    Matrix3 toRotationMatrix(void) const
    {
        Vector3 c = m_angles.array().cos();
        Vector3 s = m_angles.array().sin();
        Matrix3 res;
        res << c.y()*c.z(), -c.y()*s.z(), s.y(),
            c.z()*s.x()*s.y()+c.x()*s.z(), c.x()*c.z()-s.x()*s.y()*s.z(), -c.y()*s.x(),
            -c.x()*c.z()*s.y()+s.x()*s.z(), c.z()*s.x()+c.x()*s.y()*s.z(), c.x()*c.y();
        return res;
    }

    operator QuaternionType()
    {
        return QuaternionType(toRotationMatrix());
    }
};
typedef EulerAngles<double> EulerAnglesd;

Eigen::Matrix3d getZRotated(const Eigen::Matrix3d& mat)
{
    Eigen::Matrix3d z_rotation;
    z_rotation = Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitZ());
    return mat * z_rotation;
}

EulerAnglesd getZRotated(EulerAnglesd angles)
{
    {
        Eigen::Matrix3d rot_x, rot_y, rot_z, rot_z_pi_2;
        rot_z = Eigen::AngleAxisd(angles.coeffs()[2], Eigen::Vector3d::UnitZ());
        rot_y = Eigen::AngleAxisd(angles.coeffs()[1], Eigen::Vector3d::UnitY());
        rot_x = Eigen::AngleAxisd(angles.coeffs()[0], Eigen::Vector3d::UnitX());
        rot_z_pi_2 = Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitZ());

        Eigen::Matrix3d rot = rot_x * rot_y * rot_z;
        Eigen::Matrix3d rot2 = angles.toRotationMatrix();

        Vector3d ea = rot.eulerAngles(0, 1, 2);
        Vector3d ea2 = rot.eulerAngles(2, 1, 0);

        EulerAnglesd ret;
        ret = rot;
    }

    Eigen::Matrix3d rot_x, rot_y, rot_z, rot_z_pi_2;
    rot_z = Eigen::AngleAxisd(angles.coeffs()[2] - 0.5 * M_PI, Eigen::Vector3d::UnitZ());
    rot_y = Eigen::AngleAxisd(angles.coeffs()[0], Eigen::Vector3d::UnitY());
    rot_x = Eigen::AngleAxisd(-angles.coeffs()[1], Eigen::Vector3d::UnitX());
    //rot_z_pi_2 = Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitZ());

    Eigen::Matrix3d rot;
    EulerAnglesd ret;

    rot = rot_x * rot_y;
    ret = rot;

    rot = rot_z * rot_x * rot_y;
    ret = rot;

    return ret;
}

Eigen::Vector3d getZRotated(Eigen::Vector3d ea)
{
    {
        Eigen::Matrix3d rot_x, rot_y, rot_z;
        rot_z = Eigen::AngleAxisd(ea(2), Eigen::Vector3d::UnitZ());
        rot_y = Eigen::AngleAxisd(ea(1), Eigen::Vector3d::UnitY());
        rot_x = Eigen::AngleAxisd(ea(0), Eigen::Vector3d::UnitX());

        Eigen::Matrix3d rot = rot_z * rot_y * rot_x;
        Eigen::Matrix3d rot2 = rot_x * rot_y * rot_z;

        Vector3d ea2 = rot.eulerAngles(0, 1, 2);
        Vector3d ea3 = rot2.eulerAngles(0, 1, 2);
    }

    Eigen::Vector3d ret;

    Eigen::Matrix3d rot_x, rot_y, rot_z;
    rot_z = Eigen::AngleAxisd(ea(0) - M_PI_2, Eigen::Vector3d::UnitZ());
    rot_y = Eigen::AngleAxisd(ea(2), Eigen::Vector3d::UnitY());
    rot_x = Eigen::AngleAxisd(-ea(1), Eigen::Vector3d::UnitX());

    Eigen::Matrix3d rot = rot_z * rot_x * rot_y;
    ret = rot.eulerAngles(2, 1, 0);
    return ret;
}

void writeTransforms(std::vector<robot_state::RobotState>& robot_states)
{
    std::vector<std::pair<std::string, std::string> > link_names;
    link_names.push_back(std::make_pair<std::string, std::string>("pelvis_link", ""));
    link_names.push_back(std::make_pair<std::string, std::string>("torso_x_link", "pelvis_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("head_x_link", "torso_x_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("upper_left_arm_specula_link", "torso_x_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("upper_left_arm_z_link", "upper_left_arm_specula_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("lower_left_arm_link", "upper_left_arm_z_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("left_hand_x_link", "lower_left_arm_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("upper_right_arm_specula_link", "torso_x_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("upper_right_arm_z_link", "upper_right_arm_specula_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("lower_right_arm_link", "upper_right_arm_z_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("right_hand_x_link", "lower_right_arm_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("upper_left_leg_x_link", "pelvis_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("lower_left_leg_link", "upper_left_leg_x_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("left_foot_x_link", "lower_left_leg_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("upper_right_leg_x_link", "pelvis_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("lower_right_leg_link", "upper_right_leg_x_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("right_foot_x_link", "lower_right_leg_link"));

    std::ofstream trajectory_file;
    std::ofstream trajectory_file2;
    trajectory_file.open("euler_transforms_global_oneline.txt");
    trajectory_file2.open("euler_transforms_local_oneline.txt");
    trajectory_file.precision(std::numeric_limits<double>::digits10);
    trajectory_file2.precision(std::numeric_limits<double>::digits10);

    for (int j = 0; j < link_names.size(); ++j)
    {
        trajectory_file << link_names[j].first << std::endl;
        trajectory_file2 << link_names[j].first << std::endl;
    }

    for (int i = 0; i < robot_states.size(); ++i)
    {
        robot_state::RobotState& state = robot_states[i];
        const robot_state::RobotModelConstPtr& robot_model = state.getRobotModel();

        const robot_state::JointModelGroup* joint_model_group = state.getJointModelGroup("whole_body");

        /*
        string pose_name[] = {"tpose", "idle", "overhead_bin"};
        if (i >= 3)
            break;
        std::map<std::string, double> values;
        joint_model_group->getVariableDefaultPositions(pose_name[i], values);
        state.setVariablePositions(values);
        state.update(true);
        */

        for (int j = 0; j < link_names.size(); ++j)
        {
            const Eigen::Affine3d global_transform = state.getGlobalLinkTransform(link_names[j].first);

            Eigen::Matrix3d local;

            if (link_names[j].second != "")
            {
                const Eigen::Affine3d parent_transform = state.getGlobalLinkTransform(link_names[j].second);
                local = (global_transform.linear()) * (parent_transform.linear()).inverse();
            }
            else
                local = (global_transform.linear());

            Eigen::Matrix<double,3,3> global_rot = (global_transform.linear());
            Eigen::Matrix<double,3,3> local_rot = local;

            /*
            EulerAnglesd angles;
            angles = global_rot;
            angles = getZRotated(angles);
            EulerAnglesd angles2;
            angles2 = local_rot;
            angles2 = getZRotated(angles2);
            */

            Eigen::Vector3d ea = global_rot.eulerAngles(2, 1, 0);
            Eigen::Vector3d ea2 = local_rot.eulerAngles(2, 1, 0);

            ea = getZRotated(ea);
            ea2 = getZRotated(ea2);

            trajectory_file << ea(2) << " ";
            trajectory_file << ea(1) << " ";
            trajectory_file << ea(0) << " ";
            trajectory_file2 << ea2(2) << " ";
            trajectory_file2 << ea2(1) << " ";
            trajectory_file2 << ea2(0) << " ";

            if (j == 0)
            {
                trajectory_file << global_transform.translation()(2) << " ";
                trajectory_file << global_transform.translation()(1) << " ";
                trajectory_file << global_transform.translation()(0) << " ";
                trajectory_file2 << global_transform.translation()(2) << " ";
                trajectory_file2 << global_transform.translation()(1) << " ";
                trajectory_file2 << global_transform.translation()(0) << " ";
            }
        }
        trajectory_file << std::endl;
        trajectory_file2 << std::endl;
    }
    trajectory_file.close();
    trajectory_file2.close();
}

void writeUnrealInput(std::vector<robot_state::RobotState>& robot_states, int agent_index)
{
    std::vector<std::pair<std::string, std::string> > link_names;
    link_names.push_back(std::make_pair<std::string, std::string>("pelvis_link", ""));
    link_names.push_back(std::make_pair<std::string, std::string>("torso_x_link", "pelvis_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("head_x_link", "torso_x_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("upper_left_arm_specula_link", "torso_x_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("upper_left_arm_z_link", "upper_left_arm_specula_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("lower_left_arm_link", "upper_left_arm_z_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("left_hand_x_link", "lower_left_arm_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("upper_right_arm_specula_link", "torso_x_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("upper_right_arm_z_link", "upper_right_arm_specula_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("lower_right_arm_link", "upper_right_arm_z_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("right_hand_x_link", "lower_right_arm_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("upper_left_leg_x_link", "pelvis_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("lower_left_leg_link", "upper_left_leg_x_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("left_foot_x_link", "lower_left_leg_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("upper_right_leg_x_link", "pelvis_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("lower_right_leg_link", "upper_right_leg_x_link"));
    link_names.push_back(std::make_pair<std::string, std::string>("right_foot_x_link", "lower_right_leg_link"));

    std::stringstream ss;
    ss << "ue4_transforms_" << agent_index << ".txt";
    std::ofstream trajectory_file;
    trajectory_file.open(ss.str().c_str());
    trajectory_file.precision(std::numeric_limits<double>::digits10);

    for (int j = 0; j < link_names.size(); ++j)
    {
        trajectory_file << link_names[j].first << std::endl;
    }

    for (int i = 0; i < robot_states.size(); ++i)
    {
        robot_state::RobotState& state = robot_states[i];

        for (int j = 0; j < link_names.size(); ++j)
        {
            const Eigen::Affine3d global_transform = state.getGlobalLinkTransform(link_names[j].first);

            Eigen::Matrix3d global_rot = (global_transform.linear());

            if (j == 0)
            {
                trajectory_file << global_transform.translation()(0) << " ";
                trajectory_file << -global_transform.translation()(1) << " ";
                trajectory_file << global_transform.translation()(2) - 0.9619 << " ";
            }

            Eigen::Vector3d ea = -global_rot.eulerAngles(1, 0, 2);
            trajectory_file << -ea(1) * 180.0 / M_PI << " ";
            trajectory_file << ea(0) * 180.0 / M_PI  << " ";
            trajectory_file << ea(2) * 180.0 / M_PI  << " ";
        }
        trajectory_file << std::endl;
    }
    trajectory_file.close();
}

int main(int argc, char **argv)
{
    setbuf(stdout, NULL);

    std::vector<std::pair<std::string, int> > trajectory_file;
    int motion = 0;
    int agent = 0;
    if (argc >= 2)
    {
        motion = atoi(argv[1]);
        if(argc >=3)
        {
            agent = atoi(argv[2]);
        }
    }

    for (int i = 0; i <= motion; ++i)
    {
        std::stringstream ss;
        ss << "trajectory_out_" << std::setfill('0') << std::setw(4) << agent << "_" << std::setfill('0') << std::setw(4) << i << ".txt";
        std::string file_name = ss.str();

        int num_points = std::numeric_limits<int>::max();

        trajectory_file.push_back(std::make_pair(file_name, num_points));
    }

    ros::init(argc, argv, "move_itomp");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    ros::Publisher planning_scene_diff_publisher;
    planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
        ROS_INFO("Waiting planning_scene subscribers");
    }

    loadStaticScene(node_handle, planning_scene, robot_model, planning_scene_diff_publisher);

    /* Sleep a little to allow time to startup rviz, etc. */
    ros::WallDuration sleep_time(1.0);
    sleep_time.sleep();

    // load trajectory
    moveit_msgs::DisplayTrajectory display_trajectory;

    std::vector<robot_state::RobotState> robot_states;
    for (int i = 0; i < trajectory_file.size(); ++i)
        readTrajectory(display_trajectory, trajectory_file[i].first, trajectory_file[i].second, planning_scene, robot_states);

    // visualize trajectory
    visualizeTrajectory(display_trajectory, node_handle);

    // write transforms
    //writeTransforms(robot_states);
    writeUnrealInput(robot_states, agent);



    ROS_INFO("Done");

    return 0;
}
