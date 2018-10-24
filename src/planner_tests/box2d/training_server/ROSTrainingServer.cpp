#include <geometry_msgs/Pose2D.h>
#include <planner_tests/box2d/training_server/ROSTrainingServer.h>

using namespace planner_tests::box2d::training_server;

ROSTrainingServer::ROSTrainingServer(mps::planner::pushing::PlanningProblem& planning_problem,
    const std::string& ros_ns)
    : _nhandle(ros_ns)
    , _planning_problem(planning_problem)
{
    _get_properties_service = _nhandle.advertiseService("get_object_properties", &ROSTrainingServer::get_object_properties, this);
    _get_action_space_info_service = _nhandle.advertiseService("get_action_space_info", &ROSTrainingServer::get_action_space_info, this);
    _get_state_service = _nhandle.advertiseService("get_state", &ROSTrainingServer::get_state, this);
    _propagate_service = _nhandle.advertiseService("propagate", &ROSTrainingServer::propagate, this);
    _set_active_objects_service = _nhandle.advertiseService("set_active_objects", &ROSTrainingServer::set_active_objects, this);
    _set_state_service = _nhandle.advertiseService("set_state", &ROSTrainingServer::set_state, this);
    _box2d_world = std::dynamic_pointer_cast<sim_env::Box2DWorld>(_planning_problem.world);
    _logger = _box2d_world->getLogger();
}

ROSTrainingServer::~ROSTrainingServer() = default;

void ROSTrainingServer::setup_planner()
{
    _planner.setup(_planning_problem);
}

bool ROSTrainingServer::get_object_properties(planner_tests::GetObjectProperties::Request& req,
    planner_tests::GetObjectProperties::Response& res)
{
    const static std::string log_prefix("ROSTrainingServer::get_object_properties");
    _logger->logDebug("Retrieving object properties", log_prefix);
    std::vector<sim_env::Box2DObjectPtr> box2d_objects;
    _box2d_world->getBox2DObjects(box2d_objects);
    for (auto& object : box2d_objects) {
        res.obj_names.push_back(object->getName());
        res.masses.push_back(object->getMass());
        res.inertias.push_back(object->getInertia());
        res.friction_coeffs.push_back(object->getGroundFriction());
        auto aabb = object->getLocalAABB();
        res.widths.push_back(aabb.getWidth());
        res.heights.push_back(aabb.getHeight());
    }
    return true;
}

bool ROSTrainingServer::get_action_space_info(planner_tests::GetActionSpaceInfo::Request& req,
    planner_tests::GetActionSpaceInfo::Response& res)
{
    // TODO this is specific to RampVelocityControl
    // TODO also need to update RampVelocityControl to be parameterized differently
    // TODO it makes most sense to parameterize it as (tvel(m/s), rot_vel(rot/s), duration(s), direction (angle))
    res.dof_action_space = 4;
    res.lower_bounds.push_back(0.0);
    res.lower_bounds.push_back(-_planning_problem.control_limits.velocity_limits[2]);
    res.lower_bounds.push_back(_planning_problem.control_limits.duration_limits[0]);
    res.lower_bounds.push_back(0.0);
    // upper bounds for ramp twist
    res.upper_bounds.push_back(_planning_problem.control_limits.velocity_limits[0]);
    res.upper_bounds.push_back(_planning_problem.control_limits.velocity_limits[2]);
    res.upper_bounds.push_back(_planning_problem.control_limits.duration_limits[1]);
    res.upper_bounds.push_back(2.0f * M_PI);
    res.is_cyclic = { false, false, false, true };
    return true;
}

bool ROSTrainingServer::get_state(planner_tests::GetState::Request& req,
    planner_tests::GetState::Response& res)
{
    std::vector<sim_env::Box2DObjectPtr> objects;
    _box2d_world->getBox2DObjects(objects);
    for (auto obj : objects) {
        res.obj_names.push_back(obj->getName());
        Eigen::Vector3f pose;
        obj->getPose(pose);
        geometry_msgs::Pose2D pose_msg;
        pose_msg.x = pose[0];
        pose_msg.y = pose[1];
        pose_msg.theta = pose[2];
        res.states.push_back(pose_msg);
    }
    std::vector<sim_env::Box2DRobotPtr> robots;
    _box2d_world->getBox2DRobots(robots);
    for (auto rob : robots) {
        res.obj_names.push_back(rob->getName());
        Eigen::Vector3f pose;
        rob->getPose(pose);
        geometry_msgs::Pose2D pose_msg;
        pose_msg.x = pose[0];
        pose_msg.y = pose[1];
        pose_msg.theta = pose[2];
        res.states.push_back(pose_msg);
    }
    return true;
}

bool ROSTrainingServer::propagate(planner_tests::Propagate::Request& req, planner_tests::Propagate::Response& res)
{
    // TODO construct action from parameterization and use propagator to predict outcome
    return true;
}

bool ROSTrainingServer::set_active_objects(planner_tests::SetActiveObjects::Request& req,
    planner_tests::SetActiveObjects::Response& res)
{
    // TODO figure out how to activate and deactivate objects in box2d
    std::cout << "set_active_objects" << std::endl;
    return true;
}

bool ROSTrainingServer::set_state(planner_tests::SetState::Request& req, planner_tests::SetState::Response& res)
{
    // TODO
    return true;
}
