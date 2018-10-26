#include <geometry_msgs/Pose2D.h>
#include <planner_tests/box2d/training_server/ROSTrainingServer.h>

using namespace planner_tests::box2d::training_server;

ROSTrainingServer::ROSTrainingServer(mps::planner::pushing::PlanningProblem& planning_problem,
    const std::string& ros_ns)
    : _nhandle(ros_ns)
    , _planning_problem(planning_problem)
    , _control(nullptr)
    , _current_state(nullptr)
    , _resulting_state(nullptr)
{
    _get_properties_service = _nhandle.advertiseService("get_object_properties", &ROSTrainingServer::get_object_properties, this);
    _get_action_space_info_service = _nhandle.advertiseService("get_action_space_info", &ROSTrainingServer::get_action_space_info, this);
    _get_state_service = _nhandle.advertiseService("get_state", &ROSTrainingServer::get_state, this);
    _propagate_service = _nhandle.advertiseService("propagate", &ROSTrainingServer::propagate, this);
    _set_active_objects_service = _nhandle.advertiseService("set_active_objects", &ROSTrainingServer::set_active_objects, this);
    _set_state_service = _nhandle.advertiseService("set_state", &ROSTrainingServer::set_state, this);
    _box2d_world = std::dynamic_pointer_cast<sim_env::Box2DWorld>(_planning_problem.world);
    _box2d_robot = std::dynamic_pointer_cast<sim_env::Box2DRobot>(_planning_problem.robot);
    _logger = _box2d_world->getLogger();
}

ROSTrainingServer::~ROSTrainingServer()
{
    cleanup();
}

void ROSTrainingServer::cleanup()
{
    if (_control) {
        _planner.getControlSpace()->freeControl(_control);
    }
    if (_current_state) {
        _planner.getStateSpace()->freeState(_current_state);
    }
    if (_resulting_state) {
        _planner.getStateSpace()->freeState(_resulting_state);
    }
}

void ROSTrainingServer::setup_planner()
{
    cleanup();
    _planner.setup(_planning_problem);
    _control = dynamic_cast<mps::planner::ompl::control::RampVelocityControl*>(_planner.getControlSpace()->allocControl());
    _current_state = dynamic_cast<mps::planner::ompl::state::SimEnvWorldState*>(_planner.getStateSpace()->allocState());
    _resulting_state = dynamic_cast<mps::planner::ompl::state::SimEnvWorldState*>(_planner.getStateSpace()->allocState());
}

bool ROSTrainingServer::get_object_properties(planner_tests::GetObjectProperties::Request& req,
    planner_tests::GetObjectProperties::Response& res)
{
    const static std::string log_prefix("ROSTrainingServer::get_object_properties");
    std::lock_guard<std::recursive_mutex> lock(_box2d_world->getMutex());
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
    const static std::string log_prefix("ROSTrainingServer::get_action_space_info");
    _logger->logDebug("Retrieving action space information", log_prefix);
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
    const static std::string log_prefix("ROSTrainingServer::get_state");
    _logger->logDebug("Retrieving state", log_prefix);
    std::lock_guard<std::recursive_mutex> lock(_box2d_world->getMutex());
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
    const static std::string log_prefix("ROSTrainingServer::propagate");
    _logger->logDebug("Propagation request", log_prefix);
    res.service_success = false;
    if (req.action_params.size() == 4) {
        // ramp twist
        Eigen::Vector3f robot_pose;
        _box2d_robot->getPose(robot_pose);
        Eigen::Vector4f ltwist(req.action_params[0], req.action_params[1], req.action_params[2], req.action_params[3]);
        _control->setFromLocalTwist(robot_pose, ltwist);
    } else {
        _logger->logErr("Invalid propgation request. Invalid number of action params", log_prefix);
        // TODO propagation for car like robots
        return false;
    }
    _planner.getStateSpace()->extractState(_planning_problem.world, _current_state);
    res.valid_propagation = _planner.getStatePropagator()->propagate(_current_state, _control, _resulting_state);
    _planner.getStateSpace()->setToState(_planning_problem.world, _resulting_state);
    res.service_success = true;
    return true;
}

bool ROSTrainingServer::set_active_objects(planner_tests::SetActiveObjects::Request& req,
    planner_tests::SetActiveObjects::Response& res)
{
    const static std::string log_prefix("ROSTrainingServer::set_active_objects");
    _logger->logDebug("Setting active objects", log_prefix);
    if (req.obj_names.size() != req.active.size()) {
        res.service_success = false;
        return false;
    }
    for (size_t i = 0; i < req.obj_names.size(); ++i) {
        auto object = _box2d_world->getObject(req.obj_names[i], false);
        object->setEnabled(req.active[i]);
    }
    res.service_success = true;
    return true;
}

bool ROSTrainingServer::set_state(planner_tests::SetState::Request& req, planner_tests::SetState::Response& res)
{
    const static std::string log_prefix("ROSTrainingServer::set_state");
    std::lock_guard<std::recursive_mutex> lock(_box2d_world->getMutex());
    _box2d_world->saveState(); // save state in case we fail
    _logger->logDebug("Setting state", log_prefix);
    res.service_success = false;
    res.valid_state = false;
    Eigen::VectorXf dof_values(3);
    for (size_t i = 0; i < req.obj_names.size(); ++i) {
        auto obj = _box2d_world->getObject(req.obj_names[i], false);
        if (!obj) {
            _logger->logErr("Could not set state for object " + req.obj_names[i] + ". Object does not exist.", log_prefix);
            _box2d_world->restoreState();
            return false;
        }
        if (obj->getNumActiveDOFs() != 3) {
            _logger->logErr("Could not set state for object " + req.obj_names[i] + ". Object has invalid number of DOFs", log_prefix);
            _box2d_world->restoreState();
            return false;
        }
        dof_values[0] = req.states[i].x;
        dof_values[1] = req.states[i].y;
        dof_values[2] = req.states[i].theta;
        obj->setDOFPositions(dof_values);
    }
    res.service_success = true;
    auto validity_checker = _planner.getValidityChecker();
    res.valid_state = validity_checker->isWorldStateValid(false);
    _box2d_world->dropState(); // we succeeded, so we don't need the saved state anymore
    return true;
}
