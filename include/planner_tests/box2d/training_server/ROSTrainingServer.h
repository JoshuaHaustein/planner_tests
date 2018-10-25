//
// Created by joshua on 10/24/18.
//

#ifndef ROS_TRAINING_SERVER_H
#define ROS_TRAINING_SERVER_H

#include "ros/ros.h"
#include <planner_tests/GetActionSpaceInfo.h>
#include <planner_tests/GetObjectProperties.h>
#include <planner_tests/GetState.h>
#include <planner_tests/Propagate.h>
#include <planner_tests/SetActiveObjects.h>
#include <planner_tests/SetState.h>
#include <sim_env/Box2DController.h>
#include <sim_env/Box2DWorld.h>

#include <mps/planner/pushing/OraclePushPlanner.h>

namespace planner_tests {
namespace box2d {
    namespace training_server {

        class ROSTrainingServer {
        public:
            ROSTrainingServer(mps::planner::pushing::PlanningProblem& planning_problem, const std::string& ros_ns);
            ~ROSTrainingServer();

            void setup_planner();

            bool get_object_properties(planner_tests::GetObjectProperties::Request& req,
                planner_tests::GetObjectProperties::Response& res);

            bool get_action_space_info(planner_tests::GetActionSpaceInfo::Request& req,
                planner_tests::GetActionSpaceInfo::Response& res);

            bool get_state(planner_tests::GetState::Request& req,
                planner_tests::GetState::Response& res);

            bool propagate(planner_tests::Propagate::Request& req, planner_tests::Propagate::Response& res);

            bool set_active_objects(planner_tests::SetActiveObjects::Request& req,
                planner_tests::SetActiveObjects::Response& res);

            bool set_state(planner_tests::SetState::Request& req, planner_tests::SetState::Response& res);

        private:
            ros::NodeHandle _nhandle;
            mps::planner::pushing::PlanningProblem _planning_problem;
            sim_env::Box2DWorldPtr _box2d_world;
            sim_env::Box2DRobotPtr _box2d_robot;
            sim_env::LoggerPtr _logger;
            mps::planner::pushing::OraclePushPlanner _planner;
            mps::planner::ompl::control::RampVelocityControl* _control;
            mps::planner::ompl::state::SimEnvWorldState* _current_state;
            mps::planner::ompl::state::SimEnvWorldState* _resulting_state;

            ros::ServiceServer _get_properties_service;
            ros::ServiceServer _get_action_space_info_service;
            ros::ServiceServer _get_state_service;
            ros::ServiceServer _propagate_service;
            ros::ServiceServer _set_active_objects_service;
            ros::ServiceServer _set_state_service;

            void cleanup();
        };
    }
}
}
#endif // ROS_TRAINING_SERVER_H
