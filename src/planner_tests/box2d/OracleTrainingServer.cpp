//
// Created by joshua on 10/24/18.
//
#include "ros/ros.h"
#include <QThread>
#include <boost/program_options.hpp>
#include <mps/planner/pushing/OraclePushPlanner.h>
#include <mps/planner/util/yaml/OracleParsing.h>
#include <planner_tests/box2d/training_server/ROSTrainingServer.h>
#include <planner_tests/box2d/widget/PushPlannerWidget.h>
#include <sim_env/Box2DController.h>
#include <sim_env/Box2DWorld.h>
#include <sim_env/Box2DWorldViewer.h>

using namespace planner_tests::box2d;
namespace po = boost::program_options;

mps::planner::pushing::PlanningProblem setupPlanningProblem(
    sim_env::Box2DWorldPtr world,
    mps::planner::util::yaml::OraclePlanningProblemDesc& problem_desc)
{
    // first set up controller for the robot
    auto robot = world->getBox2DRobot(problem_desc.robot_name);
    if (!robot) {
        throw std::runtime_error("Could not find robot " + problem_desc.robot_name);
    }
    auto controller = std::make_shared<sim_env::Box2DRobotVelocityController>(robot);
    {
        robot->setController(std::bind(&sim_env::Box2DRobotVelocityController::control, controller,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));
    }
    // prepare goal specifications
    std::vector<mps::planner::ompl::state::goal::RelocationGoalSpecification> goal_specs;
    for (auto& goal_desc : problem_desc.goals) {
        mps::planner::ompl::state::goal::RelocationGoalSpecification goal_spec(goal_desc.obj_name,
            goal_desc.goal_position,
            Eigen::Quaternionf(),
            goal_desc.goal_region_radius,
            0.0f);
        goal_specs.push_back(goal_spec);
    }
    // create planning problem
    mps::planner::pushing::PlanningProblem problem(world, robot, controller, goal_specs);
    mps::planner::util::yaml::configurePlanningProblem(problem, problem_desc);
    return problem;
}

class QtROSSpinner : public QThread {
    void run()
    {
        ros::spin();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "oracle_training_server", ros::init_options::NoSigintHandler);
    const std::string log_prefix("[OracleTrainingServer]");
    // declare program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()("help", "Show help message")
                      ("planning_problem", po::value<std::string>(), "planning problem to create data for")
                      ("verbose", po::bool_switch()->default_value(false), "if set, print debug outputs")
                      ("no-gui", po::bool_switch()->default_value(false), "if set, show no GUI");
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }
    // create an empty world
    sim_env::Box2DWorldPtr world = std::make_shared<sim_env::Box2DWorld>();
    sim_env::LoggerPtr logger = world->getLogger();
    logger->logInfo("Launching OracleTrainingServer using Box2D");
    // set logger level
    if (vm["verbose"].as<bool>()) {
        logger->setLevel(sim_env::Logger::LogLevel::Debug);
    } else {
        logger->setLevel(sim_env::Logger::LogLevel::Info);
    }
    // load planning problem
    mps::planner::util::yaml::OraclePlanningProblemDesc problem_desc;
    // check what kind of arguments we have
    if (vm.count("planning_problem")) { // we have a planning problem
        std::string planning_probl_file = vm["planning_problem"].as<std::string>();
        boost::filesystem::path root_path(planning_probl_file);
        if (not boost::filesystem::exists(root_path)) {
            logger->logErr(boost::format("Could not open the file %s because it does not exist.") % planning_probl_file,
                log_prefix);
            return 1;
        }
        root_path = root_path.parent_path();
        YAML::Node node = YAML::LoadFile(planning_probl_file);
        problem_desc = node.as<mps::planner::util::yaml::OraclePlanningProblemDesc>();
        world->loadWorld(sim_env::resolveFileName(problem_desc.world_file, root_path));
    } else { // giving us nothing is bad
        logger->logErr("Please specify a planning problem", log_prefix);
        return 1;
    }

    auto pp_problem = setupPlanningProblem(world, problem_desc);
    planner_tests::box2d::training_server::ROSTrainingServer training_server(pp_problem, "/oracle_training_server/");
    training_server.setup_planner();
    int return_val = 0;
    if (vm["no-gui"].as<bool>()) {
        ros::spin();
    } else { // we have a GUI, so show it
        auto world_viewer = std::dynamic_pointer_cast<sim_env::Box2DWorldViewer>(world->getViewer());
        world_viewer->init(argc, argv);
        world_viewer->show();
        auto widget = new widget::PushPlannerWidget(world, world_viewer);
        widget->setPlanningProblem(problem_desc);
        world_viewer->addCustomWidget(widget, "OracleTrainingServer");
        QtROSSpinner ros_spinner; // run a separate thread to update ROS stuff
        ros_spinner.start();
        return_val = world_viewer->run();
        ros_spinner.exit();
    }
    return return_val;
}
