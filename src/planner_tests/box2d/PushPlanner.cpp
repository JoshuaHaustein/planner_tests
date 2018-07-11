//
// Created by joshua on 8/15/17.
//
#include <boost/program_options.hpp>
#include <sim_env/Box2DWorld.h>
#include <sim_env/Box2DController.h>
#include <sim_env/Box2DWorldViewer.h>
#include <mps/planner/pushing/OraclePushPlanner.h>
#include <planner_tests/box2d/widget/PushPlannerWidget.h>
#include <mps/planner/util/yaml/OracleParsing.h>

using namespace planner_tests::box2d;
namespace po = boost::program_options;

void saveStats(const std::vector<mps::planner::pushing::algorithm::PlanningStatistics>& stats,
               const std::string& file_name) {
    std::ofstream of(file_name.c_str(),  std::ios_base::app);
    if (not stats.empty()) {
        stats.at(0).printCVSHeader(of);
    }
    for (auto& stat : stats) {
        stat.printCVS(of);
    }
    of.close();
}

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
        using namespace std::placeholders;
        robot->setController(std::bind(&sim_env::Box2DRobotVelocityController::control, controller, _1, _2, _3, _4, _5));
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

std::vector<mps::planner::pushing::algorithm::PlanningStatistics> evaluatePlanner(sim_env::Box2DWorldPtr world,
                                                                mps::planner::util::yaml::OraclePlanningProblemDesc& problem_desc,
                                                                unsigned int num_iter) {
    auto problem = setupPlanningProblem(world, problem_desc);
    mps::planner::pushing::OraclePushPlanner planner;
    std::vector<mps::planner::pushing::algorithm::PlanningStatistics> stats_vector;
    world->getLogger()->logInfo(boost::format("Planning problem setup. Running %i iterations") % num_iter,
            "[PushPlanner::runPlanner]");
    for (unsigned int i = 0; i < num_iter; ++i) {
        world->getLogger()->logInfo(boost::format("Running test %i/%i") % (i + 1) % num_iter, "[PushPlanner::runPlanner]");
        mps::planner::pushing::PlanningSolution sol;
        planner.setup(problem);
        planner.solve(sol);
        stats_vector.push_back(sol.stats);
    }
    world->getLogger()->logInfo("Planning evaluations completed.",
                                "[PushPlanner::runPlanner]");
    return stats_vector;
}

/**
 * Attempts to compute a reproducible solution to the given problem.
 * If it succeeds to do so, it stores the solution under the given filename, else
 * it returns false and does not store anything.
 */
bool computeValidSolution(sim_env::Box2DWorldPtr world,
                          mps::planner::util::yaml::OraclePlanningProblemDesc& problem_desc,
                          unsigned int num_iter,
                          const std::string& filename) 
{
    auto problem = setupPlanningProblem(world, problem_desc);
    mps::planner::pushing::OraclePushPlanner planner;
    world->getLogger()->logInfo(boost::format("Planning problem setup. Running %i iterations") % num_iter,
            "[PushPlanner::runPlanner]");
    for (unsigned int i = 0; i < num_iter; ++i) {
        mps::planner::pushing::PlanningSolution sol;
        planner.setup(problem);
        planner.solve(sol);
        bool do_shortcut = problem_desc.shortcut_type != mps::planner::pushing::PlanningProblem::ShortcutType::NoShortcut;
        if ((do_shortcut && sol.stats.reproducible_after_shortcut) || (sol.stats.reproducible))
        {
            // save the solution and quit
            return planner.saveSolution(sol, filename);
        }
    }
    return false;
}

int main(int argc, const char* const* argv) {
    const std::string log_prefix("[PushPlanner]");
    // declare program options
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "Show help message")
            ("environment", po::value<std::string>(), "environment to load")
            ("planning_problem", po::value<std::string>(), "planning problem to solve")
            ("verbose", po::bool_switch()->default_value(false), "if set, print debug outputs")
            ("output_file", po::value<std::string>(), "File to store stats in")
            ("record_file", po::value<std::string>()->default_value("/tmp/push_traj"), "File to store the recorded trajectory in")
            ("num_iterations", po::value<unsigned int>()->default_value(1), "number of planning iterations")
            ("record", po::bool_switch()->default_value(false), "If set and there is no GUI, the application runs"
                                                                 "the planner up till num_iterations times to produce a single "
                                                                 "reproducible solution. If it succeeds, it will store "
                                                                 "the solution in the file specified by the argument record_file."
                                                                 "If the GUI is set, it will store always the latest reproducible solution "
                                                                 "computed.")
            ("no-gui", po::bool_switch()->default_value(false), "if set, show no GUI");
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
    logger->logInfo("Testing OraclePushPlanner using Box2DSimEnv");
    // set logger level
    if (vm["verbose"].as<bool>()) {
        logger->setLevel(sim_env::Logger::LogLevel::Debug);
    } else {
        logger->setLevel(sim_env::Logger::LogLevel::Info);
    }
    // load planning problem if given
    mps::planner::util::yaml::OraclePlanningProblemDesc problem_desc;
    bool planning_problem_defined = false;
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
        planning_problem_defined = true;
    } else if (vm.count("environment")) { // just an environment
        std::string env_path = vm["environment"].as<std::string>();
        world->loadWorld(env_path);
    } else { // giving us nothing is bad
        logger->logErr("Please specify either a planning problem or an environment (in combination with using the GUI)",
                       log_prefix);
        return 1;
    }

    if (vm["no-gui"].as<bool>() and not planning_problem_defined) { // in case we have no gui, we need a full problem
        logger->logErr("Please specify a planning problem to solve", log_prefix);
        return 1;
    } else if (vm["no-gui"].as<bool>()) { // we have a planning problem, so plan
        if (vm["record"].as<bool>()) {
            bool success = computeValidSolution(world, problem_desc,
                                                vm["num_iterations"].as<unsigned int>(),
                                                vm["record_file"].as<std::string>());
            if (!success) {
                logger->logErr("Failed to compute a reproducible solution for the given problem", log_prefix);
            }
        } else {
            auto stats = evaluatePlanner(world, problem_desc, vm["num_iterations"].as<unsigned int>());
            if (vm.count("output_file")) {
                saveStats(stats, vm["output_file"].as<std::string>());
            }
        }
    } else { // we are showing the GUI
        // TODO pass record file path
        auto world_viewer = std::dynamic_pointer_cast<sim_env::Box2DWorldViewer>(world->getViewer());
        world_viewer->show(argc, argv);
        auto widget = new widget::PushPlannerWidget(world, world_viewer);
        if (planning_problem_defined) {
            widget->setPlanningProblem(problem_desc);
        }
        world_viewer->addCustomWidget(widget, "PushPlanner");
        return world_viewer->run();
    }
}
