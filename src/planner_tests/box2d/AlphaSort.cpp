//
// Created by joshua on 07/02/18.
//
#include <boost/program_options.hpp>
#include <sim_env/Box2DController.h>
#include <sim_env/Box2DWorld.h>
#include <sim_env/Box2DWorldViewer.h>
// #include <mps/planner/sorting/PushSortingPlanner.h>
#include <mps/planner/util/yaml/SortingParsing.h>
#include <planner_tests/box2d/widget/AlphaSortWidget.h>

using namespace planner_tests::box2d;
namespace po = boost::program_options;

unsigned int NUM_GROUP_COLORS = 6;
float GROUP_COLORS[6][3] = { { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 },
    { 0.5, 0.5, 0.0 }, { 0.0, 0.5, 0.5 }, { 0.5, 0.0, 0.5 } };

std::tuple<sim_env::WorldPtr, sim_env::Box2DRobotVelocityControllerPtr> loadWorld(const std::string& filename, sim_env::WorldPtr world)
{
    static const std::string log_prefix("[AlphaSort::loadWorld]");
    auto logger = world->getLogger();
    world->loadWorld(filename); // loads the environment

    // you can get access to objects in the world through their names, e.g.:
    // auto my_object = world->getObject("my_object");
    // if there is no object called my_object, the returned pointer is NULL
    // Check sim_env/SimEnv.h in sim_env for documentation.
    // The Box2DWorld implemented in box2d_sim_env may implement some additional
    // functions, which you should only use when setting up things. Inside your planner
    // you should only use the interface as defined in SimEnv.h, so that we could eventually
    // also replace Box2D with another physics models like Mujoco

    // Before we can plan motions for a robot, we need a controller for the robot.
    // So let's create one.
    // first get all robots from the world
    std::vector<sim_env::RobotPtr> robots;
    world->getRobots(robots);
    if (robots.empty()) {
        logger->logErr("The world contains no robot!", log_prefix);
        return std::make_tuple(nullptr, nullptr);
    } else if (robots.size() > 1) {
        logger->logWarn("The world contains more than one robot! Only creating a controller for the first!",
            log_prefix);
    }
    // The implementation of our controller is specific to Box2D, so it needs the Box2DRobot interface
    auto box2d_robot = std::dynamic_pointer_cast<sim_env::Box2DRobot>(robots[0]);
    auto velocity_controller = std::make_shared<sim_env::Box2DRobotVelocityController>(box2d_robot);
    using namespace std::placeholders;
    // this callback is used in the simulation to compute control commands for the robot
    sim_env::Robot::ControlCallback callback = std::bind(&sim_env::Box2DRobotVelocityController::control,
        velocity_controller,
        _1, _2, _3, _4, _5);
    box2d_robot->setController(callback);
    // if you do not set a controller, your robot won't move if you tell it to
    return std::make_tuple(world, velocity_controller);
}

// Assumes planning_problem.world is set, but nothing else yet
bool loadPlanningProblem(const std::string& filename, mps::planner::sorting::PlanningProblem& planning_problem)
{
    static const std::string log_prefix("[AlphaSort::loadPlanningProblem]");
    auto logger = planning_problem.world->getLogger();
    if (not boost::filesystem::exists(boost::filesystem::path(filename))) {
        logger->logErr(boost::format("Could not open the file %s because it does not exist.") % filename,
            log_prefix);
        return false;
    }
    // load yaml problem description
    YAML::Node node = YAML::LoadFile(filename);
    auto problem_desc = node.as<mps::planner::util::yaml::SortingPlanningProblemDesc>();
    // load environment
    boost::filesystem::path root_path(filename);
    root_path = root_path.parent_path();
    auto env_path = sim_env::resolveFileName(problem_desc.world_file, root_path); // env path relative to current dir
    std::tie(planning_problem.world, planning_problem.robot_controller) = loadWorld(env_path, planning_problem.world);
    if (planning_problem.world) { // if loading the world was successful
        planning_problem.robot = planning_problem.robot_controller->getRobot();
        bool init_success = planning_problem.init_robot();
        if (!init_success)
            return false;
        mps::planner::util::yaml::configurePlanningProblem(planning_problem, problem_desc);
        return true;
    }
    return false;
}

void set_colors(sim_env::Box2DWorldViewerPtr viewer, const mps::planner::sorting::PlanningProblem& problem)
{
    auto world_view = viewer->getWorldViewer();
    auto logger = problem.world->getLogger();
    // TODO reset colors
    for (auto& elem : problem.sorting_groups) {
        auto idx = elem.second % NUM_GROUP_COLORS;
        if (elem.second > NUM_GROUP_COLORS) {
            logger->logWarn("More groups than available colors."
                            " Some groups will have identical colors.",
                "[AlphaSortWidget::updateObjectColors]");
        }
        world_view->setColor(elem.first, GROUP_COLORS[idx][0], GROUP_COLORS[idx][1], GROUP_COLORS[idx][2]);
    }
}

int main(int argc, const char* const* argv)
{
    const std::string log_prefix("[AlphaSort]");
    // declare program options
    po::options_description desc("Allowed options");
    // You can set up command line arguments here. See boost program options for details
    desc.add_options()("help", "Show help message")("environment", po::value<std::string>(), "Environment to load.")("problem", po::value<std::string>(), "Problem to load (does not apply if environment is provided)")
        // ("my_argument", po::value<std::string>(), "Description of arument") // set up additional arguments like this
        ("verbose", po::bool_switch()->default_value(false), "if set, print debug outputs")("no-gui", po::bool_switch()->default_value(false), "if set, show no GUI");
    po::variables_map vm;
    // the following commands parse the provided command lines and save the arguments in the variable vm
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) { // print a help text
        std::cout << desc << std::endl;
        return 1;
    }
    // In order to be able to do anything, we need a world, so let's create an empty one
    mps::planner::sorting::PlanningProblem planning_problem;
    planning_problem.world = std::make_shared<sim_env::Box2DWorld>();
    // the world is equipped with a logger, which we can use to print messages
    sim_env::LoggerPtr logger = planning_problem.world->getLogger();
    logger->logInfo("Testing AlphaSort using Box2DSimEnv");
    // we can set different logger level
    if (vm["verbose"].as<bool>()) {
        // setting the level to debug, means the logger prints all messages
        logger->setLevel(sim_env::Logger::LogLevel::Debug);
    } else {
        // setting it to level info, means Debug messages are not printed
        logger->setLevel(sim_env::Logger::LogLevel::Info);
        // the other available levels are Warn and Error, see SimEnv.h::Logger
        // if you set it to Warn, earning and error messages are printed.
        // if you set it to Error only error messages are printed
    }
    if (vm.count("environment")) { // checks how many times the argument 'environment' is passed
        // if we have an environment, we should load it
        std::string env_path = vm["environment"].as<std::string>();
        std::tie(planning_problem.world, planning_problem.robot_controller) = loadWorld(env_path, planning_problem.world);
        if (!planning_problem.world)
            return 1;
        if (!planning_problem.init_robot()) {
            logger->logErr("Could not initialize robot settings of planning problem.", log_prefix);
            return 1;
        }
    } else if (vm.count("problem")) {
        std::string problem_path = vm["problem"].as<std::string>();
        bool loading_success = loadPlanningProblem(problem_path, planning_problem);
        if (!loading_success)
            return 1;
    } else { // if no environment or planning problem is provided, we print an error message and exit
        // the log prefix is a tag printed in front of the message
        logger->logErr("Please specify an environment or a full planning problem.", log_prefix);
        return 1;
    }

    // Independent of whether you actually wanna show the viewer, you need to initialize it
    // in order to be able to render (also to image)
    auto world_viewer = std::dynamic_pointer_cast<sim_env::Box2DWorldViewer>(planning_problem.world->getViewer());
    world_viewer->init(argc, argv);
    set_colors(world_viewer, planning_problem); // set the colors of objects
    if (vm["no-gui"].as<bool>()) { // in case we have no gui, we need a full problem
        // To evaluate the planner you will want to execute the algorithm without a GUI.
        // This would be done here. Running the planner with the GUI slows things down, so
        // you don't want to do this when you are evaluating the runtime of your algorithm.
        logger->logInfo("You started AlphaSort without a GUI. Nothing to do here, yet", log_prefix);
        world_viewer->renderImage("/tmp/no_viewer_image.png", 500, 200);
        std::vector<sim_env::ObjectPtr> objects;
        planning_problem.world->getObjects(objects);
        Eigen::VectorXf pos = objects[0]->getDOFPositions();
        pos[0] += 0.0;
        pos[1] += 0.0;
        pos[2] += 0.9;
        objects[0]->setDOFPositions(pos);
        world_viewer->renderImage("/tmp/no_viewer_image2.png", 500, 200);
        return 0;
    } else { // we want to show a GUI
        // let's get the WorldViewer that comes with Box2DSimEnv
        // the viewer is the window that launches when you execute this executable
        // for it to show up we need to call its show function
        world_viewer->show();
        // the viewer consists of a WorldView (note this is different from WorldViewer), and two panels.
        // The WorldView renders the Box2D world. The panel on the right of it shows the state
        // of a selected object in the world. It also allows you to set the state (position, velocity)
        // of an object. The panel on the bottim of the viewer is a control panel, that has multiple tabs.
        // The first two tabs allow you to turn the simulation of Box2D on and off. When Box2D is turned off,
        // you can move objects with your mouse, but the physical interaction between them is not modelled.
        // If you turn it on, Box2D models collisions between objects and moves them accordingly. You do not
        // need to turn it on for planning, this switch is only for the GUI and does not affect your planner in anyway.
        // The other tab allows you to try out a position and a velocity controller that is implemented in Box2DSimEnv.
        // Both currently only work properly for holonomic robots consisting of a single body, which is enough
        // for AlphaSort.
        // Additionally to these tabs, you can add your own widgets as additional tabs
        // For this, first create a new widget that inherits from QTabWidget (a Qt class).
        auto widget = new widget::AlphaSortWidget(planning_problem, world_viewer); // see widget/AlphaTestWidget.h
        // then add this widget to the GUI
        world_viewer->addCustomWidget(widget, "AlphaSort");
        // Finally, start the rendering thread of the viewer. This function loops and updates the GUI until
        // the user decides to close the program.
        return world_viewer->run();
    }
}
