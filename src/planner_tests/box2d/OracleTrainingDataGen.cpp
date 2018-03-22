//
// Created by joshua on 9/11/17.
//

#include <boost/program_options.hpp>
//#include <sim_env/Box2DWorldViewer.h>
#include <sim_env/Box2DWorld.h>
#include <sim_env/Box2DController.h>
#include <mps/planner/pushing/OraclePushPlanner.h>
#include <mps/planner/util/yaml/OracleParsing.h>
#include <thread>
#include <string>
#include <functional>

namespace po = boost::program_options;

struct DataGenerator {
    std::thread thread;
    mps::planner::util::yaml::OraclePlanningProblemDesc problem_desc;
    mps::planner::pushing::OraclePushPlanner planner;
    sim_env::Box2DWorldPtr world;
    std::string output_file;
    unsigned int num_samples;
    bool noisy_propagation;
    void run() {
        auto robot = world->getBox2DRobot(problem_desc.robot_name);
        auto target_object = world->getBox2DObject(problem_desc.training_object_name);
        std::stringstream header;
        auto aabb = target_object->getLocalAABB();
        header << aabb.getWidth() << ", " << aabb.getHeight()
               << ", " << target_object->getMass()
               << ", " << target_object->getInertia()
               << ", " << target_object->getGroundFriction()
               << ", " << target_object->getName();
        auto controller = std::make_shared<sim_env::Box2DRobotVelocityController>(robot);
        using namespace std::placeholders;
        robot->setController(std::bind(&sim_env::Box2DRobotVelocityController::control, controller, _1, _2, _3, _4, _5));
        mps::planner::ompl::state::goal::RelocationGoalSpecification goal_spec(problem_desc.training_object_name,
                                                                               Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                                                                               Eigen::Quaternionf(),
                                                                               0.0f,
                                                                               0.0f);
        mps::planner::pushing::PlanningProblem problem(world, robot, controller, goal_spec);
        mps::planner::util::yaml::configurePlanningProblem(problem, problem_desc);
        planner.setup(problem);
        planner.generateData(output_file, num_samples, header.str(), !noisy_propagation);
    }
};

struct RobotFilter {
    std::string name;
    bool filter(const sim_env::Box2DRobotDescription& robot_desc) {
        return robot_desc.object_description.name.compare(name) != 0;
    }
};

struct ObjectFilter {
    std::string name;
    bool filter(const sim_env::Box2DObjectDescription& obj_desc) {
        return obj_desc.name.compare(name) != 0;
    }
};

int main(int argc, const char* const* argv) {
    // declare program options
    bool verbose = false;
    bool noisy_propagation = false;
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "Show help message")
            ("output_file", po::value<std::string>(), "path to file where to write results to")
            ("planning_problem", po::value<std::string>(), "yaml file containing a planning problem definition")
            ("num_samples", po::value<unsigned int>()->default_value(1000), "number of samples")
            ("verbose", po::bool_switch(&verbose), "Set whether debug outputs should be printed")
            ("threads", po::value<unsigned int>()->default_value(1), "number of threads")
            ("noisy_propagation", po::bool_switch(&noisy_propagation), "If true, propagates each state-action sample multiple"
            " times with random noise on the dynamics and state");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }
    if (!vm.count("output_file")) {
        std::cout << "Please specify an output file" << std::endl;
        return 1;
    }
    if (!vm.count("planning_problem")) {
        std::cout << "Please specify the planning problem to generate data for." << std::endl;
        return 1;
    }
    if (noisy_propagation) {
        std::cout << "Data generation will add random noise on state and dynamics." << std::endl;
    }
    std::string planning_probl_file = vm["planning_problem"].as<std::string>();
    boost::filesystem::path root_path(planning_probl_file);
    if (not boost::filesystem::exists(root_path)) {
        std::cerr << "Could not open the file " << planning_probl_file << " because it does not exist." << std::endl;
        return 1;
    }
    root_path = root_path.parent_path();

    YAML::Node node = YAML::LoadFile(planning_probl_file);
    auto planning_desc = node.as<mps::planner::util::yaml::OraclePlanningProblemDesc>();

    sim_env::Box2DEnvironmentDescription env_desc;
    parseYAML(sim_env::resolveFileName(planning_desc.world_file, root_path), env_desc);
    // remove any object other than the robot and the target object from the scene
    RobotFilter robot_filter;
    robot_filter.name = planning_desc.robot_name;
    std::remove_if(env_desc.robots.begin(), env_desc.robots.end(),
                   std::bind(&RobotFilter::filter, &robot_filter, std::placeholders::_1));
    assert(env_desc.robots.size() == 1);
    // remove objects
    ObjectFilter obj_filter;
    obj_filter.name = planning_desc.training_object_name;
    std::remove_if(env_desc.objects.begin(), env_desc.objects.end(),
                   std::bind(&ObjectFilter::filter, &obj_filter, std::placeholders::_1));
    assert(env_desc.objects.size() == 1);
    // create workers
    std::string output_file = vm["output_file"].as<std::string>();
    unsigned int num_threads = std::max(vm["threads"].as<unsigned int>(), 1u);
    std::vector<DataGenerator> data_generators(num_threads);
    for (unsigned int t = 0; t < data_generators.size(); ++t) {
        std::cout << "Launching thread " << (t + 1) << "/" << num_threads << " for data generation" << std::endl;
        DataGenerator& data_generator = data_generators.at(t);
        data_generator.output_file = output_file + std::to_string(t);
        data_generator.num_samples = vm["num_samples"].as<unsigned int>();
        data_generator.problem_desc = planning_desc;
        data_generator.noisy_propagation = noisy_propagation;
        data_generator.world = std::make_shared<sim_env::Box2DWorld>();
        data_generator.world->loadWorld(env_desc);
        if (verbose) {
            data_generator.world->getLogger()->setLevel(sim_env::Logger::LogLevel::Debug);
        }
        data_generator.thread = std::thread(std::bind(&DataGenerator::run, std::ref(data_generator)));
    }
//    auto viewer = data_generators[0].world->getViewer();
//    auto box2d_viewer = std::dynamic_pointer_cast<sim_env::Box2DWorldViewer>(viewer);
//    box2d_viewer->show(argc, argv);
//    int a = box2d_viewer->run();
    for (auto& data_gen : data_generators) {
        data_gen.thread.join();
        std::cout << "Joined a thread" << std::endl;
    }
    std::cout << "All done" << std::endl;
    return 0;
}
