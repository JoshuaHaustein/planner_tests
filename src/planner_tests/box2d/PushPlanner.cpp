//
// Created by joshua on 8/15/17.
//
#include <boost/program_options.hpp>
#include <sim_env/Box2DWorld.h>
#include <sim_env/Box2DWorldViewer.h>
#include <mps/planner/pushing/OraclePushPlanner.h>
#include <planner_tests/box2d/widget/PushPlannerWidget.h>

using namespace planner_tests::box2d;
namespace po = boost::program_options;

int main(int argc, const char* const* argv) {
    // declare program options
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "Show help message")
            ("environment", po::value<std::string>(), "environment to load");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }
    if (vm.count("environment")) {
        std::string env_path = vm["environment"].as<std::string>();
        std::cout << "Starting planner_test for environment " << env_path << std::endl;
        sim_env::Box2DWorldPtr world = std::make_shared<sim_env::Box2DWorld>();
        world->loadWorld(env_path);
        sim_env::LoggerPtr logger = world->getLogger();
        logger->setLevel(sim_env::Logger::LogLevel::Debug);
        logger->logInfo("Testing OraclePushPlanner using Box2DSimEnv");
        sim_env::Box2DWorldViewerPtr world_viewer = std::make_shared<sim_env::Box2DWorldViewer>(world);
        world_viewer->show(argc, argv);
        world_viewer->addCustomWidget(new widget::PushPlannerWidget(world, world_viewer), "PushPlanner");
        return world_viewer->run();
    } else {
        std::cout << "Please specify an environment to load" << std::endl;
        return 1;
    }
}
