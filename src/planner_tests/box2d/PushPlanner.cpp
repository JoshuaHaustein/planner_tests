//
// Created by joshua on 8/15/17.
//
#include <sim_env/Box2DWorld.h>
#include <sim_env/Box2DWorldViewer.h>
#include <mps/planner/pushing/OraclePushPlanner.h>
#include <planner_tests/box2d/widget/PushPlannerWidget.h>

using namespace planner_tests::box2d;

int main(int argc, char **argv) {
    sim_env::Box2DWorldPtr world = std::make_shared<sim_env::Box2DWorld>();
    world->loadWorld("/home/joshua/projects/planning_catkin/src/box2d_sim_env/test_data/test_env.yaml");
    sim_env::LoggerPtr logger = world->getLogger();
    logger->logInfo("It works!");
    sim_env::Box2DWorldViewerPtr world_viewer = std::make_shared<sim_env::Box2DWorldViewer>(world);
    world_viewer->show(argc, argv);
    world_viewer->addCustomWidget(new widget::PushPlannerWidget(world), "PushPlanner");
    return world_viewer->run();
}
