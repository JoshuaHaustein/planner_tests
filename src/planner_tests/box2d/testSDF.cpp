#include <sim_env/Box2DWorld.h>
#include <mps/sdf/SDF.h>

int main(int argc, const char* const* argv) {
    sim_env::Box2DWorldPtr world = std::make_shared<sim_env::Box2DWorld>();
    world->loadWorld("/home/joshua/projects/planning_catkin/src/planner_tests/data/box2d/worlds/slalom_world_1m_2s.yaml");
    mps::sdf::SceneSDF scene_sdf(world);
    sim_env::BoundingBox aabb;
    aabb.min_corner[0] = -1.0f;
    aabb.min_corner[1] = -1.0f;
    aabb.max_corner[0] = 1.0f;
    aabb.max_corner[1] = 1.0f;
    scene_sdf.computeSDF(aabb);
    return 0;
}