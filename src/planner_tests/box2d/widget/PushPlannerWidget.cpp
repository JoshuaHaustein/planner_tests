//
// Created by joshua on 8/22/17.
//

#include <planner_tests/box2d/widget/PushPlannerWidget.h>
#include <QtGui/QGridLayout>
#include <QtGui/QLabel>
#include <sim_env/Box2DController.h>

using namespace planner_tests::box2d::widget;

PushPlannerWidget::PushPlannerWidget(sim_env::Box2DWorldPtr world, QWidget* parent):
        QGroupBox("PushPlannerWidget", parent)
{
    _weak_world = world;
    buildUI();
    synchUI();
}

PushPlannerWidget::~PushPlannerWidget() {
}

void PushPlannerWidget::button_clicked(bool enabled) {
    auto world = lockWorld();
    if (enabled) {
        world->getLogger()->logInfo("start planner");
        // TODO it doesn't make much sense to always create new controllers -> see COntrollerWidget
        _robot_controller.reset();
        sim_env::Box2DRobotPtr robot = world->getBox2DRobot(_robot_selector->currentText().toStdString());
        sim_env::Box2DObjectPtr target = world->getBox2DObject(_target_selector->currentText().toStdString());
        _robot_controller = std::make_shared<sim_env::Box2DRobotVelocityController>(robot);
        mps::planner::pushing::PlanningProblem planning_problem(world, robot, _robot_controller, target);
        _planner.setup(planning_problem);
        // TODO debug
        _planner.dummyTest();
//        _planner.solve()

    } else {
        world->getLogger()->logInfo("stop planner");
    }
}

void PushPlannerWidget::buildUI() {
    QGridLayout *layout = new QGridLayout();
    // create robot selector
    QLabel* label = new QLabel("Robot");
    _robot_selector = new QComboBox();
    layout->addWidget(label, 0, 0);
    layout->addWidget(_robot_selector, 0, 1);
    // create target selector
    label = new QLabel("Target");
    _target_selector = new QComboBox();
    layout->addWidget(label, 1, 0);
    layout->addWidget(_target_selector, 1, 1);
    // create time out edit
    label = new QLabel("Time out");
    _time_out_edit = new QLineEdit();
    _time_out_edit->setText("60.0");
    layout->addWidget(label, 2, 0);
    layout->addWidget(_time_out_edit, 2, 1);
    // create semi-dynamic tick box
    _semi_dynamic_check_box = new QCheckBox();
    _semi_dynamic_check_box->setText("Semi-dynamic");
    _semi_dynamic_check_box->setChecked(true);
    layout->addWidget(_semi_dynamic_check_box, 3, 0, 1, 2);
    // t_max line edit
    label = new QLabel("T_max");
    _t_max_edit = new QLineEdit();
    _t_max_edit->setText("8.0");
    layout->addWidget(label, 4, 0);
    layout->addWidget(_t_max_edit, 4, 1);
    // start button
    _start_button = new QPushButton();
    _start_button->setText("Start planner");
    _start_button->setCheckable(true);
    layout->addWidget(_start_button, 5, 0, 1, 2);
    QObject::connect(_start_button, SIGNAL(clicked(bool)),
                     this, SLOT(button_clicked(bool)));
    // finally set the layout
    setLayout(layout);
}

void PushPlannerWidget::synchUI() {
    sim_env::Box2DWorldPtr world = lockWorld();
    // set selectable robots
    std::vector<sim_env::RobotPtr> robots;
    world->getRobots(robots);
    _robot_selector->clear();
    for (auto robot : robots) {
        _robot_selector->addItem(robot->getName().c_str());
    }
    // set selectable target objects
    std::vector<sim_env::ObjectPtr> objects;
    world->getObjects(objects);
    _target_selector->clear();
    for (auto object : objects) {
        _target_selector->addItem(object->getName().c_str());
    }
}

sim_env::Box2DWorldPtr PushPlannerWidget::lockWorld() {
    sim_env::Box2DWorldPtr world = _weak_world.lock();
    if (!world) {
        throw std::logic_error("[planner_tests::box2d::widget::lockWorld] Could not access Box2dWorld.");
    }
    return world;
}
