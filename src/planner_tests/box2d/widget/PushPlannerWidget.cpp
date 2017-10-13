//
// Created by joshua on 8/22/17.
//

#include <planner_tests/box2d/widget/PushPlannerWidget.h>
#include <planner_tests/box2d/widget/SliceWidget.h>
#include <QtGui/QGridLayout>
#include <QtGui/QLabel>
#include <sim_env/Box2DController.h>
#include <mps/planner/util/Logging.h>

using namespace planner_tests::box2d::widget;

void PushPlannerWidget::PlannerThread::plan() {
    planner.solve(solution);
}

void PushPlannerWidget::PlannerThread::playback() {
    if (solution.solved) {
        planner.playback(solution, std::bind(&PushPlannerWidget::PlannerThread::isInterrupted, this));
    }
}

bool PushPlannerWidget::PlannerThread::isInterrupted() {
    return interrrupt;
}

PushPlannerWidget::PushPlannerWidget(sim_env::Box2DWorldPtr world,
                                     sim_env::Box2DWorldViewerPtr viewer,
                                     QWidget* parent):
        QGroupBox("PushPlannerWidget", parent)
{
    _weak_world = world;
    _weak_viewer = viewer;
    buildUI();
    synchUI();
    auto slice_widget = new widget::SliceWidget(world);
    viewer->addCustomWidget(slice_widget, "Slice Debug Widget");
    _slice_drawer = slice_widget->getSliceDrawer();
}

PushPlannerWidget::~PushPlannerWidget() = default;

void PushPlannerWidget::setPlanningProblem(mps::planner::util::yaml::OraclePlanningProblemDesc &desc) {
    static const std::string log_prefix("[planner_tests""box2d""widget::PushPlannerWidget::setPlanningProblem]");
    auto world = _weak_world.lock();
    if (!world) {
        throw std::logic_error("Could not access underlying world!");
    }
    auto logger = world->getLogger();
    // set robot
    int combo_idx = _robot_selector->findText(QString(desc.robot_name.c_str()));
    if (combo_idx == -1) {
        logger->logErr("Could not find robot from problem description", log_prefix);
    } else {
        _robot_selector->setCurrentIndex(combo_idx);
    }
    // set target
    combo_idx = _target_selector->findText(QString(desc.target_name.c_str()));
    if (combo_idx == -1) {
        logger->logErr("Could not find target from problem description", log_prefix);
    } else {
        _target_selector->setCurrentIndex(combo_idx);
    }
    // set oracle
    std::string oracle_name = mps::planner::util::yaml::oracleTypeToString(desc.oracle_type);
    combo_idx = _oracle_selector->findText(QString(oracle_name.c_str()));
    if (combo_idx == -1) {
        logger->logErr("Could not find oracle from problem description", log_prefix);
    } else {
        _oracle_selector->setCurrentIndex(combo_idx);
    }
    // set algorithm
    std::string algorithm_name = mps::planner::util::yaml::algorithmTypeToString(desc.algorithm_type);
    combo_idx = _algorithm_selector->findText(QString(algorithm_name.c_str()));
    if (combo_idx == -1) {
        logger->logErr("Could not find algorithm from problem description", log_prefix);
    } else {
        _algorithm_selector->setCurrentIndex(combo_idx);
    }
    // set world bounds
    setValue(_min_x_workbounds, desc.x_limits[0]);
    setValue(_max_x_workbounds, desc.x_limits[1]);
    setValue(_min_y_workbounds, desc.y_limits[0]);
    setValue(_max_y_workbounds, desc.y_limits[1]);
    // set control limits
    setValue(_max_trans_vel, desc.control_limits.velocity_limits[0]);
    setValue(_max_rot_vel, desc.control_limits.velocity_limits[2]);
    setValue(_min_action_duration, desc.control_limits.duration_limits[0]);
    setValue(_max_action_duration, desc.control_limits.duration_limits[1]);
    // set planner parameters
    setValue(_time_out_edit, desc.planning_timeout);
    setValue(_num_control_samples, desc.num_control_samples); // only for naive version
    setValue(_t_max_edit, desc.t_max);
    // set goal region
    setValue(_goal_x, desc.goal_position[0]);
    setValue(_goal_y, desc.goal_position[1]);
    setValue(_goal_radius, desc.goal_region_radius);
    // set biases
    setValue(_goal_bias, desc.goal_bias);
    setValue(_target_bias, desc.target_bias);
    setValue(_robot_bias, desc.robot_bias);
    // set collision policy
    _static_col_allowed->setChecked(desc.collision_policy.static_collisions_allowed);
    for (auto& obj_name : desc.collision_policy.static_collisions_blacklist) {
        QString new_text;
        new_text.fromStdString(obj_name);
        new_text.prepend(',');
        _static_col_blacklist->text().append(new_text);
    }
    // TODO also provide object-object collision pairs

}

void PushPlannerWidget::button_clicked(bool enabled) {
    static std::string log_prefix("[planner_tests::box2d::widget::PushPlannerWidget::button_clicked]");
    auto world = lockWorld();
    auto* button_sender = dynamic_cast<QPushButton*>(QObject::sender());
    if (!button_sender) {
        world->getLogger()->logErr("Could not identify send of button_clicked signal.", log_prefix);
        return;
    }
    if (enabled) {
        if (button_sender == _start_button) {
            startPlanner();
        } else if (button_sender == _play_back_button) {
            startPlayback();
        }
    } else {
        stopPlannerThread();
    }
}

void PushPlannerWidget::buildUI() {
    auto *layout = new QGridLayout();
    // create robot selector
    ////////////////////////////////////////////////////////////
    ////////////////////// First column ////////////////////////
    unsigned int row = 0;
    unsigned int col = 0;
    QLabel* label = new QLabel("Robot");
    _robot_selector = new QComboBox();
    layout->addWidget(label, row, col);
    layout->addWidget(_robot_selector, row, col + 1);
    ++row;
    // create target selector
    label = new QLabel("Target");
    _target_selector = new QComboBox();
    layout->addWidget(label, row, col);
    layout->addWidget(_target_selector, row, col + 1);
    ++row;
    // create time out edit
    label = new QLabel("Time out");
    _time_out_edit = new QLineEdit();
    _time_out_edit->setText("60.0");
    layout->addWidget(label, row, col);
    layout->addWidget(_time_out_edit, row, col + 1);
    ++row;
    // create semi-dynamic tick box
    _semi_dynamic_check_box = new QCheckBox();
    _semi_dynamic_check_box->setText("Semi-dynamic");
    _semi_dynamic_check_box->setChecked(true);
    layout->addWidget(_semi_dynamic_check_box, row, col, 1, 1);
    // create debug tick box
    _debug_check_box = new QCheckBox();
    _debug_check_box->setText("Debug");
    _debug_check_box->setChecked(true);
    layout->addWidget(_debug_check_box, row, col + 1, 1, 1);
    ++row;
    // t_max line edit
    label = new QLabel("T_max");
    _t_max_edit = new QLineEdit();
    _t_max_edit->setText("8.0");
    layout->addWidget(label, row, col);
    layout->addWidget(_t_max_edit, row, col + 1);
    ++row;
    // num_control_samples
    label = new QLabel("Num control samples (k)");
    _num_control_samples = new QLineEdit();
    _num_control_samples->setText("10");
    layout->addWidget(label, row, col);
    layout->addWidget(_num_control_samples, row, col + 1);
    ////////////////////////////////////////////////////////////
    ////////////////////// Second column (2 - 4) ///////////////
    row = 0;
    col = 2;
    // goal settings
    label = new QLabel("Goal region (x, y):");
    layout->addWidget(label, row, col);
    _goal_x = new QLineEdit(QString("1.0"));
    layout->addWidget(_goal_x, row, col + 1);
    _goal_y = new QLineEdit(QString("1.0"));
    layout->addWidget(_goal_y, row, col + 2);
    ++row;
    // Goal radius and bias
    label = new QLabel("Goal radius and bias");
    layout->addWidget(label, row, col);
    _goal_radius = new QLineEdit(QString("0.1"));
    layout->addWidget(_goal_radius, row, col + 1);
    _goal_bias = new QLineEdit(QString("0.1"));
    layout->addWidget(_goal_bias, row, col + 2);
    ++row;
    // Workspace bounds for x
    label = new QLabel("Workspace bounds x (min, max):");
    layout->addWidget(label, row, col);
    _min_x_workbounds = new QLineEdit(QString("-2.0"));
    layout->addWidget(_min_x_workbounds, row, col + 1);
    _max_x_workbounds = new QLineEdit(QString("2.0"));
    layout->addWidget(_max_x_workbounds, row, col + 2);
    ++row;
    // Workspace bounds for y
    label = new QLabel("Workspace bounds y (min, max):");
    layout->addWidget(label, row, col);
    _min_y_workbounds = new QLineEdit(QString("-2.0"));
    layout->addWidget(_min_y_workbounds, row, col + 1);
    _max_y_workbounds = new QLineEdit(QString("2.0"));
    layout->addWidget(_max_y_workbounds, row, col + 2);
    ++row;
    // Control velocity limits
    label = new QLabel("Control velocity limits (trans, rot)");
    layout->addWidget(label, row, col);
    _max_trans_vel = new QLineEdit(QString("0.6"));
    layout->addWidget(_max_trans_vel, row, col + 1);
    _max_rot_vel = new QLineEdit(QString("1.4"));
    layout->addWidget(_max_rot_vel, row, col + 2);
    ++row;
    // Action duration
    label = new QLabel("Action duration (min, max):");
    layout->addWidget(label, row, col);
    _min_action_duration = new QLineEdit(QString("0.05"));
    layout->addWidget(_min_action_duration, row, col + 1);
    _max_action_duration = new QLineEdit(QString("1.0"));
    layout->addWidget(_max_action_duration, row, col + 2);
    ++row;
    // Oracle and algorithm selector
    label = new QLabel("Algorithm and oracle type");
    layout->addWidget(label, row, col);
    _algorithm_selector = new QComboBox();
    _algorithm_selector->addItem("Naive", mps::planner::pushing::PlanningProblem::AlgorithmType::Naive);
    _algorithm_selector->addItem("OracleRRT", mps::planner::pushing::PlanningProblem::AlgorithmType::OracleRRT);
    _algorithm_selector->addItem("SliceOracleRRT", mps::planner::pushing::PlanningProblem::AlgorithmType::SliceOracleRRT);
    _algorithm_selector->addItem("CompleteSliceOracleRRT", mps::planner::pushing::PlanningProblem::AlgorithmType::CompleteSliceOracleRRT);
    layout->addWidget(_algorithm_selector, row, col + 1);
    _oracle_selector = new QComboBox();
    _oracle_selector->addItem("Human", mps::planner::pushing::PlanningProblem::OracleType::Human);
    _oracle_selector->addItem("Learned", mps::planner::pushing::PlanningProblem::OracleType::Learned);
    layout->addWidget(_oracle_selector, row, col + 2);
    ///////////////////////////// Columns (5, 6 and 7) //////////////////////
    col = 5;
    row = 0;
    // Goal and robot bias
    label = new QLabel(QString("Target and robot bias: "));
    layout->addWidget(label, row, col);
    _target_bias = new QLineEdit(QString("0.1"));
    layout->addWidget(_target_bias, row, col + 1);
    _robot_bias = new QLineEdit(QString("0.1"));
    layout->addWidget(_robot_bias, row, col + 2);
    ++row;
    label = new QLabel(QString("Static collisions blacklist"));
    layout->addWidget(label, row, col);
    _static_col_allowed = new QCheckBox(QString("Allow static col"));
    layout->addWidget(_static_col_allowed, row, col+1);
    _static_col_blacklist = new QLineEdit("robot");
    layout->addWidget(_static_col_blacklist, row, col + 2);
    ////////////////////////////////////////////////////////////
    ////////////////////// Bottom button ///////////////////////
    // start button
    _start_button = new QPushButton();
    _start_button->setText("Start planner");
    _start_button->setCheckable(true);
    layout->addWidget(_start_button, 6, 0);
    QObject::connect(_start_button, SIGNAL(clicked(bool)),
                     this, SLOT(button_clicked(bool)));
    // play back button
    _play_back_button = new QPushButton();
    _play_back_button->setText("Playback last solution");
    _play_back_button->setCheckable(true);
    layout->addWidget(_play_back_button, 6, 1);
    QObject::connect(_play_back_button, SIGNAL(clicked(bool)),
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
    world->getObjects(objects, true);
    _target_selector->clear();
    for (auto object : objects) {
        if (not object->isStatic()) {
            _target_selector->addItem(object->getName().c_str());
        }
    }
}

void PushPlannerWidget::configurePlanningProblem(mps::planner::pushing::PlanningProblem& pp) {
    // goal
    pp.goal_position[0] = readValue(_goal_x, 1.0);
    pp.goal_position[1] = readValue(_goal_y, 1.0);
    pp.goal_region_radius = readValue(_goal_radius, 0.1);
    // workspace bounds
    pp.workspace_bounds.x_limits[0] = readValue(_min_x_workbounds, pp.workspace_bounds.x_limits[0]);
    pp.workspace_bounds.x_limits[1] = readValue(_max_x_workbounds, pp.workspace_bounds.x_limits[1]);
    pp.workspace_bounds.y_limits[0] = readValue(_min_y_workbounds, pp.workspace_bounds.y_limits[0]);
    pp.workspace_bounds.y_limits[1] = readValue(_max_y_workbounds, pp.workspace_bounds.y_limits[1]);
    pp.workspace_bounds.z_limits[0] = 0.0f;
    pp.workspace_bounds.z_limits[1] = 0.0f;
    // time out
    pp.planning_time_out = readValue(_time_out_edit, 60.0f);
    // biases
    pp.robot_bias = readValue(_robot_bias, 0.1f);
    pp.target_bias = readValue(_target_bias, 0.1f);
    pp.goal_bias = readValue(_goal_bias, 0.1f);
    // control bounds
    auto vel_limits = pp.robot->getDOFVelocityLimits();
    pp.control_limits.velocity_limits.resize(vel_limits.rows());
    assert(vel_limits.rows() >= 2);
    pp.control_limits.velocity_limits[0] = readValue(_max_trans_vel, vel_limits(0, 1));
    pp.control_limits.velocity_limits[1] = readValue(_max_trans_vel, vel_limits(1, 1));
    pp.control_limits.velocity_limits[2] = readValue(_max_rot_vel, vel_limits(2, 1));
    // TODO set limits for joint dofs
    pp.control_limits.duration_limits[0] = readValue(_min_action_duration, 0.01f);
    pp.control_limits.duration_limits[1] = readValue(_max_action_duration, 2.5f);
    // control subspace
    Eigen::VectorXi translational_dofs(2);
    translational_dofs.setLinSpaced(0, 1);
    Eigen::Array2f velocity_norm_limits;
    velocity_norm_limits[0] = 0.0f;
    velocity_norm_limits[1] = pp.control_limits.velocity_limits[0];
    mps::planner::ompl::control::RampVelocityControlSpace::ControlSubspace base_motion_subspace(translational_dofs, velocity_norm_limits);
    pp.control_subspaces.push_back(base_motion_subspace);
    // stopping condition
    pp.stopping_condition = std::bind(&PlannerThread::isInterrupted, std::ref(_planner_thread));
    // debug
    pp.debug = _debug_check_box->isChecked();
    // settings control samples
    int value = readIntValue(_num_control_samples, 10);
    pp.num_control_samples = (unsigned int) (value > 0 ? value : 10);
    // oracle type
    bool ok = true;
    int enum_value = _oracle_selector->itemData(_oracle_selector->currentIndex()).toInt(&ok);
    if (not ok) {
        enum_value = mps::planner::pushing::PlanningProblem::OracleType::Human;
    }
    pp.oracle_type = mps::planner::pushing::PlanningProblem::OracleType(enum_value);
    enum_value = _algorithm_selector->itemData(_algorithm_selector->currentIndex()).toInt(&ok);
    if (not ok) {
        enum_value = mps::planner::pushing::PlanningProblem::AlgorithmType::Naive;
    }
    pp.algorithm_type = mps::planner::pushing::PlanningProblem::AlgorithmType(enum_value);
    // collision policy
    pp.collision_policy.setStaticCollisions(_static_col_allowed->isChecked());
    // parse black list
    auto blacklist = _static_col_blacklist->text().split(',');
    for (auto& obj_name : blacklist) {
        auto object = pp.world->getObject(obj_name.toStdString(), false);
        if (object) {
            pp.collision_policy.setStaticCollisions(obj_name.toStdString(), false);
        } else {
            pp.world->getLogger()->logErr("Could not add an object with name " + obj_name.toStdString() +
                                          " to the collision policy blacklist. The object does not exist.", "[PushPlannerWidget]");
        }
    }
}

sim_env::Box2DWorldPtr PushPlannerWidget::lockWorld() {
    sim_env::Box2DWorldPtr world = _weak_world.lock();
    if (!world) {
        throw std::logic_error("[planner_tests::box2d::widget::lockWorld] Could not access Box2dWorld.");
    }
    return world;
}

void PushPlannerWidget::visualizePlanningProblem(const mps::planner::pushing::PlanningProblem& problem)
{
    sim_env::Box2DWorldViewerPtr viewer = _weak_viewer.lock();
    if (!viewer) {
        throw std::logic_error("[planner_tests::box2d::widget::PushPlannerWidget] Could not access viewer.");
    }
    // first clear any previous drawings
    for (auto& handle : _drawing_handles) {
        viewer->removeDrawing(handle);
    }
    // now visualize properties of the planning problem
    // 1. show planning bounds
    Eigen::Vector3f pos;
    pos[0] = problem.workspace_bounds.x_limits[0];
    pos[1] = problem.workspace_bounds.y_limits[0];
    Eigen::Vector3f extents;
    extents[0] = problem.workspace_bounds.x_limits[1] - problem.workspace_bounds.x_limits[0];
    extents[1] = problem.workspace_bounds.y_limits[1] - problem.workspace_bounds.y_limits[0];
    _drawing_handles.emplace_back(viewer->drawBox(pos, extents));
    // 2. show goal region
    _drawing_handles.emplace_back(viewer->drawSphere(problem.goal_position,
                                                     problem.goal_region_radius,
                                                     Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f),
                                                     0.01f));
    std::vector<sim_env::ObjectPtr> objects;
    problem.world->getObjects(objects, true);
    for (auto& obj : objects) {
        auto box2d_object = std::dynamic_pointer_cast<sim_env::Box2DObject>(obj);
        if (obj != problem.target_object) {
            viewer->getWorldViewer()->resetColor(box2d_object->getName());
        } else {
            viewer->getWorldViewer()->setColor(box2d_object->getName(), 0.0f, 1.0f, 0.0);
        }
    }
    // TODO whatever else to show
}

float PushPlannerWidget::readValue(QLineEdit* text_field, float default_value) {
    bool ok = false;
    float value = text_field->text().toFloat(&ok);
    if (ok) {
        return value;
    }
    return default_value;
}

void PushPlannerWidget::setValue(QLineEdit* text_field, float value) {
    QString q_string;
    q_string.setNum(value);
    text_field->setText(q_string);
}

int PushPlannerWidget::readIntValue(QLineEdit* text_field, int default_value) {
    bool ok = false;
    int value = text_field->text().toInt(&ok);
    if (ok) {
        return value;
    }
    return default_value;
}

void PushPlannerWidget::startPlanner() {
    static std::string log_prefix("[planner_tests::box2d::widget::PushPlannerWidget::startPlanner]");
    auto world = lockWorld();
    if (not _planner_thread.thread.joinable()) {
        world->getLogger()->logInfo("Starting planner", log_prefix);
        mps::planner::util::logging::setLogger(world->getLogger());
        sim_env::Box2DRobotPtr robot = world->getBox2DRobot(_robot_selector->currentText().toStdString());
        sim_env::Box2DObjectPtr target = world->getBox2DObject(_target_selector->currentText().toStdString());
        // set robot controller
        setupRobotController(robot);
        // create goal
        Eigen::Vector3f goal_position(1.2, 1.2, 0.0);
        // create planning problem
        mps::planner::pushing::PlanningProblem planning_problem(world, robot,
                                                                _robot_controller, target,
                                                                goal_position);
        configurePlanningProblem(planning_problem);
        _planner_thread.planner.setup(planning_problem);
        _planner_thread.planner.setSliceDrawer(_slice_drawer);
        visualizePlanningProblem(planning_problem);
        _planner_thread.interrrupt = false;
        _planner_thread.thread = std::thread(&PlannerThread::plan, std::ref(_planner_thread));
    }
}

void PushPlannerWidget::stopPlannerThread() {
    static std::string log_prefix("[planner_tests::box2d::widget::PushPlannerWidget::stopPlannerThread]");
    auto world = lockWorld();
    if (_planner_thread.thread.joinable()) {
        world->getLogger()->logInfo("Stopping planner...", log_prefix);
        _planner_thread.interrrupt = true;
        _planner_thread.thread.join();
    }
    world->getLogger()->logInfo("Planner stopped", log_prefix);
}

void PushPlannerWidget::startPlayback() {
    static std::string log_prefix("[planner_tests::box2d::widget::PushPlannerWidget::startPlayback]");
    auto world = lockWorld();
    if (not _planner_thread.thread.joinable()) {
        world->getLogger()->logInfo("Starting playback", log_prefix);
        _planner_thread.interrrupt = false;
        _planner_thread.thread = std::thread(&PlannerThread::playback, std::ref(_planner_thread));
    }
}

void PushPlannerWidget::setupRobotController(sim_env::Box2DRobotPtr robot) {
    auto iter = _velocity_controllers.find(robot->getName());
    if (iter != _velocity_controllers.end()) {
        _robot_controller = iter->second;
    } else {
        _robot_controller = std::make_shared<sim_env::Box2DRobotVelocityController>(robot);
    }
    using namespace std::placeholders;
    sim_env::Robot::ControlCallback callback = std::bind(&sim_env::Box2DRobotVelocityController::control,
                                                         _robot_controller,
                                                         _1, _2, _3, _4, _5);
    robot->setController(callback);
}
