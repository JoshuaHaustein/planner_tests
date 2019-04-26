//
// Created by joshua on 8/22/17.
//

#include <QtGui/QFileDialog>
#include <QtGui/QGridLayout>
#include <QtGui/QLabel>
#include <mps/planner/util/Logging.h>
#include <planner_tests/box2d/widget/PushPlannerWidget.h>
#include <planner_tests/box2d/widget/SliceWidget.h>
#include <sim_env/Box2DController.h>

using namespace planner_tests::box2d::widget;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////// OracleTestWidget ///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
OracleTestWidget::OracleTestWidget(PushPlannerWidget* parent)
    : QGroupBox("OracleTestWidget", parent)
{
    _parent_widget = parent;
}

OracleTestWidget::~OracleTestWidget() = default;

void OracleTestWidget::button_clicked()
{
    static std::string log_prefix("[planner_tests::box2d::widget::OracleTestWidget::button_clicked]");
    auto world = _parent_widget->lockWorld();
    auto* button_sender = dynamic_cast<QPushButton*>(QObject::sender());
    if (!button_sender) {
        world->getLogger()->logErr("Could not identify source of button_clicked signal.", log_prefix);
        return;
    }
    if (button_sender == _move_pushing_state_button || button_sender == _pushing_policy_button) {
        _last_world_states.push(world->getWorldState());
        _parent_widget->startOracle(_target_selector->currentText().toStdString(),
            readValue(_x_target, 0.0f),
            readValue(_y_target, 0.0f),
            readValue(_theta_target, 0.0f),
            button_sender == _move_pushing_state_button);
    } else if (button_sender == _reset_button) {
        if (!_last_world_states.empty()) {
            _parent_widget->resetOracle(_last_world_states.top());
            _last_world_states.pop();
        }
    } else {
        world->getLogger()->logErr("Click event received from an unknown button", log_prefix);
    }
}

void OracleTestWidget::buildUI()
{
    auto* layout = new QGridLayout();
    // create target selector
    unsigned int row = 0;
    unsigned int col = 0;
    QLabel* label = new QLabel("Target");
    _target_selector = new QComboBox();
    layout->addWidget(label, row, col);
    layout->addWidget(_target_selector, row, col + 1);
    col += 2;
    // x line edit
    label = new QLabel("x");
    _x_target = new QLineEdit();
    _x_target->setText("0.0");
    layout->addWidget(label, row, col);
    layout->addWidget(_x_target, row, col + 1);
    label = new QLabel("x");
    col += 2;
    // y line edit
    label = new QLabel("y");
    _y_target = new QLineEdit();
    _y_target->setText("0.0");
    layout->addWidget(label, row, col);
    layout->addWidget(_y_target, row, col + 1);
    col += 2;
    // theta line edit
    label = new QLabel("theta");
    _theta_target = new QLineEdit();
    _theta_target->setText("0.0");
    layout->addWidget(label, row, col);
    layout->addWidget(_theta_target, row, col + 1);
    ++row;
    col = 0;
    ////////////////////////////////////////////////////////////
    ////////////////////// Bottom button ///////////////////////
    // move to pushing state button
    _move_pushing_state_button = new QPushButton();
    _move_pushing_state_button->setText("Move to sampled pushing state");
    layout->addWidget(_move_pushing_state_button, row, col);
    QObject::connect(_move_pushing_state_button, SIGNAL(clicked()),
        this, SLOT(button_clicked()));
    col += 1;
    // execute pushing policy button
    _pushing_policy_button = new QPushButton();
    _pushing_policy_button->setText("Execute Pushing Action");
    layout->addWidget(_pushing_policy_button, row, col);
    QObject::connect(_pushing_policy_button, SIGNAL(clicked()),
        this, SLOT(button_clicked()));
    col += 1;
    // reset button
    _reset_button = new QPushButton();
    _reset_button->setText("Stop and reset");
    layout->addWidget(_reset_button, row, col);
    QObject::connect(_reset_button, SIGNAL(clicked()),
        this, SLOT(button_clicked()));

    // finally set the layout
    setLayout(layout);
}

void OracleTestWidget::synchUI()
{
    sim_env::Box2DWorldPtr world = _parent_widget->lockWorld();
    // set selectable robots
    std::vector<sim_env::ObjectPtr> objects;
    world->getObjects(objects, false);
    _target_selector->clear();
    for (auto obj : objects) {
        _target_selector->addItem(obj->getName().c_str());
    }
    _last_world_states.push(world->getWorldState());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////// PlannerSetupWidget /////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PlannerSetupWidget::PlannerSetupWidget(PushPlannerWidget* parent)
    : QGroupBox("PlannerSetupWidget", parent)
{
    _parent_widget = parent;
}

PlannerSetupWidget::~PlannerSetupWidget() = default;

void PlannerSetupWidget::setPlanningProblem(mps::planner::util::yaml::OraclePlanningProblemDesc& desc)
{
    static const std::string log_prefix("[planner_tests::box2d::widget::PlannerSetupWidget::setPlanningProblem]");
    auto world = _parent_widget->lockWorld();
    auto logger = world->getLogger();
    // set robot
    int combo_idx = _robot_selector->findText(QString(desc.robot_name.c_str()));
    if (combo_idx == -1) {
        logger->logErr("Could not find robot from problem description", log_prefix);
    } else {
        _robot_selector->setCurrentIndex(combo_idx);
    }
    // set oracle
    std::string oracle_name = mps::planner::util::yaml::oracleTypeToString(desc.oracle_type);
    combo_idx = _oracle_selector->findText(QString(oracle_name.c_str()));
    if (combo_idx == -1) {
        logger->logErr("Could not find oracle from problem description", log_prefix);
    } else {
        _oracle_selector->setCurrentIndex(combo_idx);
    }
    // set robot oracle
    std::string local_planner_type = mps::planner::util::yaml::localPlannerTypeToString(desc.local_planner_type);
    combo_idx = _local_planner_selector->findText(QString(local_planner_type.c_str()));
    if (combo_idx == -1) {
        logger->logErr("Could not find local planner from problem description", log_prefix);
    } else {
        _local_planner_selector->setCurrentIndex(combo_idx);
    }
    // set algorithm
    std::string algorithm_name = mps::planner::util::yaml::algorithmTypeToString(desc.algorithm_type);
    combo_idx = _algorithm_selector->findText(QString(algorithm_name.c_str()));
    if (combo_idx == -1) {
        logger->logErr("Could not find algorithm from problem description", log_prefix);
    } else {
        _algorithm_selector->setCurrentIndex(combo_idx);
    }
    // set shortcut type
    std::string shortcut_name = mps::planner::util::yaml::shortcutTypeToString(desc.shortcut_type);
    combo_idx = _shortcut_selector->findText(QString(shortcut_name.c_str()));
    if (combo_idx == -1) {
        logger->logErr("Could not find shorcut type from problem description", log_prefix);
    } else {
        _shortcut_selector->setCurrentIndex(combo_idx);
    }
    // set shortcut timeout
    setValue(_shortcut_time, desc.shortcut_timeout);
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
    {
        unsigned int idx = 0;
        for (auto& goal_spec : desc.goals) {
            if (_goals_table->rowCount() - 1 < idx) {
                addNewGoal();
            }
            setGoal(goal_spec, idx);
            idx += 1;
        }
    }
    // set biases
    setValue(_goal_bias, desc.goal_bias);
    setValue(_target_bias, desc.target_bias);
    setValue(_robot_bias, desc.robot_bias);
    setValue(_action_noise, desc.action_noise);
    setValue(_state_noise, desc.state_noise);
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

void PlannerSetupWidget::button_clicked(bool enabled)
{
    static std::string log_prefix("[planner_tests::box2d::widget::PlannerSetupWidget::button_clicked]");
    auto world = _parent_widget->lockWorld();
    auto* button_sender = dynamic_cast<QPushButton*>(QObject::sender());
    if (!button_sender) {
        world->getLogger()->logErr("Could not identify source of button_clicked signal.", log_prefix);
        return;
    }
    if (enabled) {
        if (button_sender == _start_button) {
            _parent_widget->startPlanner();
        } else if (button_sender == _play_back_button) {
            _parent_widget->startPlayback();
        } else if (button_sender == _execute_button) {
            _parent_widget->startExecution();
        } else {
            world->getLogger()->logErr("Click event received from an unknown button", log_prefix);
        }
    } else {
        if (button_sender == _show_sdf_button) {
            _parent_widget->showSDF();
        } else if (button_sender == _save_solution_button) {
            QFileDialog open_file_dialog(this);
            open_file_dialog.setAcceptMode(QFileDialog::AcceptSave);
            open_file_dialog.setOption(QFileDialog::Option::DontUseNativeDialog, true);
            QStringList filenames;
            if (open_file_dialog.exec()) {
                filenames = open_file_dialog.selectedFiles();
                _parent_widget->saveSolution(filenames.at(0).toStdString());
            }
        } else if (button_sender == _load_solution_button) {
            QFileDialog open_file_dialog(this);
            open_file_dialog.setAcceptMode(QFileDialog::AcceptOpen);
            open_file_dialog.setOption(QFileDialog::Option::DontUseNativeDialog, true);
            QStringList filenames;
            if (open_file_dialog.exec()) {
                filenames = open_file_dialog.selectedFiles();
                auto filename = filenames.at(0).toStdString();
                std::cout << filename << std::endl;
                _parent_widget->loadSolution(filename);
            }
        } else {
            _parent_widget->stopPlannerThread();
        }
    }
}

void PlannerSetupWidget::goal_edit_button_clicked()
{
    static std::string log_prefix("[planner_tests::box2d::widget::PLannerSetupWidget::goal_edit_button_clicked]");
    auto world = _parent_widget->lockWorld();
    auto* button_sender = dynamic_cast<QPushButton*>(QObject::sender());
    if (!button_sender) {
        world->getLogger()->logErr("Could not identify source of button_clicked signal.", log_prefix);
        return;
    }
    if (button_sender == _add_goal_button) {
        addNewGoal();
    } else if (button_sender == _remove_goal_button) {
        removeGoal();
    } else {
        world->getLogger()->logErr("Click event received from an unknown button", log_prefix);
    }
}

void PlannerSetupWidget::buildUI()
{
    auto* layout = new QGridLayout();
    unsigned int bottom_row = 0;
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
    // label = new QLabel("Target");
    // _target_selector = new QComboBox();
    // layout->addWidget(label, row, col);
    // layout->addWidget(_target_selector, row, col + 1);
    // ++row;
    // create time out edit
    label = new QLabel("Time out");
    _time_out_edit = new QLineEdit();
    _time_out_edit->setText("60.0");
    layout->addWidget(label, row, col);
    layout->addWidget(_time_out_edit, row, col + 1);
    ++row;
    // create semi-dynamic tick box
    // _semi_dynamic_check_box = new QCheckBox();
    // _semi_dynamic_check_box->setText("Semi-dynamic");
    // _semi_dynamic_check_box->setChecked(true);
    // layout->addWidget(_semi_dynamic_check_box, row, col, 1, 1);
    // create playback tick box
    _synch_playback_box = new QCheckBox();
    _synch_playback_box->setText("Synch playback");
    _synch_playback_box->setChecked(true);
    layout->addWidget(_synch_playback_box, row, col, 1, 1);
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
    // Local planner selector
    ++row;
    label = new QLabel("Local planner type");
    layout->addWidget(label, row, col);
    _local_planner_selector = new QComboBox();
    _local_planner_selector->addItem("Line", mps::planner::pushing::PlanningProblem::LocalPlanner::Line);
    _local_planner_selector->addItem("PotentialField", mps::planner::pushing::PlanningProblem::LocalPlanner::PotentialField);
    layout->addWidget(_local_planner_selector, row, col + 1);
    // Shortcut type
    ++row;
    label = new QLabel("Shortcut type");
    layout->addWidget(label, row, col);
    _shortcut_selector = new QComboBox();
    _shortcut_selector->addItem("NoShortcut", mps::planner::pushing::PlanningProblem::ShortcutType::NoShortcut);
    _shortcut_selector->addItem("NaiveShortcut", mps::planner::pushing::PlanningProblem::ShortcutType::NaiveShortcut);
    _shortcut_selector->addItem("LocalShortcut", mps::planner::pushing::PlanningProblem::ShortcutType::LocalShortcut);
    _shortcut_selector->addItem("OracleShortcut", mps::planner::pushing::PlanningProblem::ShortcutType::OracleShortcut);
    _shortcut_selector->addItem("LocalOracleShortcut", mps::planner::pushing::PlanningProblem::ShortcutType::LocalOracleShortcut);
    layout->addWidget(_shortcut_selector, row, col + 1);
    ////////////////////////////////////////////////////////////
    ////////////////////// Second column (2 - 4) ///////////////
    bottom_row = std::max(bottom_row, row);
    row = 0;
    col = 2;
    // goal settings
    // label = new QLabel("Goal region (x, y):");
    // layout->addWidget(label, row, col);
    // _goal_x = new QLineEdit(QString("1.0"));
    // layout->addWidget(_goal_x, row, col + 1);
    // _goal_y = new QLineEdit(QString("1.0"));
    // layout->addWidget(_goal_y, row, col + 2);
    // ++row;
    // Goal bias
    label = new QLabel("Action noise and state noise");
    layout->addWidget(label, row, col);
    _action_noise = new QLineEdit(QString("0.001"));
    layout->addWidget(_action_noise, row, col + 1);
    _state_noise = new QLineEdit(QString("0.001"));
    layout->addWidget(_state_noise, row, col + 2);
    ++row;
    label = new QLabel("Goal bias");
    layout->addWidget(label, row, col);
    _goal_bias = new QLineEdit(QString("0.1"));
    layout->addWidget(_goal_bias, row, col + 1);
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
    _algorithm_selector->addItem("HybridActionRRT", mps::planner::pushing::PlanningProblem::AlgorithmType::HybridActionRRT);
    _algorithm_selector->addItem("GreedyMultiExtendRRT", mps::planner::pushing::PlanningProblem::AlgorithmType::GreedyMultiExtendRRT);
    layout->addWidget(_algorithm_selector, row, col + 1);
    _oracle_selector = new QComboBox();
    _oracle_selector->addItem("Human", mps::planner::pushing::PlanningProblem::OracleType::Human);
    _oracle_selector->addItem("Learned", mps::planner::pushing::PlanningProblem::OracleType::Learned);
    _oracle_selector->addItem("QuasiStaticSE2", mps::planner::pushing::PlanningProblem::OracleType::QuasiStaticSE2Oracle);
    layout->addWidget(_oracle_selector, row, col + 2);
    ++row;
    ///////////////////////////// Columns (5, 6 and 7) //////////////////////
    bottom_row = std::max(bottom_row, row);
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
    // Collision list
    label = new QLabel(QString("Static collisions blacklist"));
    layout->addWidget(label, row, col);
    _static_col_allowed = new QCheckBox(QString("Allow static col"));
    layout->addWidget(_static_col_allowed, row, col + 1);
    _static_col_blacklist = new QLineEdit("robot");
    layout->addWidget(_static_col_blacklist, row, col + 2);
    ++row;
    // slice ball projection
    label = new QLabel(QString("Shortcut timeout"));
    layout->addWidget(label, row, col);
    _shortcut_time = new QLineEdit("5.0");
    layout->addWidget(_shortcut_time, row, col + 1);
    ++row;
    // Goal table
    QStringList header_names;
    header_names << QString("Target") << QString("x") << QString("y") << QString("r");
    _goals_table = new QTableWidget(0, 4);
    _goals_table->setHorizontalHeaderLabels(header_names);
    layout->addWidget(_goals_table, row, col, 3, 2);
    _add_goal_button = new QPushButton();
    _add_goal_button->setText("Add new goal");
    layout->addWidget(_add_goal_button, row, col + 2);
    QObject::connect(_add_goal_button, SIGNAL(clicked()),
        this, SLOT(goal_edit_button_clicked()));
    _remove_goal_button = new QPushButton();
    _remove_goal_button->setText("Remove selected goal");
    layout->addWidget(_remove_goal_button, row + 1, col + 2);
    QObject::connect(_remove_goal_button, SIGNAL(clicked()),
        this, SLOT(goal_edit_button_clicked()));
    addNewGoal();
    bottom_row = std::max(bottom_row, row);
    ////////////////////////////////////////////////////////////
    ////////////////////// Bottom button ///////////////////////
    bottom_row += 1;
    // start button
    _start_button = new QPushButton();
    _start_button->setText("Start planner");
    _start_button->setCheckable(true);
    layout->addWidget(_start_button, bottom_row, 0);
    QObject::connect(_start_button, SIGNAL(clicked(bool)),
        this, SLOT(button_clicked(bool)));
    // play back button
    _play_back_button = new QPushButton();
    _play_back_button->setText("Playback last solution");
    _play_back_button->setCheckable(true);
    layout->addWidget(_play_back_button, bottom_row, 1);
    QObject::connect(_play_back_button, SIGNAL(clicked(bool)),
        this, SLOT(button_clicked(bool)));
    // play back button
    _execute_button = new QPushButton();
    _execute_button->setText("Execute last solution");
    _execute_button->setCheckable(true);
    layout->addWidget(_execute_button, bottom_row, 2);
    QObject::connect(_execute_button, SIGNAL(clicked(bool)),
        this, SLOT(button_clicked(bool)));
    // show sdf button
    _show_sdf_button = new QPushButton();
    _show_sdf_button->setText("Show/Update SDF");
    layout->addWidget(_show_sdf_button, bottom_row, 3);
    QObject::connect(_show_sdf_button, SIGNAL(clicked(bool)),
        this, SLOT(button_clicked(bool)));
    // save solution button
    _save_solution_button = new QPushButton();
    _save_solution_button->setText("Save last solution");
    layout->addWidget(_save_solution_button, bottom_row, 4);
    QObject::connect(_save_solution_button, SIGNAL(clicked(bool)),
        this, SLOT(button_clicked(bool)));
    // load solution button
    _load_solution_button = new QPushButton();
    _load_solution_button->setText("Load solution");
    layout->addWidget(_load_solution_button, bottom_row, 5);
    QObject::connect(_load_solution_button, SIGNAL(clicked(bool)),
        this, SLOT(button_clicked(bool)));
    // finally set the layout
    setLayout(layout);
}

void PlannerSetupWidget::synchUI()
{
    sim_env::Box2DWorldPtr world = _parent_widget->lockWorld();
    // set selectable robots
    std::vector<sim_env::RobotPtr> robots;
    world->getRobots(robots);
    _robot_selector->clear();
    for (auto robot : robots) {
        _robot_selector->addItem(robot->getName().c_str());
    }
    // set selectable target objects
    // TODO update goal list
}

mps::planner::pushing::PlanningProblem PlannerSetupWidget::getPlanningProblem()
{
    auto world = _parent_widget->lockWorld();
    sim_env::Box2DRobotPtr robot = world->getBox2DRobot(_robot_selector->currentText().toStdString());
    // set robot controller
    auto robot_controller = _parent_widget->setupRobotController(robot);
    // create dummy goal
    mps::planner::ompl::state::goal::RelocationGoalSpecification goal_spec("DUMMY", Eigen::Vector3f(),
        Eigen::Quaternionf(), 0.0f, 0.0f);
    // create planning problem
    mps::planner::pushing::PlanningProblem planning_problem(world, robot,
        robot_controller, goal_spec);
    configurePlanningProblem(planning_problem);
    return planning_problem;
}

void PlannerSetupWidget::configurePlanningProblem(mps::planner::pushing::PlanningProblem& pp)
{
    // goal
    pp.relocation_goals.clear();
    std::vector<std::string> target_objects;
    for (int row_id = 0; row_id < _goals_table->rowCount(); ++row_id) {
        auto* cell_widget = _goals_table->cellWidget(row_id, 0);
        auto* combo_widget = dynamic_cast<QComboBox*>(cell_widget);
        if (!combo_widget) {
            auto world = _parent_widget->lockWorld();
            if (world) {
                world->getLogger()->logErr("Could not cast goal table widget to QComboBox",
                    "[planner_tests::box2d::widget::PlannerSetupWidget::configurePlanningProblem]");
            }
            continue;
        }
        std::string object_name = combo_widget->currentText().toStdString();
        auto object_iter = std::find(target_objects.begin(), target_objects.end(), object_name);
        if (object_iter != target_objects.end()) {
            continue;
        }
        target_objects.push_back(object_name);
        Eigen::VectorXf position(3);
        position[0] = readValue(_goals_table->item(row_id, 1)->text(), 0.0);
        position[1] = readValue(_goals_table->item(row_id, 2)->text(), 0.0);
        position[2] = 0.0f;
        float pos_tolerance = readValue(_goals_table->item(row_id, 3)->text(), 0.1);
        float angle_tolerance = 0.0f;
        mps::planner::ompl::state::goal::RelocationGoalSpecification goal_spec(object_name,
            position,
            Eigen::Quaternionf(),
            pos_tolerance,
            angle_tolerance);
        pp.relocation_goals.push_back(goal_spec);
    }
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
    pp.action_noise = readValue(_action_noise, 0.001f);
    pp.state_noise = readValue(_state_noise, 0.001f);
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
    pp.stopping_condition = std::bind(&PushPlannerWidget::PlannerThread::isInterrupted, std::ref(_parent_widget->_planner_thread));
    // debug
    pp.debug = _debug_check_box->isChecked();
    // settings control samples
    int value = readIntValue(_num_control_samples, 10);
    pp.num_control_samples = (unsigned int)(value > 0 ? value : 10);
    // oracle type
    bool ok = true;
    int enum_value = _oracle_selector->itemData(_oracle_selector->currentIndex()).toInt(&ok);
    if (not ok) {
        enum_value = mps::planner::pushing::PlanningProblem::OracleType::Human;
    }
    pp.oracle_type = mps::planner::pushing::PlanningProblem::OracleType(enum_value);
    // algorithm type
    enum_value = _algorithm_selector->itemData(_algorithm_selector->currentIndex()).toInt(&ok);
    if (not ok) {
        enum_value = mps::planner::pushing::PlanningProblem::AlgorithmType::Naive;
    }
    pp.algorithm_type = mps::planner::pushing::PlanningProblem::AlgorithmType(enum_value);
    // local planner type
    enum_value = _local_planner_selector->itemData(_local_planner_selector->currentIndex()).toInt(&ok);
    if (not ok) {
        enum_value = mps::planner::pushing::PlanningProblem::LocalPlanner::Line;
    }
    pp.local_planner_type = mps::planner::pushing::PlanningProblem::LocalPlanner(enum_value);
    // shortcut type
    enum_value = _shortcut_selector->itemData(_shortcut_selector->currentIndex()).toInt(&ok);
    if (not ok) {
        enum_value = mps::planner::pushing::PlanningProblem::ShortcutType::NoShortcut;
    }
    pp.shortcut_type = mps::planner::pushing::PlanningProblem::ShortcutType(enum_value);
    // shortcut timeout
    pp.max_shortcut_time = readValue(_shortcut_time, 5.0f);
    // collision policy
    pp.collision_policy.setStaticCollisions(_static_col_allowed->isChecked());
    // parse black list
    auto blacklist = _static_col_blacklist->text().split(',');
    for (auto& obj_name : blacklist) {
        auto object = pp.world->getObject(obj_name.toStdString(), false);
        if (object) {
            pp.collision_policy.setStaticCollisions(obj_name.toStdString(), false);
        } else {
            pp.world->getLogger()->logErr("Could not add an object with name " + obj_name.toStdString() + " to the collision policy blacklist. The object does not exist.", "[PushPlannerWidget]");
        }
    }
}

void PlannerSetupWidget::setGoal(const mps::planner::util::yaml::GoalDesc& goal_desc, unsigned int idx)
{
    const std::string log_prefix("[planner_tests::box2d::widget::PlannerSetupWidget::setGoal]");
    auto world = _parent_widget->lockWorld();
    if (!world) {
        throw std::logic_error(log_prefix + "Can not access Box2d world");
    }
    if (_goals_table->rowCount() - 1 < idx) {
        throw std::logic_error(log_prefix + " Requested to set a goal entry that does not exist.");
    }
    auto* cell_widget = _goals_table->cellWidget(idx, 0);
    auto* combo_widget = dynamic_cast<QComboBox*>(cell_widget);
    if (!combo_widget) {
        throw std::logic_error(log_prefix + " Goal table does not contain a valid QComboBox");
    }
    int combo_idx = combo_widget->findText(QString(goal_desc.obj_name.c_str()));
    if (combo_idx == -1) {
        world->getLogger()->logErr(boost::format("Could not set goal for object %s. Object unknown!") % goal_desc.obj_name, log_prefix);
        return;
    }
    combo_widget->setCurrentIndex(combo_idx);
    _goals_table->item(idx, 1)->setText(QString("").setNum(goal_desc.goal_position[0]));
    _goals_table->item(idx, 2)->setText(QString("").setNum(goal_desc.goal_position[1]));
    _goals_table->item(idx, 3)->setText(QString("").setNum(goal_desc.goal_region_radius));
}

void PlannerSetupWidget::addNewGoal()
{
    auto world = _parent_widget->lockWorld();
    if (!world) {
        auto logger = sim_env::DefaultLogger::getInstance();
        logger->logErr("Can not access underlying Box2D World", "[planner_tests::box2d::widget::PlannerSetupWidget::addNewGoal]");
        return;
    }
    std::vector<sim_env::ObjectPtr> objects;
    world->getObjects(objects, true);
    std::vector<sim_env::ObjectPtr> movable_objects;
    for (auto& object : objects) {
        if (not object->isStatic()) {
            movable_objects.push_back(object);
        }
    }
    auto logger = sim_env::DefaultLogger::getInstance();
    if (_goals_table->rowCount() == movable_objects.size()) {
        // no object left to add
        logger->logDebug("There are as many goals as movable objects. Nothing to add.", "[planner_tests::box2d::widget::PlannerSetupWidget::addNewGoal]");
        return;
    }
    auto new_row_id = _goals_table->rowCount();
    _goals_table->insertRow(new_row_id);
    auto target_selector = new QComboBox();
    for (auto object : movable_objects) {
        target_selector->addItem(object->getName().c_str());
    }
    _goals_table->setSortingEnabled(false);
    _goals_table->setCellWidget(new_row_id, 0, target_selector);
    _goals_table->setItem(new_row_id, 1, new QTableWidgetItem("0.0"));
    _goals_table->setItem(new_row_id, 2, new QTableWidgetItem("0.0"));
    _goals_table->setItem(new_row_id, 3, new QTableWidgetItem("0.0"));
    logger->logDebug(boost::format("Added a new goal in row %i") % new_row_id, "[planner_tests::box2d::widget::PlannerSetupWidget::addNewGoal]");
}

void PlannerSetupWidget::removeGoal()
{
    auto selected_ranges = _goals_table->selectedRanges();
    if (selected_ranges.size() == 0 or _goals_table->rowCount() == 1) {
        auto logger = sim_env::DefaultLogger::getInstance();
        logger->logDebug("Either nothing selected, or minimal number of goals.",
            "[planner_tests::box2d::widget::PlannerSetupWidget::removeGoal]");
        return;
    }
    auto first_range = selected_ranges.at(0);
    auto row_to_remove = first_range.topRow();
    _goals_table->removeRow(row_to_remove);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////// PushPlannerWidget //////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PushPlannerWidget::PlannerThread::plan()
{
    planner.solve(solution);
}

void PushPlannerWidget::PlannerThread::playback()
{
    if (solution.solved) {
        planner.playback(solution, std::bind(&PushPlannerWidget::PlannerThread::isInterrupted, this), playback_synch);
    }
}

void PushPlannerWidget::PlannerThread::execute()
{
    if (solution.solved) {
        bool success = planner.execute(solution, exec_callback,
            std::bind(&PushPlannerWidget::PlannerThread::isInterrupted, this));
        auto logger = sim_env::DefaultLogger::getInstance();
        if (success) {
            logger->logInfo("Execution successful", "");
        } else {
            logger->logInfo("Execution failed");
        }
    }
}

bool PushPlannerWidget::PlannerThread::isInterrupted()
{
    return interrrupt;
}

void PushPlannerWidget::PlannerThread::testOracle()
{
    // TODO depending on oracle_approach either only call for an approach action, or for pushing policy
    solution.path = planner.testOracle(oracle_goal, oracle_approach);
    solution.solved = true;
    playback_synch = false;
    playback();
}

PushPlannerWidget::PushPlannerWidget(sim_env::Box2DWorldPtr world,
    sim_env::Box2DWorldViewerPtr viewer,
    QWidget* parent)
    : QTabWidget(parent)
{
    _weak_world = world;
    _weak_viewer = viewer;
    // set up planner setup widget
    _planner_tab = new widget::PlannerSetupWidget(this);
    addTab(_planner_tab, "Planner setup");
    _planner_tab->buildUI();
    _planner_tab->synchUI();
    // set up oracle setup widget
    _oracle_tab = new widget::OracleTestWidget(this);
    _oracle_tab->buildUI();
    _oracle_tab->synchUI();
    addTab(_oracle_tab, "Oracle test");
    // set up slice widget
    auto slice_widget = new widget::SliceWidget(world);
    addTab(slice_widget, "Slice widget");
    _slice_drawer = slice_widget->getSliceDrawer();
}

PushPlannerWidget::~PushPlannerWidget() = default;

sim_env::Box2DWorldPtr PushPlannerWidget::lockWorld()
{
    sim_env::Box2DWorldPtr world = _weak_world.lock();
    if (!world) {
        throw std::logic_error("[planner_tests::box2d::widget::lockWorld] Could not access Box2dWorld.");
    }
    return world;
}

void PushPlannerWidget::setPlanningProblem(mps::planner::util::yaml::OraclePlanningProblemDesc& desc)
{
    _planner_tab->setPlanningProblem(desc);
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
    _drawing_handles.emplace_back(viewer->drawBox(pos, extents, Eigen::Vector4f(0, 0, 0, 1), false, 0.01f));
    // 2. reset colors
    std::vector<sim_env::ObjectPtr> objects;
    problem.world->getObjects(objects, true);
    for (auto& obj : objects) {
        auto box2d_object = std::dynamic_pointer_cast<sim_env::Box2DObject>(obj);
        viewer->resetColor(box2d_object->getName());
    }
    // 3. also show goal regions
    for (auto& goal_spec : problem.relocation_goals) {
        Eigen::Vector4f green_color(0.0f, 1.0f, 0.0f, 1.0f);
        _drawing_handles.emplace_back(viewer->drawSphere(goal_spec.goal_position,
            goal_spec.position_tolerance,
            green_color,
            0.01f));
        viewer->setColor(goal_spec.object_name, green_color);
    }
}

void PushPlannerWidget::showSDF()
{
    static std::string log_prefix("[planner_tests::box2d::widget::PushPlannerWidget::showSDF]");
    auto world = lockWorld();
    if (not _planner_thread.thread.joinable()) {
        world->getLogger()->logInfo("Showing SDF", log_prefix);
        mps::planner::util::logging::setLogger(world->getLogger());
        auto planning_problem = _planner_tab->getPlanningProblem();
        _planner_thread.planner.setup(planning_problem);
        _planner_thread.planner.renderSDF(0.01f);
    }
}

void PushPlannerWidget::startPlanner()
{
    static std::string log_prefix("[planner_tests::box2d::widget::PushPlannerWidget::startPlanner]");
    auto world = lockWorld();
    if (not _planner_thread.thread.joinable()) {
        world->getLogger()->logInfo("Starting planner", log_prefix);
        mps::planner::util::logging::setLogger(world->getLogger());
        auto planning_problem = _planner_tab->getPlanningProblem();
        _planner_thread.planner.setup(planning_problem);
        _planner_thread.planner.setSliceDrawer(_slice_drawer);
        visualizePlanningProblem(planning_problem);
        _planner_thread.interrrupt = false;
        _planner_thread.thread = std::thread(&PlannerThread::plan, std::ref(_planner_thread));
    } else {
        world->getLogger()->logWarn("Could not start planner because the planner thread is still running.", log_prefix);
        world->getLogger()->logWarn("You need to stop it manually!", log_prefix);
    }
}

void PushPlannerWidget::stopPlannerThread()
{
    static std::string log_prefix("[planner_tests::box2d::widget::PushPlannerWidget::stopPlannerThread]");
    auto world = lockWorld();
    if (_planner_thread.thread.joinable()) {
        world->getLogger()->logInfo("Stopping planner...", log_prefix);
        _planner_thread.interrrupt = true;
        _planner_thread.thread.join();
    }
    world->getLogger()->logInfo("Planner stopped", log_prefix);
}

void PushPlannerWidget::startPlayback()
{
    static std::string log_prefix("[planner_tests::box2d::widget::PushPlannerWidget::startPlayback]");
    auto world = lockWorld();
    if (not _planner_thread.thread.joinable()) {
        world->getLogger()->logInfo("Starting playback", log_prefix);
        _planner_thread.interrrupt = false;
        _planner_thread.playback_synch = _planner_tab->_synch_playback_box->isChecked();
        _planner_thread.thread = std::thread(&PlannerThread::playback, std::ref(_planner_thread));
    } else {
        world->getLogger()->logWarn("Could not start playback because the planner thread is still running.", log_prefix);
        world->getLogger()->logWarn("You need to stop it manually!", log_prefix);
    }
}

void PushPlannerWidget::startExecution()
{
    static std::string log_prefix("[planner_tests::box2d::widget::PushPlannerWidget::startExecution]");
    auto world = lockWorld();
    if (not _planner_thread.thread.joinable()) {
        world->getLogger()->logInfo("Starting execution", log_prefix);
        _planner_thread.interrrupt = false;
        _planner_thread.thread = std::thread(&PlannerThread::execute, std::ref(_planner_thread));
    } else {
        world->getLogger()->logWarn("Could not start execution because the planner thread is still running.", log_prefix);
        world->getLogger()->logWarn("You need to stop it manually!", log_prefix);
    }
}

void PushPlannerWidget::saveSolution(const std::string& filename)
{
    static std::string log_prefix("[planner_tests::box2d::widget::PushPlannerWidget::saveSolution]");
    stopPlannerThread();
    if (_planner_thread.solution.solved) {
        auto world = lockWorld();
        auto logger = world->getLogger();
        logger->logDebug("Saving solution to file " + filename, log_prefix);
        _planner_thread.planner.saveSolution(_planner_thread.solution, filename);
    }
}

void PushPlannerWidget::loadSolution(const std::string& filename)
{
    static std::string log_prefix("[planner_tests::box2d::widget::PushPlannerWidget::loadSolution]");
    stopPlannerThread();
    auto world = lockWorld();
    auto logger = world->getLogger();
    logger->logDebug("Loading solution from file " + filename, log_prefix);
    auto planning_problem = _planner_tab->getPlanningProblem();
    _planner_thread.planner.setup(planning_problem); // setup the planner just in case
    _planner_thread.planner.loadSolution(_planner_thread.solution, filename);
}

void PushPlannerWidget::startOracle(const std::string& target, float x, float y, float theta, bool b_approach)
{
    static std::string log_prefix("[planner_tests::box2d::widget::PushPlannerWidget::startOracle]");
    auto world = lockWorld();
    auto state = world->getWorldState();
    resetOracle(state);
    if (not _planner_thread.thread.joinable()) {
        world->getLogger()->logInfo("Starting oracle", log_prefix);
        mps::planner::util::logging::setLogger(world->getLogger());
        auto planning_problem = _planner_tab->getPlanningProblem();
        _planner_thread.planner.setup(planning_problem);
        // _planner_thread.planner.setSliceDrawer(_slice_drawer);
        _planner_thread.oracle_goal.goal_position[0] = x;
        _planner_thread.oracle_goal.goal_position[1] = y;
        _planner_thread.oracle_goal.goal_position[2] = theta;
        _planner_thread.oracle_goal.object_name = target;
        _planner_thread.interrrupt = false;
        _planner_thread.oracle_approach = b_approach;
        _planner_thread.thread = std::thread(&PlannerThread::testOracle, std::ref(_planner_thread));
    } else {
        world->getLogger()->logWarn("Could not start oracle because the planner thread is still running.", log_prefix);
        world->getLogger()->logWarn("You need to stop it manually!", log_prefix);
    }
}

void PushPlannerWidget::resetOracle(sim_env::WorldState& previous_state)
{
    stopPlannerThread();
    auto world = lockWorld();
    world->setWorldState(previous_state);
}

sim_env::Box2DRobotVelocityControllerPtr PushPlannerWidget::setupRobotController(sim_env::Box2DRobotPtr robot)
{
    auto iter = _velocity_controllers.find(robot->getName());
    sim_env::Box2DRobotVelocityControllerPtr robot_controller;
    if (iter != _velocity_controllers.end()) {
        robot_controller = iter->second;
    } else {
        robot_controller = std::make_shared<sim_env::Box2DRobotVelocityController>(robot);
        _velocity_controllers[robot->getName()] = robot_controller;
    }
    using namespace std::placeholders;
    sim_env::Robot::ControlCallback callback = std::bind(&sim_env::Box2DRobotVelocityController::control,
        robot_controller,
        _1, _2, _3, _4, _5);
    robot->setController(callback);
    return robot_controller;
}
