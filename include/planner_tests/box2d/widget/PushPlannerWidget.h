//
// Created by joshua on 8/22/17.
//

#ifndef PLANNER_TESTS_PUSHPLANNERWIDGET_H
#define PLANNER_TESTS_PUSHPLANNERWIDGET_H

#include <thread>

#include <QtGui/QWidget>
#include <QtGui/QTabWidget>
#include <QtGui/QGroupBox>
#include <QtGui/QComboBox>
#include <QtGui/QLineEdit>
#include <QtGui/QCheckBox>
#include <QtGui/QPushButton>
#include <QtGui/QTableWidget>

#include <sim_env/Box2DWorld.h>
#include <sim_env/Box2DWorldViewer.h>
#include <sim_env/Box2DController.h>

#include <mps/planner/pushing/OraclePushPlanner.h>
#include <mps/planner/util/yaml/OracleParsing.h>

#include <planner_tests/box2d/widget/QtUtils.h>

namespace planner_tests {
    namespace box2d {
        namespace widget {
            class PushPlannerWidget;

            class OracleTestWidget : public QGroupBox {
                Q_OBJECT
                friend class PushPlannerWidget;
                public:
                    OracleTestWidget(PushPlannerWidget* parent);
                    ~OracleTestWidget();
                public slots:
                    void button_clicked();
                protected:
                    void buildUI();
                    void synchUI();
                private:
                    PushPlannerWidget* _parent_widget;
                    QComboBox* _target_selector;
                    QPushButton* _move_pushing_state_button;
                    QPushButton* _pushing_policy_button;
                    QPushButton* _reset_button;
                    QLineEdit* _x_target;
                    QLineEdit* _y_target;
                    QLineEdit* _theta_target;

                    std::stack<sim_env::WorldState> _last_world_states;
            };

            class PlannerSetupWidget : public QGroupBox {
                Q_OBJECT
                friend class PushPlannerWidget;
            public:
                PlannerSetupWidget(PushPlannerWidget* parent);
                ~PlannerSetupWidget();
            public slots:
                void button_clicked(bool enabled);
                void goal_edit_button_clicked();
            protected:
                void buildUI();
                void synchUI();
                void setPlanningProblem(mps::planner::util::yaml::OraclePlanningProblemDesc& desc);
                mps::planner::pushing::PlanningProblem getPlanningProblem();
            private:
                // input widgets
                // first column group (0, 1)
                QComboBox* _robot_selector;
                // QComboBox* _target_selector;
                QLineEdit* _time_out_edit;
                QCheckBox* _semi_dynamic_check_box;
                QCheckBox* _debug_check_box;
                QLineEdit* _t_max_edit;
                QLineEdit* _num_control_samples;
                QComboBox* _local_planner_selector;
                // second column group (2, 3, 4)
                QLineEdit* _min_x_workbounds;
                QLineEdit* _max_x_workbounds;
                QLineEdit* _min_y_workbounds;
                QLineEdit* _max_y_workbounds;
                QLineEdit* _max_trans_vel;
                QLineEdit* _max_rot_vel;
                QLineEdit* _min_action_duration;
                QLineEdit* _max_action_duration;
                QComboBox* _oracle_selector;
                QComboBox* _algorithm_selector;
                QComboBox* _shortcut_selector;
                QLineEdit* _goal_bias;
                QLineEdit* _action_noise;
                QLineEdit* _state_noise;
                // third column group (5, 6)
                QLineEdit* _robot_bias;
                QLineEdit* _target_bias;
                QCheckBox* _static_col_allowed;
                QLineEdit* _static_col_blacklist;
                QLineEdit* _shortcut_time;
                QTableWidget* _goals_table;
                QPushButton* _add_goal_button;
                QPushButton* _remove_goal_button;
                // all columns
                QPushButton* _start_button;
                QPushButton* _play_back_button;
                QPushButton* _show_sdf_button;

                PushPlannerWidget* _parent_widget;

                mps::planner::ompl::state::SimEnvValidityChecker::CollisionPolicy _collision_policy;
                void setGoal(const mps::planner::util::yaml::GoalDesc& goal_desc,
                             unsigned int idx);
                void addNewGoal();
                void removeGoal();
                void configurePlanningProblem(mps::planner::pushing::PlanningProblem& pp);
            };

            class PushPlannerWidget : public QTabWidget {
                Q_OBJECT
                friend class PlannerSetupWidget;
                friend class OracleTestWidget;
            public:
                PushPlannerWidget(sim_env::Box2DWorldPtr world, sim_env::Box2DWorldViewerPtr viewer, QWidget* parent=0);
                ~PushPlannerWidget();
                void setPlanningProblem(mps::planner::util::yaml::OraclePlanningProblemDesc& desc);

            protected:
                void visualizePlanningProblem(const mps::planner::pushing::PlanningProblem& problem);
                void showSDF();
                void startPlanner();
                void startPlayback();
                void startOracle(const std::string& target, float x, float y, float theta, bool b_approach);
                void resetOracle(sim_env::WorldState& previous_state);
                void stopPlannerThread();
                sim_env::Box2DRobotVelocityControllerPtr setupRobotController(sim_env::Box2DRobotPtr robot);
                sim_env::Box2DWorldPtr lockWorld();

                struct PlannerThread {
                    mps::planner::pushing::OraclePushPlanner planner;
                    mps::planner::pushing::PlanningSolution solution;
                    mps::planner::ompl::state::goal::RelocationGoalSpecification oracle_goal;
                    bool oracle_approach;
                    bool isInterrupted();
                    std::thread thread;
                    bool interrrupt;
                    void plan();
                    void testOracle();
                    void playback();
                };

                PlannerThread _planner_thread;
            private:
                sim_env::Box2DWorldWeakPtr _weak_world;
                std::map<std::string, sim_env::Box2DRobotVelocityControllerPtr> _velocity_controllers;
                sim_env::Box2DWorldViewerWeakPtr _weak_viewer;
                mps::planner::pushing::algorithm::SliceDrawerInterfacePtr _slice_drawer;
                planner_tests::box2d::widget::PlannerSetupWidget* _planner_tab;
                planner_tests::box2d::widget::OracleTestWidget* _oracle_tab;
                // drawing handles
                std::vector<sim_env::WorldViewer::Handle> _drawing_handles;
            };
        }
    }
}
#endif //PLANNER_TESTS_PUSHPLANNERWIDGET_H
