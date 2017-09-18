//
// Created by joshua on 8/22/17.
//

#ifndef PLANNER_TESTS_PUSHPLANNERWIDGET_H
#define PLANNER_TESTS_PUSHPLANNERWIDGET_H

#include <thread>

#include <QtGui/QWidget>
#include <QtGui/QGroupBox>
#include <QtGui/QComboBox>
#include <QtGui/QLineEdit>
#include <QtGui/QCheckBox>
#include <QtGui/QPushButton>

#include <sim_env/Box2DWorld.h>
#include <sim_env/Box2DWorldViewer.h>
#include <sim_env/Box2DController.h>

#include <mps/planner/pushing/OraclePushPlanner.h>
#include <mps/planner/util/yaml/OracleParsing.h>

namespace planner_tests {
    namespace box2d {
        namespace widget {
            class PushPlannerWidget : public QGroupBox {
                Q_OBJECT
            public:
                PushPlannerWidget(sim_env::Box2DWorldPtr world, sim_env::Box2DWorldViewerPtr viewer, QWidget* parent=0);
                ~PushPlannerWidget();
                void setPlanningProblem(mps::planner::util::yaml::OraclePlanningProblemDesc& desc);
            public slots:
                void button_clicked(bool enabled);
            private:
                struct PlannerThread {
                    mps::planner::pushing::OraclePushPlanner planner;
                    mps::planner::pushing::PlanningSolution solution;
                    bool isInterrupted();
                    std::thread thread;
                    bool interrrupt;
                    void plan();
                    void playback();
                };

                PlannerThread _planner_thread;
                sim_env::Box2DWorldWeakPtr _weak_world;
                sim_env::Box2DRobotVelocityControllerPtr _robot_controller;
                std::map<std::string, sim_env::Box2DRobotVelocityControllerPtr> _velocity_controllers;
                sim_env::Box2DWorldViewerWeakPtr _weak_viewer;
                // drawing handles
                std::vector<sim_env::WorldViewer::Handle> _drawing_handles;
                // input widgets
                // first column group (0, 1)
                QComboBox* _robot_selector;
                QComboBox* _target_selector;
                QLineEdit* _time_out_edit;
                QCheckBox* _semi_dynamic_check_box;
                QCheckBox* _debug_check_box;
                QLineEdit* _t_max_edit;
                QLineEdit* _num_control_samples;
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
                QLineEdit* _goal_x;
                QLineEdit* _goal_y;
                QLineEdit* _goal_radius;
                QLineEdit* _goal_bias;
                // third column group (5, 6)
                QLineEdit* _robot_bias;
                QLineEdit* _target_bias;
                // all columns
                QPushButton* _start_button;
                QPushButton* _play_back_button;

                // member functions
                void buildUI();
                void synchUI();
                void configurePlanningProblem(mps::planner::pushing::PlanningProblem& pp);
                sim_env::Box2DWorldPtr lockWorld();
                void visualizePlanningProblem(const mps::planner::pushing::PlanningProblem& problem);
                float readValue(QLineEdit* text_field, float default_value);
                void setValue(QLineEdit* text_field, float value);
                int readIntValue(QLineEdit* text_field, int default_value);
                void startPlanner();
                void startPlayback();
                void stopPlannerThread();
                void setupRobotController(sim_env::Box2DRobotPtr robot);
            };
        }
    }
}
#endif //PLANNER_TESTS_PUSHPLANNERWIDGET_H
