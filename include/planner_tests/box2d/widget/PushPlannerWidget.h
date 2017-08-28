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
#include <sim_env/Box2DController.h>

#include <mps/planner/pushing/OraclePushPlanner.h>

namespace planner_tests {
    namespace box2d {
        namespace widget {
            class PushPlannerWidget : public QGroupBox {
                Q_OBJECT
            public:
                PushPlannerWidget(sim_env::Box2DWorldPtr world, QWidget* parent=0);
                ~PushPlannerWidget();
            public slots:
                void button_clicked(bool enabled);
            private:
                struct PlannerThread {
                    mps::planner::pushing::OraclePushPlanner planner;
                    std::thread thread;
                    bool interrrupt;
                    void run();
                };
                PlannerThread _planner_thread;
                sim_env::Box2DWorldWeakPtr _weak_world;
                sim_env::Box2DRobotVelocityControllerPtr _robot_controller;
                // input widgets
                QComboBox* _robot_selector;
                QComboBox* _target_selector;
                QLineEdit* _time_out_edit;
                QCheckBox* _semi_dynamic_check_box;
                QLineEdit* _t_max_edit;
                QPushButton* _start_button;
                // member functions
                void buildUI();
                void synchUI();
                sim_env::Box2DWorldPtr lockWorld();
            };
        }
    }
}
#endif //PLANNER_TESTS_PUSHPLANNERWIDGET_H