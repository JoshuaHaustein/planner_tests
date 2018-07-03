//
// Created by joshua on 7/02/18.
//

#ifndef PLANNER_TESTS_ALPHASORTWIDGET_H
#define PLANNER_TESTS_ALPHASORTWIDGET_H

#include <thread>

#include <QtGui/QWidget>
#include <QtGui/QTabWidget>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>

#include <sim_env/Box2DWorld.h>
#include <sim_env/Box2DWorldViewer.h>
#include <sim_env/Box2DController.h>

#include <mps/planner/sorting/PushSortingPlanner.h>

#include <planner_tests/box2d/widget/QtUtils.h>

namespace planner_tests {
    namespace box2d {
        namespace widget {

            class AlphaSortWidget : public QGroupBox { // the widget needs to inherit from QWidget or a decendant
                Q_OBJECT
            public:
                AlphaSortWidget(sim_env::Box2DWorldPtr world, sim_env::Box2DRobotVelocityControllerPtr controller,
                                sim_env::Box2DWorldViewerPtr viewer, QWidget* parent=0);
                ~AlphaSortWidget();
            public slots:
                void button_clicked(bool enabled); // callback for button click events
            protected:
                void buildUI(); // constructs GUI elements
            private:

                // A helper struct that allows us to run a planner in another thread
                struct PlannerThread {
                    mps::planner::sorting::PushSortingPlanner planner;
                    mps::planner::sorting::PlanningSolution solution;
                    bool isInterrupted();
                    std::thread thread;
                    bool interrrupt;
                    bool playback_synch;
                    void plan();
                    void playback();
                };

                // starts the planner
                void startPlanner();
                // starts playback of a solution
                void startPlayback();
                // stops the planner or playback
                void stopPlannerThread();
                //
                sim_env::Box2DRobotVelocityControllerPtr setupRobotController(sim_env::Box2DRobotPtr robot);
                sim_env::Box2DWorldPtr lockWorld();

                // GUI elements
                QLineEdit* _a_text_box;
                QPushButton* _start_button;
                // thread object for running a planner
                PlannerThread _planner_thread;
            private:
                sim_env::Box2DWorldWeakPtr _weak_world;
                sim_env::Box2DWorldViewerWeakPtr _weak_viewer;
                sim_env::Box2DRobotVelocityControllerWeakPtr _weak_controller;
                // drawing handles
                std::vector<sim_env::WorldViewer::Handle> _drawing_handles;
            };
        }
    }
}
#endif //PLANNER_TESTS_PUSHPLANNERWIDGET_H
