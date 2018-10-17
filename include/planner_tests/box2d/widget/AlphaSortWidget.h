//
// Created by joshua on 7/02/18.
//

#ifndef PLANNER_TESTS_ALPHASORTWIDGET_H
#define PLANNER_TESTS_ALPHASORTWIDGET_H

#include <thread>

#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QTabWidget>
#include <QtGui/QWidget>

#include <sim_env/Box2DController.h>
#include <sim_env/Box2DWorld.h>
#include <sim_env/Box2DWorldViewer.h>

#include <mps/planner/sorting/PushSortingPlanner.h>

#include <planner_tests/box2d/widget/QtUtils.h>

namespace planner_tests {
namespace box2d {
    namespace widget {

        class AlphaSortWidget : public QGroupBox { // the widget needs to inherit from QWidget or a decendant
            Q_OBJECT
        public:
            AlphaSortWidget(mps::planner::sorting::PlanningProblem& planning_problem,
                sim_env::Box2DWorldViewerPtr viewer, QWidget* parent = 0);
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
            // update the color of objects according to group assignments
            // void updateObjectColors();
            // check whether the planning problem is fully set up
            bool isValidPlanningProblem() const;

            // GUI elements
            QLineEdit* _a_text_box;
            QPushButton* _start_button;
            QPushButton* _render_button;
            // thread object for running a planner
            PlannerThread _planner_thread;

            sim_env::Box2DWorldViewerWeakPtr _weak_viewer;
            mps::planner::sorting::PlanningProblem _planning_problem;
            // drawing handles
            std::vector<sim_env::WorldViewer::Handle> _drawing_handles;
        };
    }
}
}
#endif //PLANNER_TESTS_PUSHPLANNERWIDGET_H
