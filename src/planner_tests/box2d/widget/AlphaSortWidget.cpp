//
// Created by joshua on 8/22/17.
//

#include <planner_tests/box2d/widget/AlphaSortWidget.h>
#include <QtGui/QGridLayout>
#include <QtGui/QLabel>
#include <QtGui/QFileDialog>
#include <sim_env/Box2DController.h>
#include <mps/planner/util/Logging.h>

using namespace planner_tests::box2d::widget;
namespace mps_sorting = mps::planner::sorting;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////// AlphaSortWidget /////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int NUM_GROUP_COLORS = 6;
float GROUP_COLORS[6][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0},
                            {0.5, 0.5, 0.0}, {0.0, 0.5, 0.5}, {0.5, 0.0, 0.5}};


AlphaSortWidget::AlphaSortWidget(mps_sorting::PlanningProblem& planning_problem,
                                 sim_env::Box2DWorldViewerPtr viewer, QWidget* parent) :
    QGroupBox("AlphaSortWidget", parent),_weak_viewer(viewer), _planning_problem(planning_problem)
{
    buildUI(); // fill this GUI element with content
    _planning_problem.stopping_condition = std::bind(&AlphaSortWidget::PlannerThread::isInterrupted, std::ref(_planner_thread));
    updateObjectColors();
}

AlphaSortWidget::~AlphaSortWidget() = default;

void AlphaSortWidget::button_clicked(bool enabled) {
    // This function is called when the user clicks _start_button
    static std::string log_prefix("[planner_tests::box2d::widget::AlphaSortWidget::button_clicked]");
    auto world = _planning_problem.world; 
    // we can identify the source of this call by calling QObject::sender() and compare it to
    // the buttons that we have
    auto* button_sender = dynamic_cast<QPushButton*>(QObject::sender());
    if (!button_sender) {
        // in case there is no sender, there is something seriously wrong
        world->getLogger()->logErr("Could not identify source of button_clicked signal.", log_prefix);
        return;
    }
    if (enabled) { // enabled is true if the button is clicked
        if (button_sender == _start_button) {
            updateObjectColors();
            // here you can start your planner
            startPlanner();
            // for now, let us just read the text the user put in in the text box
            // and print it using the world's logger.
            world->getLogger()->logInfo("You pressed the start button!");
            auto input_txt = _a_text_box->text();
            // the following line will only be printed if the the logger is set to debug level
            world->getLogger()->logDebug("The user intput is " + input_txt.toStdString());
        } else {
            world->getLogger()->logErr("Click event received from an unknown button", log_prefix);
        }
    } else {
        // here you could stop the planner
        stopPlannerThread();
        world->getLogger()->logInfo("Button released");
    }
}

void AlphaSortWidget::buildUI() {
    // a widget needs to have a layout that decides where it should place GUI elements
    auto *layout = new QGridLayout(); // for our purposes we can use a GridLayout, which places GUI elements in a grid
    // let's add some elements to this layout
    unsigned int row = 0; // variables to keep track of where in the grid we place our GUI elements
    unsigned int col = 0;
    // let's put a label that just shows some text and a LineEdit, which allows the user to input some text
    QLabel* label = new QLabel("This is a text label"); // we can store this one in a local variable
    // for the case that we want to reference the line edit later, we keep a reference
    // to it in a member variable.
    _a_text_box = new QLineEdit();
    _a_text_box->setText("Enter some text here!"); // we can set some initial text value
    // we add elements to the GUI by adding them to the layout
    layout->addWidget(label, row, col);
    layout->addWidget(_a_text_box, row, col + 1);
    // Note that now that we added these elements to the layout, the layout will take
    // care of their lifetime management. That is, we do not need to delete _a_test_box
    // or label. It will be sufficient to delete layout, which will be done by this widget
    // once we added it. See below and the Qt documentation for more details.
    ++row;
    // Let us also add a button
    _start_button = new QPushButton(); // this one we also keep as a member variable
    _start_button->setText("Start"); // we can set a label here
    // buttons can be checkable, that is they stay pressed until the user presses the button again.
    // this is useful for using a button as turn on /off switch. As long as the button is pressed(checked),
    // we can keep running the planner. The user can then abort the planning process by unpressing the button
    _start_button->setCheckable(true);
    layout->addWidget(_start_button, row, 0); // add the buttom in the row below the above two elements
    // In order to be able to react to a button click, we need to connect its click signal
    // to a slot function, where we define what to do. See the Qt documentation for more details on Signals and Slots.
    // For now it suffices to understand that the following lines make our button_clicked function being
    // called when the user clicks the start_button.
    QObject::connect(_start_button, SIGNAL(clicked(bool)),
                     this, SLOT(button_clicked(bool)));
    setLayout(layout);  // finally add the layout to this widget (we are inheriting from QGroupBox, which is a widget)
    // we don't need to delete any of the things we created here, as they will be deleted when this
    // widget is deleted by default.
}

///////////////////////////////////////////////////////////////////////////////////////////////
//////// You can use the following code to run your planner in a separate thread //////////////
///////////////////////////////////////////////////////////////////////////////////////////////
void AlphaSortWidget::PlannerThread::plan() {
    planner.solve(solution);
}

void AlphaSortWidget::PlannerThread::playback() {
    if (solution.solved) {
        planner.playback(solution, std::bind(&AlphaSortWidget::PlannerThread::isInterrupted, this), playback_synch);
    }
}

bool AlphaSortWidget::PlannerThread::isInterrupted() {
    return interrrupt;
}

void AlphaSortWidget::startPlanner() {
    static std::string log_prefix("[planner_tests::box2d::widget::AlphaSortWidget::startPlanner]");
    auto world = _planning_problem.world;
    // let's launch the planner in a separate thread
    if (not _planner_thread.thread.joinable()) { // if the thread is not already running
        // first check if the planning problem is fully defined, i.e. whether we actually have groups for all objects
        if (_planning_problem.sorting_groups.empty()) {
            world->getLogger()->logErr("Cannot start planner. No sorting groups set.", log_prefix);
            return;
        }
        if (!isValidPlanningProblem()) return;
        world->getLogger()->logInfo("Starting planner", log_prefix);
        mps::planner::util::logging::setLogger(world->getLogger());
        _planner_thread.interrrupt = false;
        _planner_thread.planner.setup(_planning_problem);
        // the following line creates a thread object and tells this thread to execute
        // the plan function of the _planner_thread object. This is the function defined a few line above,
        // i.e. AlphaSortWidget::PlannerThread::plan
        _planner_thread.thread = std::thread(&PlannerThread::plan, std::ref(_planner_thread));
    } else {
        world->getLogger()->logWarn("Could not start planner because the planner thread is still running.", log_prefix);
        world->getLogger()->logWarn("You need to stop it manually!", log_prefix);
    }
}

void AlphaSortWidget::stopPlannerThread() {
    static std::string log_prefix("[planner_tests::box2d::widget::AlphaSortPlanner::stopPlannerThread]");
    auto world = _planning_problem.world;
    if (_planner_thread.thread.joinable()) {
        world->getLogger()->logInfo("Stopping planner...", log_prefix);
        _planner_thread.interrrupt = true;
        _planner_thread.thread.join();
    }
    world->getLogger()->logInfo("Planner stopped", log_prefix);
}

void AlphaSortWidget::startPlayback() {
    // this fuction is similiar to startPlanner, but lets the thread execute the playback
    // function which is there so playback a solution found by the planner
    static std::string log_prefix("[planner_tests::box2d::widget::AlphaSortWidget::startPlayback]");
    auto world = _planning_problem.world;
    if (not _planner_thread.thread.joinable()) {
        world->getLogger()->logInfo("Starting playback", log_prefix);
        _planner_thread.interrrupt = false;
         // Box2D behaves non-determinstically (due to some internal state that we can not access).
         // Hence, if we playback a solution, it might not turn out the way it did when the planner
         // was modelling it. To be able to see the solution as it was planned, we can
         // force the playback function to synchronize the simulated state during playback
         // with the modeled state during planning.
        _planner_thread.playback_synch = true; // let's synchronize for now
        _planner_thread.thread = std::thread(&PlannerThread::playback, std::ref(_planner_thread));
    } else {
        world->getLogger()->logWarn("Could not start playback because the planner thread is still running.", log_prefix);
        world->getLogger()->logWarn("You need to stop it manually!", log_prefix);
    }
}

void AlphaSortWidget::updateObjectColors() {
    auto viewer = _weak_viewer.lock();
    if (!viewer) return; // TODO print error msg
    auto world_view = viewer->getWorldViewer();
    auto logger = _planning_problem.world->getLogger();
    // TODO reset colors
    for (auto& elem : _planning_problem.sorting_groups) {
        auto idx = elem.second % NUM_GROUP_COLORS;
        if (elem.second > NUM_GROUP_COLORS) {
            logger->logWarn("More groups than available colors."
                            " Some groups will have identical colors.",
                            "[AlphaSortWidget::updateObjectColors]");
        }
        world_view->setColor(elem.first, GROUP_COLORS[idx][0], GROUP_COLORS[idx][1], GROUP_COLORS[idx][2]);
    }
}

bool AlphaSortWidget::isValidPlanningProblem() const {
    // check whether we have a world
    if (!_planning_problem.world or !_planning_problem.robot or !_planning_problem.robot_controller) {
        return false;
    }
    auto logger = _planning_problem.world->getLogger();
    // check whether we have groups for all objects
    std::vector<sim_env::ObjectConstPtr> objects;
    _planning_problem.world->getObjects(objects);
    bool problem_valid = true;
    for (auto& object : objects) {
        if (!object->isStatic()) {
            auto map_iter = _planning_problem.sorting_groups.find(object->getName());
            if (map_iter == _planning_problem.sorting_groups.end()) {
                auto msg = boost::format("Planning problem invalid. Group assignment for %s is missing.") % object->getName();
                logger->logWarn(msg, "[AlphaSortWidget::isValidPlanningProblem()");
                problem_valid = false;
            }
        }
    }
    return problem_valid;
}
