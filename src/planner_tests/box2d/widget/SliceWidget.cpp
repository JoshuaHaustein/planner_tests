//
// Created by joshua on 10/4/17.
//

#include <planner_tests/box2d/widget/SliceWidget.h>

using namespace planner_tests::box2d::widget;

SliceWidget::SliceWidget(sim_env::Box2DWorldPtr world, QWidget *parent) :
        QTableView(parent),
        _world(world)
{
    _table_model = new SlicesTableModel(this);
    auto *m = selectionModel();
    setModel(_table_model);
    delete m;
}

SliceWidget::~SliceWidget() {
    if (_slice_drawer) {
        _slice_drawer->detach();
    }
}

mps::planner::pushing::algorithm::SliceDrawerInterfacePtr SliceWidget::getSliceDrawer() {
    if (!_slice_drawer) {
        _slice_drawer = std::make_shared<ListSliceDrawer>(this);
    }
    return _slice_drawer;
}

void SliceWidget::selectionChanged(const QItemSelection& selected, const QItemSelection& deselected) {
    if (not selected.first().indexes().isEmpty()) {
        int slice_idx = selected.first().indexes().first().row();
        _slice_drawer->sliceSelected(_table_model->_slices.at(slice_idx));
    }
}

sim_env::Box2DWorldPtr SliceWidget::getWorld() {
    return _world;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////// ListSliceDrawer ////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SliceWidget::ListSliceDrawer::ListSliceDrawer(SliceWidget *widget) {
    _widget = widget;
}

SliceWidget::ListSliceDrawer::~ListSliceDrawer() = default;

void SliceWidget::ListSliceDrawer::detach() {
    _widget = nullptr;
    auto logger = sim_env::DefaultLogger::getInstance();
    logger->logWarn("Detaching ListSliceDrawer from widget. Debug drawing of slices will not continue to work with this drawer",
                    "[planner_test::box2d::widget::SliceWidget::ListSliceDrawer::detach]");
}

void SliceWidget::ListSliceDrawer::clear() {
    if(_widget) {
        _widget->_table_model->clearSlices();
    } else {
        auto logger = sim_env::DefaultLogger::getInstance();
        logger->logErr("Attempting to clear a detached ListSliceDrawer!",
                       "[planner_test::box2d::widget::SliceWidget::ListSliceDrawer::clear]");
    }
}

void SliceWidget::ListSliceDrawer::addSlice(
        mps::planner::pushing::algorithm::SliceBasedOracleRRT::SliceConstPtr slice) {
    if(_widget) {
        _widget->_table_model->addSlice(slice);
    } else {
        auto logger = sim_env::DefaultLogger::getInstance();
        logger->logErr("Attempting to add a slice to a detached ListSliceDrawer!",
                        "[planner_test::box2d::widget::SliceWidget::ListSliceDrawer::addSlice]");
    }
}

void SliceWidget::ListSliceDrawer::sliceSelected(
        mps::planner::pushing::algorithm::SliceBasedOracleRRT::SliceConstPtr slice) {
    if(_widget) {
        auto world = _widget->getWorld();
        auto state = dynamic_cast<const mps::planner::ompl::state::SimEnvWorldState*>(slice->repr->getConstState());
        _state_space->setToState(world, state);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////// SlicesTableModel ////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SliceWidget::SlicesTableModel::SlicesTableModel(QObject* parent) : QAbstractTableModel(parent) {
}

SliceWidget::SlicesTableModel::~SlicesTableModel() = default;

int SliceWidget::SlicesTableModel::rowCount(const QModelIndex& parent) const {
    return (int)_slices.size();
}

int SliceWidget::SlicesTableModel::columnCount(const QModelIndex& parent) const {
    return 1;
}

QVariant SliceWidget::SlicesTableModel::data(const QModelIndex& index, int role) const {
    if (role == Qt::DisplayRole) {
        int slice_idx = index.row();
        assert(slice_idx >= 0 && slice_idx < _slices.size());
        int column_idx = index.column();
        assert(column_idx >= 0 && column_idx < columnCount(QModelIndex()));
        auto slice = _slices.at(slice_idx);
        switch (column_idx) {
//            case 0: { // print identifier
//                return QVariant(slice_idx);
//            }
            case 0: { // print number of robot states
                return QVariant((int)slice->slice_samples_list.size());
            }
            default: {
                return QVariant();
            }
        }
    } else {
        return QVariant();
    }
}

QVariant SliceWidget::SlicesTableModel::headerData(int section, Qt::Orientation orientation, int role) const {
    if (role == Qt::DisplayRole) {
        switch (orientation) {
            case Qt::Horizontal: {
                switch (section) {
                    case 0: {
                        QString q_string("Number of states:");
                        return QVariant(q_string);
                    }
                    default:
                    {
                        QString q_string("UNKNOWN");
                        return QVariant(q_string);
                    }
                }
            }
            case Qt::Vertical: {
                if (section < _slices.size()) {
                    QString q_string("Slice %1");
                    q_string = q_string.arg(section);
                    return QVariant(q_string);
                }
                return QVariant("Empty");
            }
        }
    } else {
        return QVariant();
    }
}

void SliceWidget::SlicesTableModel::addSlice(mps::planner::pushing::algorithm::SliceBasedOracleRRT::SliceConstPtr slice) {
    beginInsertRows(QModelIndex(), (int)_slices.size(), (int)_slices.size() + 1);
    _slices.push_back(slice);
    endInsertRows();
}

void SliceWidget::SlicesTableModel::clearSlices() {
    beginRemoveRows(QModelIndex(), 0, (int)_slices.size());
    _slices.clear();
    endRemoveRows();
}
