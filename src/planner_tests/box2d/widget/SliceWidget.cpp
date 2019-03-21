//
// Created by joshua on 10/4/17.
//

#include <planner_tests/box2d/widget/SliceWidget.h>

using namespace planner_tests::box2d::widget;

SliceWidget::SliceWidget(sim_env::Box2DWorldPtr world, QWidget* parent)
    : QTableView(parent)
    , _world(world)
{
    _table_model = new SlicesTableModel(this);
    auto* m = selectionModel();
    setModel(_table_model);
    delete m;
    QObject::connect(this, SIGNAL(clicked(const QModelIndex&)), this, SLOT(myItemClicked(const QModelIndex&)));
}

SliceWidget::~SliceWidget()
{
    if (_slice_drawer) {
        _slice_drawer->detach();
    }
}

mps::planner::pushing::algorithm::SliceDrawerInterfacePtr SliceWidget::getSliceDrawer()
{
    if (!_slice_drawer) {
        _slice_drawer = std::make_shared<ListSliceDrawer>(this);
    }
    return _slice_drawer;
}

void SliceWidget::myItemClicked(const QModelIndex& index)
{
    if (!_slice_drawer)
        return;
    if (index.row() >= 0 and index.row() < _table_model->_slices.size()) {
        assert(_selected_slice);
        _slice_drawer->sliceSelected(_selected_slice, _state_idx);
        _state_idx = (unsigned int)((_state_idx + 1) % _selected_slice->slice_samples_list.size());
    }
}

void SliceWidget::selectionChanged(const QItemSelection& selected, const QItemSelection& deselected)
{
    QTableView::selectionChanged(selected, deselected);
    if (!_slice_drawer)
        return;
    if (not selected.isEmpty()) {
        if (not selected.first().indexes().isEmpty()) {
            int slice_idx = selected.first().indexes().first().row();
            _selected_slice = _table_model->_slices.at(slice_idx);
            _state_idx = 0;
            _slice_drawer->sliceSelected(_selected_slice, _state_idx);
        }
    } else {
        _selected_slice = nullptr;
        _slice_drawer->noSliceSelected();
    }
}

sim_env::Box2DWorldPtr SliceWidget::getWorld()
{
    return _world;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////// ListSliceDrawer ////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SliceWidget::ListSliceDrawer::ListSliceDrawer(SliceWidget* widget)
{
    _widget = widget;
}

SliceWidget::ListSliceDrawer::~ListSliceDrawer() = default;

void SliceWidget::ListSliceDrawer::detach()
{
    _widget = nullptr;
    auto logger = sim_env::DefaultLogger::getInstance();
    logger->logWarn("Detaching ListSliceDrawer from widget. Debug drawing of slices will not continue to work with this drawer",
        "[planner_test::box2d::widget::SliceWidget::ListSliceDrawer::detach]");
}

void SliceWidget::ListSliceDrawer::clear()
{
    if (_widget) {
        _widget->_table_model->clearSlices();
    } else {
        auto logger = sim_env::DefaultLogger::getInstance();
        logger->logErr("Attempting to clear a detached ListSliceDrawer!",
            "[planner_test::box2d::widget::SliceWidget::ListSliceDrawer::clear]");
    }
}

void SliceWidget::ListSliceDrawer::addSlice(
    mps::planner::pushing::algorithm::SliceConstPtr slice)
{
    if (_widget) {
        _widget->_table_model->addSlice(slice);
    } else {
        auto logger = sim_env::DefaultLogger::getInstance();
        logger->logErr("Attempting to add a slice to a detached ListSliceDrawer!",
            "[planner_test::box2d::widget::SliceWidget::ListSliceDrawer::addSlice]");
    }
}

void SliceWidget::ListSliceDrawer::sliceSelected(
    mps::planner::pushing::algorithm::SliceConstPtr slice,
    unsigned int state_idx)
{
    if (_widget) {
        auto world = _widget->getWorld();
        auto state = dynamic_cast<const mps::planner::ompl::state::SimEnvWorldState*>(slice->slice_samples_list.at(state_idx)->getState());
        _state_space->setToState(world, state);
        auto debug_drawer = _debug_drawer.lock();
        if (debug_drawer) {
            debug_drawer->clear(false);
            for (auto& motion : slice->slice_samples_list) {
                debug_drawer->addNewMotion(motion);
            }
        }
    }
}

void SliceWidget::ListSliceDrawer::noSliceSelected()
{
    auto debug_drawer = _debug_drawer.lock();
    if (_widget) {
        auto& slices = _widget->_table_model->_slices;
        if (debug_drawer) {
            debug_drawer->clear(false);
            for (auto& slice : slices) {
                for (auto& motion : slice->slice_samples_list) {
                    debug_drawer->addNewMotion(motion);
                }
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////// SlicesTableModel ////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SliceWidget::SlicesTableModel::SlicesTableModel(QObject* parent)
    : QAbstractTableModel(parent)
{
}

SliceWidget::SlicesTableModel::~SlicesTableModel() = default;

int SliceWidget::SlicesTableModel::rowCount(const QModelIndex& parent) const
{
    return (int)_slices.size();
}

int SliceWidget::SlicesTableModel::columnCount(const QModelIndex& parent) const
{
    return 1;
}

QVariant SliceWidget::SlicesTableModel::data(const QModelIndex& index, int role) const
{
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

QVariant SliceWidget::SlicesTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role == Qt::DisplayRole) {
        switch (orientation) {
        case Qt::Horizontal: {
            switch (section) {
            case 0: {
                QString q_string("Number of states:");
                return QVariant(q_string);
            }
            default: {
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

void SliceWidget::SlicesTableModel::addSlice(mps::planner::pushing::algorithm::SliceConstPtr slice)
{
    beginInsertRows(QModelIndex(), (int)_slices.size(), (int)_slices.size() + 1);
    _slices.push_back(slice);
    endInsertRows();
}

void SliceWidget::SlicesTableModel::clearSlices()
{
    beginRemoveRows(QModelIndex(), 0, (int)_slices.size());
    _slices.clear();
    endRemoveRows();
}
