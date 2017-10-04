//
// Created by joshua on 10/4/17.
//

#ifndef PLANNER_TESTS_SLICEWIDGET_H
#define PLANNER_TESTS_SLICEWIDGET_H

#include <QtGui/QWidget>
#include <QtGui/QTableView>
#include <sim_env/Box2DWorld.h>
#include <mps/planner/pushing/algorithm/RRT.h>

namespace planner_tests {
    namespace box2d {
        namespace widget {
            class SliceWidget : public QTableView {
                Q_OBJECT
                friend class ListSliceDrawer;
            public:

                class ListSliceDrawer : public mps::planner::pushing::algorithm::SliceDrawerInterface {
                    friend class SliceWidget;
                public:
                    ListSliceDrawer(SliceWidget* widget);
                    ~ListSliceDrawer();
                    void clear() override;
                    void addSlice(mps::planner::pushing::algorithm::SliceBasedOracleRRT::SliceConstPtr slice) override;
                    void sliceSelected(mps::planner::pushing::algorithm::SliceBasedOracleRRT::SliceConstPtr slice);

                protected:
                    void detach();

                private:
                    SliceWidget* _widget;
                };

                // member functions
                SliceWidget(sim_env::Box2DWorldPtr world, QWidget* parent=0);
                ~SliceWidget();
                mps::planner::pushing::algorithm::SliceDrawerInterfacePtr getSliceDrawer();

            protected:

                class SlicesTableModel : public QAbstractTableModel {
                public:
                    SlicesTableModel(QObject* parent);
                    ~SlicesTableModel();
                    int rowCount(const QModelIndex& parent) const override;
                    int columnCount(const QModelIndex& parent) const override;
                    QVariant data(const QModelIndex& index, int role) const override;
                    QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
                    void addSlice(mps::planner::pushing::algorithm::SliceBasedOracleRRT::SliceConstPtr slice);
                    void clearSlices();
                    std::vector<mps::planner::pushing::algorithm::SliceBasedOracleRRT::SliceConstPtr> _slices;
                };

                // member functions
                void selectionChanged(const QItemSelection& selected, const QItemSelection& deselected) override;
                SlicesTableModel* _table_model;
                sim_env::Box2DWorldPtr getWorld();
            private:
                sim_env::Box2DWorldPtr _world;
                std::shared_ptr<ListSliceDrawer> _slice_drawer;
            };
        }
    }
}

#endif //PLANNER_TESTS_SLICEWIDGET_H
