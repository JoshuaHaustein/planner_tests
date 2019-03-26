//
// Created by joshua on 10/4/17.
//

#ifndef PLANNER_TESTS_SLICEWIDGET_H
#define PLANNER_TESTS_SLICEWIDGET_H

#include <QtGui/QTableView>
#include <QtGui/QWidget>
#include <mps/planner/pushing/algorithm/RearrangementPlanner.h>
#include <sim_env/Box2DWorld.h>

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
                void addSlice(mps::planner::pushing::algorithm::SliceConstPtr slice) override;
                void sliceSelected(mps::planner::pushing::algorithm::SliceConstPtr slice, unsigned int state_idx = 0);
                void noSliceSelected();

            protected:
                void detach();

            private:
                SliceWidget* _widget;
            };

            // member functions
            SliceWidget(sim_env::Box2DWorldPtr world, QWidget* parent = 0);
            ~SliceWidget();
            mps::planner::pushing::algorithm::SliceDrawerInterfacePtr getSliceDrawer();
        public slots:
            void myItemClicked(const QModelIndex& index);

        protected:
            class SlicesTableModel : public QAbstractTableModel {
            public:
                SlicesTableModel(QObject* parent);
                ~SlicesTableModel();
                int rowCount(const QModelIndex& parent) const override;
                int columnCount(const QModelIndex& parent) const override;
                QVariant data(const QModelIndex& index, int role) const override;
                QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
                void addSlice(mps::planner::pushing::algorithm::SliceConstPtr slice);
                void clearSlices();
                std::vector<mps::planner::pushing::algorithm::SliceConstPtr> _slices;
            };

            // member functions
            void selectionChanged(const QItemSelection& selected, const QItemSelection& deselected) override;
            SlicesTableModel* _table_model;
            sim_env::Box2DWorldPtr getWorld();

        private:
            sim_env::Box2DWorldPtr _world;
            std::shared_ptr<ListSliceDrawer> _slice_drawer;
            unsigned int _state_idx;
            mps::planner::pushing::algorithm::SliceConstPtr _selected_slice;
        };
    }
}
}

#endif //PLANNER_TESTS_SLICEWIDGET_H
