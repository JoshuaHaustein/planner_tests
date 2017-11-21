#ifndef PLANNER_TESTS_QTUTILS_H
#define PLANNER_TESTS_QTUTILS_H

#include <QtGui/QLineEdit>

namespace planner_tests {
    namespace box2d {
        namespace widget {
            float readValue(QLineEdit* text_field, float default_value);
            float readValue(const QString& text, float default_value);
            void setValue(QLineEdit* text_field, float value);
            int readIntValue(QLineEdit* text_field, int default_value);
        }
    }
}
#endif