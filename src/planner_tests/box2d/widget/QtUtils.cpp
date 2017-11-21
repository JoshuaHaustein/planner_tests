#include <planner_tests/box2d/widget/QtUtils.h>

using namespace planner_tests::box2d::widget;


float planner_tests::box2d::widget::readValue(QLineEdit* text_field, float default_value) {
    return readValue(text_field->text(), default_value);
}

float planner_tests::box2d::widget::readValue(const QString& text, float default_value) {
    bool ok = false;
    float value = text.toFloat(&ok);
    if (ok) {
        return value;
    }
    return default_value;
}

void planner_tests::box2d::widget::setValue(QLineEdit* text_field, float value) {
    QString q_string;
    q_string.setNum(value);
    text_field->setText(q_string);
}

int planner_tests::box2d::widget::readIntValue(QLineEdit* text_field, int default_value) {
    bool ok = false;
    int value = text_field->text().toInt(&ok);
    if (ok) {
        return value;
    }
    return default_value;
}