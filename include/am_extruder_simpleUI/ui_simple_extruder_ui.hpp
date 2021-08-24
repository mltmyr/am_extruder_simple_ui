#ifndef UI_SIMPLE_EXTRUDER_UI_HPP__
#define UI_SIMPLE_EXTRUDER_UI_HPP__

#include <functional>

#include <QMainWindow>

#include <QLabel>
#include <QLineEdit>


class SimpleExtruderUIFrame : public QMainWindow
{
	Q_OBJECT
public:
	SimpleExtruderUIFrame(std::function<void(double)> onTempTargChange, std::function<void(double)> onSpeedTargChange);
	void setCurrentTemp(const double temp);
	void setCurrentSpeed(const double speed);

private:
	std::function<void(double)> tempTargChanged;
	std::function<void(double)> speedTargChanged;

	QLineEdit* tempTargetField;
	QLineEdit* speedTargetField;

	QLabel* tempCurrentLabel;
	QLabel* speedCurrentLabel;

private slots:
	void tempTargetFieldChanged(void);
	void speedTargetFieldChanged(void);
}

#endif /* UI_SIMPLE_EXTRUDER_UI_HPP__ */
