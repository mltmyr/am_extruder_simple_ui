#include "am_extruder_simple_ui/ui_simple_extruder_ui.hpp"

#include <QGroupBox>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QDoubleValidator>

constexpr double TEMP_MIN =   0.0;
constexpr double TEMP_MAX = 250.0;

constexpr double SPEED_MIN = -100.0;
constexpr double SPEED_MAX =  100.0;

SimpleExtruderUIFrame::SimpleExtruderUIFrame(std::function<void(double)> onTempTargChange, std::function<void(double)> onSpeedTargChange)
{
	this->tempTargChanged  = onTempTargChange;
	this->speedTargChanged = onSpeedTargChange;

	// Widget setup
	QGroupBox* tempGBox  = new QGroupBox("Nozzle Temperature");

	// Nozzle Temperature box
	QLabel* tempTargetLabel  = new QLabel("Target:");
	this->tempTargetField    = new QLineEdit();
	QLabel* tempTargetUnit   = new QLabel("Celsius");

	QLabel* tempCurrentLabel = new QLabel("Current:");
	this->tempCurrentLabel   = new QLabel("0.0");
	QLabel* tempCurrentUnit  = new QLabel("Celsius");

	this->tempTargetField->setMaximumWidth(64);
	this->tempTargetField->setText("0.0");
	tempGBox->setStyleSheet("QGroupBox {qproperty-alignment: AlignLeft}");

	QGridLayout* tempGridLayout = new QGridLayout;
	tempGridLayout->addWidget(tempTargetLabel, 	      0, 0, 1, 1);
	tempGridLayout->addWidget(this->tempTargetField,  0, 1, 1, 1);
	tempGridLayout->addWidget(tempTargetUnit,         0, 2, 1, 1);

	tempGridLayout->addWidget(tempCurrentLabel,       1, 0, 1, 1);
	tempGridLayout->addWidget(this->tempCurrentLabel, 1, 1, 1, 1);
	tempGridLayout->addWidget(tempCurrentUnit,        1, 2, 1, 1);

	tempGridLayout->setColumnStretch(3,1);

	tempGBox->setLayout(tempGridLayout);

	// Filament speed box
	QGroupBox* speedGBox = new QGroupBox("Filament Speed");

	QLabel* speedTargetLabel  = new QLabel("Target:");
	this->speedTargetField    = new QLineEdit();
	QLabel* speedTargetUnit   = new QLabel("mm/s");

	QLabel* speedCurrentLabel = new QLabel("Current:");
	this->speedCurrentLabel   = new QLabel("0.00");
	QLabel* speedCurrentUnit  = new QLabel("mm/s");

	this->speedTargetField->setMaximumWidth(64);
	this->speedTargetField->setText("0.00");
	speedGBox->setStyleSheet("QGroupBox {qproperty-alignment: AlignLeft}");

	QGridLayout* speedGridLayout = new QGridLayout;
	speedGridLayout->addWidget(speedTargetLabel, 	    0, 0, 1, 1);
	speedGridLayout->addWidget(this->speedTargetField,  0, 1, 1, 1);
	speedGridLayout->addWidget(speedTargetUnit,         0, 2, 1, 1);

	speedGridLayout->addWidget(speedCurrentLabel,       1, 0, 1, 1);
	speedGridLayout->addWidget(this->speedCurrentLabel, 1, 1, 1, 1);
	speedGridLayout->addWidget(speedCurrentUnit,        1, 2, 1, 1);

	speedGridLayout->setColumnStretch(3,1);

	speedGBox->setLayout(speedGridLayout);

	// Combine boxes into widget
	QVBoxLayout* mainVLayout = new QVBoxLayout;
	mainVLayout->addWidget(tempGBox);
	mainVLayout->addWidget(speedGBox);
	mainVLayout->addStretch();

	QWidget* centralWidget = new QWidget;
	centralWidget->setLayout(mainVLayout);

	this->setCentralWidget(centralWidget);

	// LineEdit masks and validators
	QDoubleValidator* tempTargetValidator  = new QDoubleValidator( TEMP_MIN,  TEMP_MAX, 1);
	tempTargetValidator->setNotation(QDoubleValidator::StandardNotation);
	this->tempTargetField->setValidator(tempTargetValidator);

	QDoubleValidator* speedTargetValidator = new QDoubleValidator(SPEED_MIN, SPEED_MAX, 2);
	speedTargetValidator->setNotation(QDoubleValidator::StandardNotation);
	this->speedTargetField->setValidator(speedTargetValidator);

	// Internal connections
	QObject::connect(this->tempTargetField,  SIGNAL(editingFinished()), this, SLOT(tempTargetFieldChanged(void)) );
	QObject::connect(this->speedTargetField, SIGNAL(editingFinished()), this, SLOT(speedTargetFieldChanged(void)) );
}

SimpleExtruderUIFrame::~SimpleExtruderUIFrame()
{
	
}

void SimpleExtruderUIFrame::setCurrentTemp(const double temp)
{
	QString tempStr = QString::number(temp, 'f', 1);
	this->tempCurrentLabel->setText(tempStr);
	return;
}

void SimpleExtruderUIFrame::setCurrentSpeed(const double speed)
{
	QString speedStr = QString::number(speed, 'f', 2);
	this->speedCurrentLabel->setText(speedStr);
	return;
}

void SimpleExtruderUIFrame::tempTargetFieldChanged(void)
{
	QString tempStr = this->tempTargetField->text();
	double tempVal = tempStr.toDouble();

	this->tempTargChanged(tempVal);
	return;
}

void SimpleExtruderUIFrame::speedTargetFieldChanged(void)
{
	QString speedStr = this->speedTargetField->text();
	double speedVal = speedStr.toDouble();

	this->speedTargChanged(speedVal);
	return;
}
