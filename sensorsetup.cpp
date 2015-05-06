#include "sensorsetup.h"
#include "ui_sensorsetup.h"

SensorSetup::SensorSetup(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SensorSetup)
{
    ui->setupUi(this);
}

SensorSetup::~SensorSetup()
{
    delete ui;
}

const QString SensorSetup::GetStepCOM() const
{
    return ui->comboBox_step_comport->currentText();
}

const QString SensorSetup::GetLaserCOM() const
{
    return ui->comboBox_laser291_comport->currentText();
}

const QString SensorSetup::GetLMotorCOM() const
{
    return ui->comboBox_dcmotor_left_comport->currentText();
}

const QString SensorSetup::GetRMotorCOM() const
{
    return ui->comboBox_dcmotor_right_comport->currentText();
}

const QString SensorSetup::GetStepBaud() const
{
    return ui->comboBox_step_baudrate->currentText();
}

const QString SensorSetup::GetLaserPreBaud() const
{
    return ui->comboBox_laser291_pre_baudrate->currentText();
}

const QString SensorSetup::GetLaserSetBaud() const
{
    return ui->comboBox_laser291_baudrate->currentText();
}

const QString SensorSetup::GetLMotorBaud() const
{
    return ui->comboBox_dcmotor_left_baudrate->currentText();
}

const QString SensorSetup::GetRMotorBaud() const
{
    return ui->comboBox_dcmotor_right_baudrate->currentText();
}

