#ifndef SENSORSETUP_H
#define SENSORSETUP_H

#include <QWidget>

namespace Ui {
class SensorSetup;
}

class SensorSetup : public QWidget
{
    Q_OBJECT

public:
    explicit SensorSetup(QWidget *parent = 0);
    ~SensorSetup();

    const QString GetStepCOM() const;
    const QString GetLaserCOM() const;
    const QString GetLMotorCOM() const;
    const QString GetRMotorCOM() const;
    const QString GetStepBaud() const;
    const QString GetLaserPreBaud() const;
    const QString GetLaserSetBaud() const;
    const QString GetLMotorBaud() const;
    const QString GetRMotorBaud() const;

private slots:


private:
    Ui::SensorSetup *ui;

};

#endif // SENSORSETUP_H
