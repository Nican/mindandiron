#pragma once

#include <QSerialPort>
#include <eigen3/Eigen/Dense>
#include "robot.h"

namespace Robot {

class KratosTeensy : public Teensy
{
    Q_OBJECT

    QSerialPort* mSerial;

public:
    
    KratosTeensy(QObject* parent);

    void SetVelocities(double left, double right);

protected slots:
	void receiveSerialData();
	void receiveError(QSerialPort::SerialPortError error);


};

}