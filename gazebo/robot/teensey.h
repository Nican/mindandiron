#pragma once

#include <QSerialPort>
#include <eigen3/Eigen/Dense>

namespace Robot {

struct TeenseyStatus
{
	double leftPosition;
	double rightPosition;
	Eigen::Vector3d acceleration;
	bool autoFlag;
};

class Teensey : public QObject
{
    Q_OBJECT

    QSerialPort* mSerial;



public:
    
    Teensey(QObject* parent);

protected slots:
	void receiveSerialData();
	void receiveError();

signals:
    void statusUpdate(TeenseyStatus);


};

}