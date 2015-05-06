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
	virtual ~KratosTeensy();
	void SetVelocities(double left, double right);

protected slots:
	void receiveSerialData();
	void receiveError(QSerialPort::SerialPortError error);
};

class KratosTeensy2 : public Teensy2
{
	Q_OBJECT
	QSerialPort* mSerial;

public:
	KratosTeensy2(QObject* parent);
	virtual void sendRaw(int intAngle) override;

protected slots:
	void receiveSerialData();
	void receiveError(QSerialPort::SerialPortError error);
};

class KratosDecawave : public Decawave
{
	Q_OBJECT

public:
	nzmqt::ZMQSocket* mSubSocket;

	KratosDecawave(nzmqt::ZMQContext* context, QObject* parent);

public slots:
	void messageReceived(const QList<QByteArray>& messages);
};

}