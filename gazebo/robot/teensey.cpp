#include "teensey.h"
#include <iostream>
#include <QStringList>

namespace Robot {

Teensey::Teensey(QObject* parent) : QObject(parent)
{

	mSerial = new QSerialPort("/dev/serial/by-id/usb-Teensyduino_USB_Serial_587440-if00", this);

	QObject::connect(mSerial, &QSerialPort::readyRead, this, &Teensey::receiveSerialData);

	if(!mSerial->open(QIODevice::ReadWrite))
		std::cout << "Unabled to open Serial port\n";

}

void Teensey::receiveError()
{
	std::cout << "Teensey received error. " << mSerial->error() << "\n";
}

void Teensey::receiveSerialData()
{
	while(mSerial->canReadLine())
	{
		char buf[1024]; 
	    qint64 lineLength = mSerial->readLine(buf, sizeof(buf));
	    if (lineLength != -1) {
	        QString line(buf);
	        QStringList parts = line.trimmed().split("\t");

	        if(parts.size() != 16)
	        {
	        	std::cerr << "Unable to parse teensey string: '" << buf << "'\n";
	        	return;
	        }

	        TeenseyStatus status;
	        status.leftPosition = parts[1].toDouble() * (2 * M_PI / 23330.0) * 0.155;  // Distance traveled in meters
	        status.rightPosition = parts[3].toDouble() * (2 * M_PI / 23330.0) * 0.155;  // Distance traveled in meters
	        // status.leftVelocity = parts[5].toDouble();
	        // status.rightVelocity = parts[7].toDouble();
	        status.acceleration.x() = parts[9].toDouble() - 512.0;
	        status.acceleration.y() = parts[11].toDouble() - 512.0;
	        status.acceleration.z() = parts[13].toDouble() - 512.0;
	        status.autoFlag = parts[15] == "1";

	        status.acceleration = status.acceleration / 200.0; //Convert from voltage to m/s^2

	        emit statusUpdate(status);
	    }
	}
}

};