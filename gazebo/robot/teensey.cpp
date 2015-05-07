#include "teensey.h"
#include <iostream>
#include <QStringList>

using namespace Robot;


/////////////////////////////
//// KratosTeensy2
/////////////////////////////
KratosTeensy2::KratosTeensy2(QObject* parent) : Teensy2(parent)
{

	mSerial = new QSerialPort("/dev/serial/by-id/usb-Teensyduino_USB_Serial_765570-if00", this);

	connect(mSerial, &QSerialPort::readyRead, this, &KratosTeensy2::receiveSerialData);
	connect(mSerial, SIGNAL(error(QSerialPort::SerialPortError)), this, SLOT(receiveError(QSerialPort::SerialPortError)));

	if(!mSerial->open(QIODevice::ReadWrite))
	{
		std::cout << "Unabled to open Serial port for CAMERA teensy2\n";
		return;
	}

	mSerial->clear();
}

void KratosTeensy2::receiveError(QSerialPort::SerialPortError error)
{
	std::cout << "Teensy2 received error. " << mSerial->error() << "\n";
}

void KratosTeensy2::receiveSerialData()
{
	while(mSerial->canReadLine())
	{
		char buf[1024]; 
		qint64 lineLength = mSerial->readLine(buf, sizeof(buf));
		if (lineLength != -1) {
			QString line(buf);
			QStringList parts = line.trimmed().split("\t");

			if(parts.size() != 6)
			{
				std::cerr << "Unable to parse Teensy2 string: '" << buf << "'\n";
				return;
			}

			Teensy2Status status;
			status.servoAngle = parts[1].toDouble() / 180.0 * M_PI;
			status.current = parts[3].toDouble();
			status.isPaused = parts[5].toInt();

			lastStatus = status;
			emit statusUpdate(status);
		}
	}
	//mSerial->clear();
}

void KratosTeensy2::sendRaw(int intAngle)
{
	QString sendString;
	sendString.sprintf("\t%d\tEND", intAngle);
	mSerial->write(sendString.toLocal8Bit());
}

/////////////////////////////
//// KratosTeensy
/////////////////////////////

KratosTeensy::KratosTeensy(QObject* parent) : Teensy(parent)
{

	mSerial = new QSerialPort("/dev/serial/by-id/usb-Teensyduino_USB_Serial_587440-if00", this);

	QObject::connect(mSerial, &QSerialPort::readyRead, this, &KratosTeensy::receiveSerialData);
	connect(mSerial, SIGNAL(error(QSerialPort::SerialPortError)), this, SLOT(receiveError(QSerialPort::SerialPortError)));

	if(!mSerial->open(QIODevice::ReadWrite)){
		std::cout << "Unabled to open Serial port for motor teensy\n";
		return;
	}

	mSerial->clear();
}

KratosTeensy::~KratosTeensy()
{
	std::cout << "Shutting down teensy. Waiting to shut down motors.\n";
	SetVelocities(0.0, 0.0);
	std::cout << "\tResult of writing bytes: " << mSerial->waitForBytesWritten(500) << "\n";
}

void KratosTeensy::receiveError(QSerialPort::SerialPortError error)
{
	std::cout << "Teensy received error. " << mSerial->error() << "\n";
}

void KratosTeensy::SetVelocities(double left, double right)
{
	if(!mSerial->isOpen())
		return;

	int leftPos = 0;
	int rightPos = 0;
	int useVelocity = 1;
	int collector = 0;
	int sorter = 0;

	QString sendString;
	//sendString.sprintf("LVEL\t%d\tRVEL\t%d\tCOLL\t%s\tSORT\t%s\tEND", leftInt, rightInt, "0", "0");
	sendString.sprintf("\t%.2f\t%.2f\t%d\t%d\t%d\t%d\t%d\tEND", left, right, leftPos, rightPos, useVelocity, collector, sorter);
	//std::cout << "Sending value: " << sendString.toStdString() << "\n";
	mSerial->write(sendString.toLocal8Bit());
}

void KratosTeensy::receiveSerialData()
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
				std::cerr << "Unable to parse Teensy string: '" << buf << "'\n";
				return;
			}

			TeenseyStatus status;
			status.leftPosition = parts[1].toDouble();
			status.rightPosition = parts[3].toDouble(); 
			status.leftVelocity = parts[5].toDouble();
			status.rightVelocity = parts[7].toDouble();
			status.acceleration.x() = parts[9].toDouble() - 512.0;
			status.acceleration.y() = parts[11].toDouble() - 512.0;
			status.acceleration.z() = parts[13].toDouble() - 512.0;
			status.autoFlag = parts[15] == "1";
	        status.acceleration = status.acceleration / 200.0; //Convert from voltage to m/s^2

			emit statusUpdate(status);
		}
	}
	//mSerial->clear();
}


KratosDecawave::KratosDecawave(nzmqt::ZMQContext* context, QObject* parent) : Decawave(parent)
{

	mSubSocket = context->createSocket(nzmqt::ZMQSocket::TYP_SUB, this);
	mSubSocket->setObjectName("Subscriber.Socket.socket(SUB)");
	mSubSocket->connectTo("tcp://127.0.0.1:5560");
	mSubSocket->setOption(nzmqt::ZMQSocket::OPT_SUBSCRIBE, "", 0);

	connect(mSubSocket, SIGNAL(messageReceived(const QList<QByteArray>&)), SLOT(messageReceived(const QList<QByteArray>&)));
}

	
void KratosDecawave::messageReceived(const QList<QByteArray>& messages)
{
	//std::cout << "Message size: " << message.size () << "\n";

	std::cout << "Got message! \n";
	//Why is this a list of arrays?
	for(auto& byteArray : messages)
	{
		QString readString(byteArray);

		if(readString.contains("LAST:"))
		{
			QString doubleVal = readString.replace("LAST: ", "");

			double val = doubleVal.toDouble();

			std::cout << "Read decawave value: " << val << "\n";

			emit statusUpdate(val);
		}

	 	//std::cout << "\tReceived string: " << QString(byteArray).toStdString() << "\n";
	}

}
