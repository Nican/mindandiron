import serial
import time


def main():
    ser = serial.Serial('/dev/ttyACM0', 9600)
    Servo = 60

    counter = 0
    cycleCounts = 10

    while(True):
        print(ser.readline())
        print counter
        if counter % cycleCounts == 0:
            print('WRITING DATA TO ARDUINO')
            ser.write('CAMSRV\t%s\tEND' % (Servo))
        counter += 1
        time.sleep(0.05)


if __name__ == '__main__':
    main()