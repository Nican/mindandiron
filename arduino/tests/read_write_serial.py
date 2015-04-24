import serial
import time


def main():
    # ser = serial.Serial('/dev/kratos_arduino', 9600)
    ser = serial.Serial('/dev/kratos_teensy', 9600)
    Servo = 40

    counter = 0
    cycleCounts = 10

    time.sleep(5)

    while(True):
        # print(ser.readline())
        # print(ser.readline())
        # if counter % cycleCounts == 0:
        if counter == 0:
            print('WRITING 0 TO ARDUINO')
            Servo = 0
            ser.write('CAMSRV\t%s\tEND' % (Servo))
            counter += 1
        else:
            print('WRITING 100 TO ARDUINO')
            Servo = 100
            ser.write('CAMSRV\t%s\tEND' % (Servo))
            counter -= 1
        # counter += 1
        time.sleep(0.33)


if __name__ == '__main__':
    main()