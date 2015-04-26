import serial
import random
import time


def main():
    ser = serial.Serial('/dev/ttyACM0', 9600)
    counter = 0
    cycle_counts = 500
    command_position = 0
    delta = 45
    direction = 1
    command_max = 91
    while(True):
        print(ser.readline())
        if counter % cycle_counts == 0:
            # TESTS A PYTHON CONTROLLED SWEEP
            if abs(command_position + direction * delta) > command_max:
                direction *= -1
            command_position += direction * delta
            # TESTS THE ARDUINO SWEEP
            # command_position = 255
            print('WRITING DATA TO ARDUINO: %s' %(command_position))
            ser.write('\t%s\tEND' %(command_position))
        counter += 1


if __name__ == '__main__':
    main()