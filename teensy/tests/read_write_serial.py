import serial


def checkForJumps(msg, LPOS, RPOS):
    data_list = msg.split('\t')
    lPos = int(data_list[1])
    rPos = int(data_list[3])
    if abs(lPos - LPOS) > 2500:
        print('LPOS had a large jump of %d' % (lPos - LPOS))
    if abs(rPos - RPOS) > 2500:
        print('RPOS had a large jump of %d' % (rPos - RPOS))
    return lPos, rPos


def main():
    ser = serial.Serial('/dev/kratos_teensy', 9600)
    counter = 0
    cycleCounts = 50
    lVel = 0.55
    lAdd = 0.05
    rVel = -0.55
    rAdd = -0.05
    lPos = 400000
    rPos = 400000
    LPOS = 0  # Read from the Teensy
    RPOS = 0  # Read from the Teensy
    useVelocity = 1
    Collector = 0
    Sorter = 0
    while(True):
        message = ser.readline()
        print(message)
        LPOS, RPOS = checkForJumps(message, LPOS, RPOS)
        if counter % cycleCounts == 0:
            print('WRITING DATA TO TEENSY')
            # Format
            # \tLVEL\tRVEL\tLPOS\tRPOS\tVEL?\tCOLL\tSORT\tEND
            message = '\t%.2f\t%.2f\t%d\t%d\t%d\t%d\t%d\tEND' % (lVel, rVel, lPos, rPos, useVelocity, Collector, Sorter)
            ser.write(message)
            print(message)
            lVel += lAdd
            if abs(lVel) > 0.65:
                lAdd *= -1
            rVel += rAdd
            if abs(rVel) > 0.65:
                rAdd *= -1
        counter += 1


if __name__ == '__main__':
    main()