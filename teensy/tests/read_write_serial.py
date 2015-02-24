import serial


def main():
	ser = serial.Serial('/dev/ttyACM0', 9600)
	counter = 0
	cycleCounts = 50
	Lcmd = 0
	Rcmd = -0
	while(True):
		print(ser.readline())
		if counter % cycleCounts == 0:
			print('WRITING DATA TO TEENSY')
			ser.write('LVEL\t%s\tRVEL\t%s\tEND' % (Lcmd, Rcmd))
			Lcmd += 10
			if Lcmd > 130:
				Lcmd = -130
			Rcmd -= 10
			if Rcmd < -130:
				Rcmd = 130
		counter += 1


if __name__ == '__main__':
	main()