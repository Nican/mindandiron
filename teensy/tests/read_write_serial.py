import serial


def main():
	ser = serial.Serial('/dev/ttyACM1', 9600)
	counter = 0
	cycleCounts = 50
	Lcmd = 0
	Rcmd = -0
	Collector = 0
	Sorter = 0
	while(True):
		print(ser.readline())
		if counter % cycleCounts == 0:
			print('WRITING DATA TO TEENSY')
			ser.write('LVEL\t%s\tRVEL\t%s\tCOLL\t%s\tSORT\t%s\tEND'
					  % (Lcmd, Rcmd, Collector, Sorter))
			Lcmd += 10
			if Lcmd > 160:
				Lcmd = -160
			Rcmd -= 10
			if Rcmd < -160:
				Rcmd = 160
		counter += 1


if __name__ == '__main__':
	main()