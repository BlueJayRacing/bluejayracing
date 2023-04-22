#!/usr/bin/env python3
import LCD1602
import time

def setup():
	LCD1602.init(0x27, 1)	# init(slave address, background light)
	LCD1602.write(0, 0, 'Hello, world')
	LCD1602.write(0, 1, 'IIC/I2C LCD2004')
	LCD1602.write(0, 2, '20 cols, 4 rows')
	LCD1602.write(0, 3, 'www.sunfounder.com')
	time.sleep(2)

def destroy():
	LCD1602.clear()

if __name__ == "__main__":
	try:
		setup()
	except KeyboardInterrupt:
		destroy()
