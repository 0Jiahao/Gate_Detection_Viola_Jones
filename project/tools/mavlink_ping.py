#!/usr/bin/env python

import sys, os
from optparse import OptionParser
import time
# tell python where to find mavlink so we can import it
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink'))
from pymavlink import mavutil
from pymavlink.dialects.v10 import common as mavlink
def ping_loop(m):

	mav = m.mav
	i = 0
	while(True):
		print("Sending message")
		t1 = time.time()
		mav.ping_send(int(t1*1e6), i%255, 0, 0)
		i += 1
		time.sleep(1)

def main():

	# read command line options
	parser = OptionParser("readdata.py [options]")
	parser.add_option("--baudrate", dest="baudrate", type='int',
					  help="master port baud rate", default=115200)
	parser.add_option("--device", dest="device", default=None, help="serial device")
	parser.add_option("--rate", dest="rate", default=4, type='int', help="requested stream rate")
	parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
					  default=255, help='MAVLink source system for this GCS')
	parser.add_option("--showmessages", dest="showmessages", action='store_true',
					  help="show incoming messages", default=False)
	(opts, args) = parser.parse_args()
	
	if opts.device is None:
		print("You must specify a serial device")
		sys.exit(1)

	# create a mavlink serial instance
	master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate)

	# wait for the heartbeat msg to find the system ID
	master.wait_heartbeat()

	# request data to be sent at the given rate
	master.mav.request_data_stream_send(master.target_system, master.target_component, 
	mavutil.mavlink.MAV_DATA_STREAM_ALL, opts.rate, 1)

	# enter the data loop
	ping_loop(master)


if __name__ == '__main__':
	main()
