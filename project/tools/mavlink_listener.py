#!/usr/bin/env python

import sys, os
from optparse import OptionParser

# tell python where to find mavlink so we can import it
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink'))
from pymavlink import mavutil

def handle_heartbeat(msg):
	mode = mavutil.mode_string_v10(msg)
	is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
	is_enabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED

def handle_rc_raw(msg):
	channels = (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, 
			msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw)

def handle_hud(msg):
	hud_data = (msg.airspeed, msg.groundspeed, msg.heading, 
				msg.throttle, msg.alt, msg.climb)
	print "Aspd\tGspd\tHead\tThro\tAlt\tClimb"
	print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f" % hud_data

def handle_attitude(msg):
	attitude_data = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed, 
				msg.pitchspeed, msg.yawspeed)
	print "Roll\tPit\tYaw\tRSpd\tPSpd\tYSpd"
	print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t" % attitude_data

def handle_unknown(msg):
	print("Unknown Message Received: ",msg)

def handle_vision_position_estimate(msg):
	print("X:{} | Y:{} | Z:{} | Roll:{} | Pitch:{} | Yaw:{}".format(msg.x,msg.y,msg.z,msg.roll,msg.pitch,msg.yaw))

def handle_param_value(msg):
	print("Param Type: {} | ID: {} | Value: {} | Count: {} | Index: {}".format(msg.param_type,msg.param_id,msg.param_value,msg.param_count,msg.param_index))

def read_loop(m):

	while(True):

		# grab a mavlink message
		msg = m.recv_match(blocking=False)
		
		if not msg:
			#print("Message was None")
			continue

		# handle the message based on its type
		msg_type = msg.get_type()
		print(msg_type)
		if msg_type == "BAD_DATA":
			if mavutil.all_printable(msg.data):
				sys.stdout.write(msg.data)
				sys.stdout.flush()
		elif msg_type == "RC_CHANNELS_RAW": 
			handle_rc_raw(msg)
		elif msg_type == "HEARTBEAT":
			handle_heartbeat(msg)
		elif msg_type == "VFR_HUD":
			handle_hud(msg)
		elif msg_type == "ATTITUDE":
			handle_attitude(msg)
		elif msg_type == "VISION_POSITION_ESTIMATE":
			handle_vision_position_estimate(msg)
		elif msg_type == "PARAM_VALUE":
			handle_param_value(msg)
		else:
			handle_unknown(msg)
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
	#master.wait_heartbeat()

	# request data to be sent at the given rate
	master.mav.request_data_stream_send(master.target_system, master.target_component, 
	mavutil.mavlink.MAV_DATA_STREAM_ALL, opts.rate, 1)

	# enter the data loop
	read_loop(master)


if __name__ == '__main__':
	main()
