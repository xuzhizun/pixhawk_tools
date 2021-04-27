
'''
set stream rate on an APM
'''
from __future__ import print_function
from builtins import range

import sys

from pymavlink import mavutil

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))

def show_messages(m):
    '''show incoming mavlink messages'''
    while True:
        msg = m.recv_match(blocking=True)
        if not msg:
            return
        if msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
                sys.stdout.flush()
        if msg.get_type() == "RAW_IMU":
            msg_d = msg.to_dict()
            print(msg)

# create a mavlink serial instance


master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# wait for the heartbeat msg to find the system ID
wait_heartbeat(master)

#print("Sending all stream request for rate %u" % args.rate)
for i in range(0, 3):
    master.mav.request_data_stream_send(master.target_system, master.target_component,
                                        mavutil.mavlink.MAV_DATA_STREAM_ALL, 9600, 1)
    
show_messages(master)
