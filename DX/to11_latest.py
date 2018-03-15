#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
mission_basic.py: Example demonstrating basic mission operations including creating, clearing and monitoring missions.

Full documentation is provided at http://python.dronekit.io/examples/mission_basic.html
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
import serial

ser=serial.Serial('COM12', 115200, timeout=1)

#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)
#vehicle = connect('com7', wait_ready=True)
##########################MY VARIABLES##############

##########################


def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.

######################################
def readmission(aFileName):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).

    This function is used by upload_mission().
    """
    print "\nReading mission from file: %s" % aFileName
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist,i


def upload_mission(aFileName):
    """
    Upload a mission from a file. 
    """
    #Read mission from file
    missionlist,no_of_waypoints = readmission(aFileName)
    
    print "\nUpload mission from a file: %s" % import_mission_filename
    #Clear existing mission from vehicle
    print ' Clear mission'
    cmds = vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print ' Upload mission'
    vehicle.commands.upload()
    return no_of_waypoints
###############################

def doauto():
    while True:
        nextwaypoint=vehicle.commands.next
        #print 'Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint())
        print " Altitude: ", vehicle.location.global_relative_frame.alt
    
        '''if nextwaypoint==3: #Skip to next waypoint
            print 'Skipping to Waypoint 5 when reach waypoint 3'
            vehicle.commands.next = 5'''
        if nextwaypoint==no_wp: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
            print "Exit 'standard' mission when start heading to final waypoint (5)"
            break;
        time.sleep(1)

def get_serial_data_home():
    while True:
        ser_data=[]
        ser_data.append(ser.readline())
        #print s
        if ser_data[0]=="":
            print "BULLSHIT"
        elif ser_data[0]=='INVALID\n':
           print "exit"
        else:
            x=[]
        
            for line in ser_data:
                words=line.split('\n')
                x.append(words[0])
            y=[]
            for line in x:
                words=line.split(',')
                y.append(words[0])
                y.append(words[1])
                print y
            home_lat=y[0]
            home_lon=y[1]
            space=""
            F = open("home.txt","w")
            F.write("QGC WPL 110\n")
            F.write("0	1	0	16	0	0	0	0	"+home_lat+"\t"+home_lon+"\t100.000000	1\n")
            F.write("1	0	3	22	0.000000	0.000000	0.000000	0.000000	0.000000	0.000000	100.000000	1\n")
            F.write("2	0	3	16	0.000000	0.000000	0.000000	0.000000	"+home_lat+"\t"+home_lon+"\t100.000000	1\n")
            F.close()
            return home_lat,home_lon
def get_serial_data(home_lat,home_lon):
    while True:
        ser_data=[]
        ser_data.append(ser.readline())
        #print s
        if ser_data[0]=="":
            print "BULLSHIT"
        elif ser_data[0]=='INVALID\n':
           print "exit"
        elif ser_data[0]=="LAND":
            ret_val=1
            break
        elif ser_data[0]=="DISARM\n":
            ret_val=2
            break
        else:
            x=[]
        
            for line in ser_data:
                words=line.split('\n')
                x.append(words[0])
            y=[]
            for line in x:
                words=line.split(',')
                y.append(words[0])
                y.append(words[1])
                print y
            F = open("loiter.txt","w")
            F.write("QGC WPL 110\n")
            F.write("0	1	3	16	0.000000	0.000000	0.000000	0.000000	"+home_lat+"\t"+home_lon+"\t100.000000	1\n")
            F.write("1	0	3	16	0.000000	0.000000	0.000000	0.000000	"+y[0]+"\t"+y[1]+"\t100.000000	1\n")
            F.close()
            ret_val=0
            break
    return ret_val
###################################
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    ########################################################FOR SIMULATION####################
    with open("b.txt", "r") as f:
        data=f.readlines()

    x=[]
    
    for line in data:
        words=line.split('\n')
        x.append(words[0])
    y=[]

    for line in x:
        words=line.split(',')
        vehicle.parameters[words[0]]=float(words[1])
        print vehicle.parameters[words[0]]
    
    ############################################################################################
    print "Basic pre-arm checks"
    # Don't let the user try to arm until autopilot is ready
    ser_data=[]
    
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    while ser_data!="ARM\n":
        ser_data=ser.readline()
    print "Arming motors"
    

    time.sleep(5)
    vehicle.mode = VehicleMode("MANUAL")
    vehicle.armed = True
    print vehicle.mode
    while not vehicle.armed:      
        print " Waiting for arming..."
        time.sleep(1)
    print"blahblahblahblahblah"
    print vehicle.mode
    print vehicle.mode
    print vehicle.mode
    print "Taking off!"
    '''vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print "Reached target altitude"
            break
        time.sleep(1)
    '''
        
print 'Create a new mission (for current location)'
#adds_square_mission(vehicle.location.global_frame,50)
home_lat,home_lon=get_serial_data_home()
import_mission_filename = 'home.txt'
no_wp=upload_mission(import_mission_filename)
# From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
arm_and_takeoff(10)

print "Starting mission"
# Reset mission set to first (0) waypoint
vehicle.commands.next=0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")
print vehicle.mode

time.sleep(2)
# Monitor mission. 
# Demonstrates getting and setting the command number 
# Uses distance_to_current_waypoint(), a convenience function for finding the 
#   distance to the next waypoint.
doauto()
print 'LOITER'
vehicle.mode = VehicleMode("LOITER")

while True:
    ret_val=get_serial_data(home_lat,home_lon)
    if ret_val>0:
        break
    import_mission_filename = 'loiter.txt'
    no_wp=upload_mission(import_mission_filename)
    vehicle.commands.next=0
    vehicle.mode = VehicleMode("AUTO")

    doauto()
    print 'LOITER'
    vehicle.mode = VehicleMode("LOITER")
    time.sleep(10)
if ret_val==2:
    vehicle.armed = False
else:

    vehicle.mode = VehicleMode("RTL")

    alt1=10000
    while alt1>500:
        alt1=alt1-500
        vehicle.parameters['ALT_HOLD_RTL']=alt1
        print vehicle.parameters['ALT_HOLD_RTL']
        vehicle.mode = VehicleMode("MANUAL")
        vehicle.mode = VehicleMode("RTL")
        time.sleep(5)
#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()

# Shut down simulator if it was started.
'''if sitl is not None:
    sitl.stop()
'''
