# borrowed ideas from miniterm.py, part of Python2.7 distribution.

import sys
import os
import argparse
import time
import timeit
import math
import serial
import crcmod
import threading
from serial.tools.list_ports import comports
import dragonfly_fcb_pb2
import google.protobuf.message
from collections import deque

from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg

debug_print = True
script_name = os.path.basename(__file__)[:-3]

def pprint(format, *args):
    string_to_print = script_name + ": " + format % args
    print string_to_print

def dprint(format, *args):
    string_to_print = script_name + ": " + format % args
    if debug_print:
        pprint(string_to_print)

tool_description="Sample the estimated RPY angles from the FCB card. Also see README.md."

arg_parser = argparse.ArgumentParser(description=tool_description)
arg_parser.add_argument("interval_ms", type=int, help="[100 ...] nbr of milliseconds between each state sample")
arg_parser.add_argument("duration_s", type=int, help="[2 ..] state will be sampled for a duration of seconds")
arg_parser.add_argument("--com", type=str, default="/dev/ttyACM0", help="give the name of the COM port for STM32 Virtual Com Port")
# TODO arg_parser.add_argument("--data", type=str, default="cmsfr", help="Specify which combined data to acquire and plot (c=RC, m=motor, s=sensor, f=states, r=reference")
cli_args = arg_parser.parse_args()

pprint("starting ...")
dprint("given args: interval_ms:%d duration_s:%d com:%s" 
       % (cli_args.interval_ms, cli_args.duration_s, cli_args.com))
dprint("imports successful, Captain!")

fcb_serial = 0;
do_exit = False;

rc_sema = threading.Condition()
motors_sema = threading.Condition()
states_sema = threading.Condition()
refs_sema = threading.Condition()
ctrl_sema = threading.Condition()

NBR_DISPLAYED_SAMPLES = 100

DATA_HEADER_SIZE = 7 # Will be 7: 1 byte ID, 4 bytes CRC, 2 bytes msg length

# RC DATA
rcthro_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)
rcaile_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)
rcelev_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)
rcrudd_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)

rc_time_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)

RC_VALUE_SCALING = 100.0/32767.0

# MOTORS DATA
m1_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)
m2_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)
m3_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)
m4_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)

motors_time_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)

MOTOR_VALUE_SCALING = 100.0/65535.0

# STATES DATA
roll_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES) # append & pop from opposite ends is thread safe according to .py doc 
pitch_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES) # append & pop from opposite ends is thread safe according to .py doc 
yaw_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES) # append & pop from opposite ends is thread safe according to .py doc 
rollrate_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES) # append & pop from opposite ends is thread safe according to .py doc 
pitchrate_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES) # append & pop from opposite ends is thread safe according to .py doc 
yawrate_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES) # append & pop from opposite ends is thread safe according to .py doc 

states_time_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)

RAD_TO_DEG_FACTOR = 180/math.pi

# REFERENCE SIGNALS DATA
rollref_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)
pitchref_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)
yawref_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)
yawrateref_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)

ref_time_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)

# CONTROL SIGNALS DATA
ctrlthrust_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)
ctrlroll_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)
ctrlpitch_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)
ctrlyaw_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)

ctrl_time_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)

start_time = timeit.default_timer()

logfile = open('logfile', 'w')

def update_rc_plot_data(new_thro, new_aile, new_elev, new_rudd):
    rc_sema.acquire(blocking = 1)
    
    rcthro_data.append(new_thro*RC_VALUE_SCALING)
    datatoplot = rcthro_data.popleft()
    rcaile_data.append(new_aile*RC_VALUE_SCALING)
    datatoplot = rcaile_data.popleft()
    rcelev_data.append(new_elev*RC_VALUE_SCALING)
    datatoplot = rcelev_data.popleft()
    rcrudd_data.append(new_rudd*RC_VALUE_SCALING)
    datatoplot = rcrudd_data.popleft()
    
    rc_time_data.append(timeit.default_timer()-start_time)
    datatoplot = rc_time_data.popleft()
    
    rc_sema.notify() # notify plot that data is available
    rc_sema.release() # TODO use in plotStatesUpdate()

def update_motor_plot_data(new_M1, new_M2, new_M3, new_M4):
    motors_sema.acquire(blocking = 1)
    
    m1_data.append(new_M1*MOTOR_VALUE_SCALING)
    datatoplot = m1_data.popleft()
    m2_data.append(new_M2*MOTOR_VALUE_SCALING)
    datatoplot = m2_data.popleft()
    m3_data.append(new_M3*MOTOR_VALUE_SCALING)
    datatoplot = m3_data.popleft()
    m4_data.append(new_M4*MOTOR_VALUE_SCALING)
    datatoplot = m4_data.popleft()
    
    motors_time_data.append(timeit.default_timer()-start_time)
    datatoplot = motors_time_data.popleft()
    
    motors_sema.notify() # notify plot that data is available
    motors_sema.release() # TODO use in plotStatesUpdate()

def update_states_plot_data(new_roll_data, new_pitch_data, new_yaw_data, new_rollrate_data, new_pitchrate_data, new_yawrate_data):
    states_sema.acquire(blocking = 1)
    
    roll_data.append(new_roll_data*RAD_TO_DEG_FACTOR)
    datatoplot = roll_data.popleft() # only want 100 values to plot at any one time
    pitch_data.append(new_pitch_data*RAD_TO_DEG_FACTOR)
    datatoplot = pitch_data.popleft() # only want 100 values to plot at any one time    
    yaw_data.append(new_yaw_data*RAD_TO_DEG_FACTOR)
    datatoplot = yaw_data.popleft() # only want 100 values to plot at any one time    
    
    rollrate_data.append(new_rollrate_data*RAD_TO_DEG_FACTOR)
    datatoplot = rollrate_data.popleft() # only want 100 values to plot at any one time
    pitchrate_data.append(new_pitchrate_data*RAD_TO_DEG_FACTOR)
    datatoplot = pitchrate_data.popleft() # only want 100 values to plot at any one time    
    yawrate_data.append(new_yawrate_data*RAD_TO_DEG_FACTOR)
    datatoplot = yawrate_data.popleft() # only want 100 values to plot at any one time   
    
    states_time_data.append(timeit.default_timer()-start_time)
    datatoplot = states_time_data.popleft()
    
    states_sema.notify() # notify plot that data is available
    states_sema.release() # TODO use in plotStatesUpdate()
    
def update_ref_plot_data(new_rollref_data, new_pitchref_data, new_yawref_data, new_yawrateref_data):
    refs_sema.acquire(blocking = 1)
    
    rollref_data.append(new_rollref_data*RAD_TO_DEG_FACTOR)
    datatoplot = rollref_data.popleft()
    pitchref_data.append(new_pitchref_data*RAD_TO_DEG_FACTOR)
    datatoplot = pitchref_data.popleft()  
    yawref_data.append(new_yawref_data*RAD_TO_DEG_FACTOR)
    datatoplot = yawref_data.popleft()
    yawrateref_data.append(new_yawrateref_data*RAD_TO_DEG_FACTOR)
    datatoplot = yawrateref_data.popleft()
    
    ref_time_data.append(timeit.default_timer()-start_time)
    datatoplot = ref_time_data.popleft()
    
    refs_sema.notify() # notify plot that data is available
    refs_sema.release() # TODO use in plotStatesUpdate()
    
def update_ctrl_plot_data(new_thrustctrl_data, new_rollctrl_data, new_pitchctrl_data, new_yawctrl_data):
    ctrl_sema.acquire(blocking = 1)
    
    ctrlthrust_data.append(new_thrustctrl_data)
    datatoplot = ctrlthrust_data.popleft()
    ctrlroll_data.append(new_rollctrl_data)
    datatoplot = ctrlroll_data.popleft()  
    ctrlpitch_data.append(new_pitchctrl_data)
    datatoplot = ctrlpitch_data.popleft()
    ctrlyaw_data.append(new_yawctrl_data)
    datatoplot = ctrlyaw_data.popleft()
    
    ctrl_time_data.append(timeit.default_timer()-start_time)
    datatoplot = ctrl_time_data.popleft()
    
    ctrl_sema.notify() # notify plot that data is available
    ctrl_sema.release() # TODO use in plotStatesUpdate()

def parse_received_data(msg_type_id, msg_data_len, byte_buffer):
    ERROR_MSG_ID = 0
    RC_MSG_ID = 1
    MOTOR_MSG_ID = 2
    SENSOR_MSG_ID = 3
    STATES_MSG_ID = 4
    REFSIGNAL_MSG_ID = 6
    CTRLSIGNAL_MSG_ID = 8

    rc = dragonfly_fcb_pb2.ReceiverSignalValuesProto()
    motor = dragonfly_fcb_pb2.MotorSignalValuesProto()
    # sensor = dragonfly_fcb_pb2.SensorSamplesProto()
    state = dragonfly_fcb_pb2.FlightStatesProto()
    refsignals = dragonfly_fcb_pb2.ControlReferenceSignalsProto()
    ctrlsignals = dragonfly_fcb_pb2.ControlSignalsProto()

    if msg_type_id      == MOTOR_MSG_ID:
        try:
    	    motor.ParseFromString(str(byte_buffer[DATA_HEADER_SIZE : DATA_HEADER_SIZE+msg_data_len]))
        except google.protobuf.message.DecodeError as de:
            pprint("motors sample discarded: %s" % (de.__str__()))
            fcb_serial.reset_input_buffer() # Flush input
            #continue
        
        update_motor_plot_data(motor.M1, motor.M2, motor.M3, motor.M4)
        logfile.write('{0}, {1}, {2}, {3}, '.format(motor.M1, motor.M2, motor.M3, motor.M4))
    elif msg_type_id    == STATES_MSG_ID:
        try:
            state.ParseFromString(str(byte_buffer[DATA_HEADER_SIZE : DATA_HEADER_SIZE+msg_data_len]))
        except google.protobuf.message.DecodeError as de:
            pprint("states sample discarded: %s" % (de.__str__()))
            fcb_serial.reset_input_buffer() # Flush input

        update_states_plot_data(state.rollAngle, state.pitchAngle, state.yawAngle, state.rollRate, state.pitchRate, state.yawRate)
        logfile.write('{0}, {1}, {2}, {3}, {4}, {5}, '.format(state.rollAngle, state.pitchAngle, state.yawAngle, state.rollRate, state.pitchRate, state.yawRate))
    elif msg_type_id    == RC_MSG_ID:
        try:
            rc.ParseFromString(str(byte_buffer[DATA_HEADER_SIZE : DATA_HEADER_SIZE+msg_data_len]))
        except google.protobuf.message.DecodeError as de:
            pprint("RC sample discarded: %s" % (de.__str__()))
            fcb_serial.reset_input_buffer() # Flush input
            #continue

        update_rc_plot_data(rc.throttle, rc.aileron, rc.elevator, rc.rudder)
        logfile.write('{0}, {1}, {2}, {3}, '.format(rc.throttle, rc.aileron, rc.elevator, rc.rudder))
        
    elif msg_type_id    == REFSIGNAL_MSG_ID:
        try:
            refsignals.ParseFromString(str(byte_buffer[DATA_HEADER_SIZE : DATA_HEADER_SIZE+msg_data_len]))
        except google.protobuf.message.DecodeError as de:
            pprint("Ref signal sample discarded: %s" % (de.__str__()))
            fcb_serial.reset_input_buffer() # Flush input
            #continue

        update_ref_plot_data(refsignals.refRoll, refsignals.refPitch, refsignals.refYaw, refsignals.refYawRate)
        logfile.write('{0}, {1}, {2}, {3}, '.format(refsignals.refRoll, refsignals.refPitch, refsignals.refYaw, refsignals.refYawRate))
        
    elif msg_type_id    == CTRLSIGNAL_MSG_ID:
        try:
            ctrlsignals.ParseFromString(str(byte_buffer[DATA_HEADER_SIZE : DATA_HEADER_SIZE+msg_data_len]))
        except google.protobuf.message.DecodeError as de:
            pprint("Ctrl signal sample discarded: %s" % (de.__str__()))
            fcb_serial.reset_input_buffer() # Flush input
            #continue

        update_ctrl_plot_data(ctrlsignals.thrustCtrl, ctrlsignals.rollCtrl, ctrlsignals.pitchCtrl, ctrlsignals.yawCtrl)
        logfile.write('{0}, {1}, {2}, {3}\n'.format(ctrlsignals.thrustCtrl, ctrlsignals.rollCtrl, ctrlsignals.pitchCtrl, ctrlsignals.yawCtrl))

def receive_data(last_send_time):
    byte_buffer = bytearray()
    byte_cnt = 0
    data = ""
    msg_type_id = -1
    msg_data_len = -1 # Holds data payload byte length
    msg_data_crc = -1

    # header + payload + 2 bytes msg end chars \n\r, TODO check for valid size and msg id
    while ((byte_cnt < msg_data_len+DATA_HEADER_SIZE+2 and msg_data_len > 0) or (msg_data_len < 0)): # and data != '\r')):
        new_time = timeit.default_timer()
        if(new_time >= last_send_time + cli_args.interval_ms/1000.0):
            pprint("Read timeout, flushing buffer.  Received nbr of bytes %d", byte_cnt)
            fcb_serial.reset_input_buffer() # Flush input if timed out
            break
        try:
            data = fcb_serial.read(1);
        except serial.SerialException as se:
            pprint("serial.SerialException: %s" % (se.__str__()))
            do_exit = True
            break
        if data != "":
            byte_buffer.append(data[0])
            byte_cnt += 1
            #line += str(data)
            if byte_cnt == DATA_HEADER_SIZE:
                msg_type_id = byte_buffer[0]
                msg_data_crc = byte_buffer[1:5]
                msg_data_len = byte_buffer[6]*256 + byte_buffer[5]
        else:
            pprint("No datat received")

    return msg_type_id, msg_data_len, msg_data_crc, byte_buffer, byte_cnt


def handle_received_message(last_send_time):
    #last_send_time = timeit.default_timer()
    #time.sleep(0.005)

    msg_type_id, msg_data_len, msg_data_crc, byte_buffer, byte_cnt = receive_data(last_send_time)

    if byte_cnt > DATA_HEADER_SIZE and msg_type_id >= 0 and byte_cnt >= msg_data_len:
        parse_received_data(msg_type_id, msg_data_len, byte_buffer)
    else:
        pprint("%s received: %s", comReader.__name__, byte_buffer)

        for x in range(0, byte_cnt):
            pprint("%d, ", byte_buffer[x])

        fcb_serial.reset_input_buffer() # Flush input


# handy tip, type: python -m serial.tools.list_ports
# to list available COM ports at the terminal.

def comReader(duration_s):
    global do_exit
    pprint("%s starting" % comReader.__name__)
    dprint("comReader is executing in: " + threading.currentThread().name)

    logfile.write('motor1, motor2, motor3, motor4, throttle, aileron, elevator, rudder, rollAngle, pitchAngle, yawAngle, rollRate, pitchRate, yawRate, refRoll, refPitch, refYaw, refYawRate, thrustCtrl, rollCtrl, pitchCtrl, yawCtrl\n')
       
    i = 0;
    
    byte_buffer = bytearray()
        
    last_send_time = start_time
    end_time = start_time + duration_s
    now_time = 0

    while (now_time < end_time and not do_exit):
        
        now_time = timeit.default_timer()
        
        # Sleep, then send requests for new data from FCB
        time.sleep(cli_args.interval_ms/1000.0)
        
        msg_sent = False
        msg_cnt = 0
        
        
        # Get motor values
        try:
            fcb_serial.write("get-motors p\r")
            msg_sent = True
            msg_cnt += 1
        except serial.SerialTimeoutException as ste:
            fcb_serial.close()
            pprint(ste.__str__())
            quit()
        
        #handle_received_message()    
        time.sleep(0.002)
        
        # Get RC values
        try:
            fcb_serial.write("get-receiver p\r")
            msg_sent = True
            msg_cnt += 1
        except serial.SerialTimeoutException as ste:
            fcb_serial.close()
            pprint(ste.__str__())
            quit()
            
        #handle_received_message()    
        time.sleep(0.002)
            
        # Get state values
        try:
            fcb_serial.write("get-states p\r")
            msg_sent = True
            msg_cnt += 1
        except serial.SerialTimeoutException as ste:
            fcb_serial.close()
            pprint(ste.__str__())
            quit()
        
        #handle_received_message()    
        time.sleep(0.002)
        
        # Get reference signal values
        try:
            fcb_serial.write("get-ref-signals p\r")
            msg_sent = True
            msg_cnt += 1
        except serial.SerialTimeoutException as ste:
            fcb_serial.close()
            pprint(ste.__str__())
            quit()
            
        #handle_received_message()    
        time.sleep(0.002)
        
        # Get control signal values
        try:
            fcb_serial.write("get-ctrl-signals p\r")
            msg_sent = True
            msg_cnt += 1
        except serial.SerialTimeoutException as ste:
            fcb_serial.close()
            pprint(ste.__str__())
            quit()
            
        #handle_received_message()    
        if msg_sent:
            last_send_time = timeit.default_timer()

        time.sleep(0.001)
        
# TODO real CRC Check

# TODO get max proto message sizes for each message from nanopb-generated proto (copy-paste them here)
# Use regex / parsing from C header to get these dynamically?

        while msg_cnt > 0:
            handle_received_message(last_send_time)
            #msg_type_id, msg_data_len, msg_data_crc, byte_buffer, byte_cnt = receive_data(last_send_time)
            
            #if byte_cnt > DATA_HEADER_SIZE and msg_type_id >= 0 and byte_cnt >= msg_data_len:
            #    parse_received_data(msg_type_id, msg_data_len, byte_buffer)
            #else:
            #    pprint("%s received: %s", comReader.__name__, byte_buffer)

            #    for x in range(0, byte_cnt):
            #        pprint("%d, ", byte_buffer[x])

            #    fcb_serial.reset_input_buffer() # Flush input
                
            msg_cnt -= 1

    fcb_serial.close()
    pprint("%s done - close graphics window to exit" % comReader.__name__)
    
    logfile.close()


#### Graphics objects & helper functions

app = QtGui.QApplication([])

win = pg.GraphicsWindow(title="Flight states plot")
win.resize(1000, 600)
win.setWindowTitle('Flight states plot')
pg.setConfigOptions(antialias=True) # Enable antialiasing for prettier plots

# TODO Concatenate all angles to one, and all angle rated to one

angle_plot = win.addPlot()
angle_plot.setLabel('bottom', "Time [s]")
angle_plot.setLabel('left', "Rotation angle [deg]")
roll_plot_curve = angle_plot.plot(pen='r', name='Roll')
pitch_plot_curve = angle_plot.plot(pen='g', name='Pitch')
yaw_plot_curve = angle_plot.plot(pen='y', name='Yaw')
angle_plot.showGrid(x=True, y=True)
angle_plot.setYRange(-180, 180)

motor_plot = win.addPlot()
motor_plot.setLabel('bottom', "Time [s]")
motor_plot.setLabel('left', "Motor signal [%]")
m1_curve = motor_plot.plot(pen='r', name='Motor 1')
m2_curve = motor_plot.plot(pen='g', name='Motor 2')
m3_curve = motor_plot.plot(pen='b', name='Motor 3')
m4_curve = motor_plot.plot(pen='y', name='Motor 4')
motor_plot.showGrid(x=True, y=True)
motor_plot.setYRange(0, 100)

win.nextRow()

anglerate_plot = win.addPlot()
anglerate_plot.setLabel('bottom', "Time [s]")
anglerate_plot.setLabel('left', "Rotational rate [deg/s]")
rollrate_plot_curve = anglerate_plot.plot(pen='r', name='Roll rate')
pitchrate_plot_curve = anglerate_plot.plot(pen='g', name='Pitch rate')
yawrate_plot_curve = anglerate_plot.plot(pen='y', name='Yaw rate')
anglerate_plot.showGrid(x=True, y=True)
anglerate_plot.setYRange(-180, 180)

rc_plot = win.addPlot()
rc_plot.setLabel('bottom', "Time [s]")
rc_plot.setLabel('left', "RC control [%]")
rcthro_curve = rc_plot.plot(pen='r', name='Throttle')
rcaile_curve = rc_plot.plot(pen='g', name='Aileron')
rcelev_curve = rc_plot.plot(pen='b', name='Elevator')
rcrudd_curve = rc_plot.plot(pen='y', name='Rudder')
rc_plot.showGrid(x=True, y=True)
rc_plot.setYRange(-180, 180)

win.nextRow()

ref_plot = win.addPlot()
ref_plot.setLabel('bottom', "Time [s]")
ref_plot.setLabel('left', "Reference signals [deg, deg/s]")
rollref_curve = ref_plot.plot(pen='r', name='Roll ref')
pitchref_curve = ref_plot.plot(pen='g', name='Pitch ref')
yawref_curve = ref_plot.plot(pen='b', name='Yaw ref')
yawrateref_curve = ref_plot.plot(pen='y', name='Yaw rate ref')
ref_plot.showGrid(x=True, y=True)
ref_plot.setYRange(-35, 35)

ctrl_plot = win.addPlot()
ctrl_plot.setLabel('bottom', "Time [s]")
ctrl_plot.setLabel('left', "Control signals [N, Nm]")
ctrlthrust_curve = ctrl_plot.plot(pen='r', name='Thrust force')
ctrlroll_curve = ctrl_plot.plot(pen='g', name='Roll moment')
ctrlpitch_curve = ctrl_plot.plot(pen='b', name='Pitch moment')
ctrlyaw_curve = ctrl_plot.plot(pen='y', name='Yaw moment')
ctrl_plot.showGrid(x=True, y=True)
ctrl_plot.setYRange(-.5, .5)


def plotUpdate():
    
    plotRCUpdate()
    plotMotorsUpdate()
    #plotSensorsUpdate()
    plotStatesUpdate()
    plotRefSignalsUpdate()
    plotCtrlSignalsUpdate()

def plotRCUpdate():
    global ptr, rc_plot

    rcthro_curve.setData(rc_time_data, rcthro_data)
    rcaile_curve.setData(rc_time_data, rcaile_data)
    rcelev_curve.setData(rc_time_data, rcelev_data)
    rcrudd_curve.setData(rc_time_data, rcrudd_data)
    
def plotRefSignalsUpdate():
    global ptr, ref_plot

    rollref_curve.setData(ref_time_data, rollref_data)
    pitchref_curve.setData(ref_time_data, pitchref_data)
    yawref_curve.setData(ref_time_data, yawref_data)
    yawrateref_curve.setData(ref_time_data, yawrateref_data)

def plotCtrlSignalsUpdate():
    global ptr, ctrl_plot

    ctrlthrust_curve.setData(ctrl_time_data, ctrlthrust_data)
    ctrlroll_curve.setData(ctrl_time_data, ctrlroll_data)
    ctrlpitch_curve.setData(ctrl_time_data, ctrlpitch_data)
    ctrlyaw_curve.setData(ctrl_time_data, ctrlyaw_data)

def plotMotorsUpdate():
    global ptr, motor_plot

    m1_curve.setData(motors_time_data, m1_data)
    m2_curve.setData(motors_time_data, m2_data)
    m3_curve.setData(motors_time_data, m3_data)
    m4_curve.setData(motors_time_data, m4_data)
    
def plotStatesUpdate():
    global ptr, angle_plot, anglerate_plot
    
    roll_plot_curve.setData(states_time_data, roll_data)
    pitch_plot_curve.setData(states_time_data, pitch_data)
    yaw_plot_curve.setData(states_time_data, yaw_data)
    
    rollrate_plot_curve.setData(states_time_data, rollrate_data)
    pitchrate_plot_curve.setData(states_time_data, pitchrate_data)
    yawrate_plot_curve.setData(states_time_data, yawrate_data)


interval_s = cli_args.interval_ms/float(1000)
duration_s = int(cli_args.duration_s)

timer = QtCore.QTimer()
timer.timeout.connect(plotUpdate)
timer.start(10)

fcb_serial = serial.Serial(cli_args.com, 115200, parity=serial.PARITY_NONE, rtscts=False, xonxoff=False, timeout=1, write_timeout=2)
myComReaderThread = threading.Thread(target=comReader, name="tComReader", args=(duration_s,))
myComReaderThread.daemon = True
myComReaderThread.start()
#fcb_serial.write("about\r") # data does not show up when removing this line ... work-around
time.sleep(1)

#try:
    #fcb_serial.write("start-state-sampling %d %d p" % (cli_args.interval_ms, cli_args.duration_s) + "\r")
#except serial.SerialTimeoutException as ste:
    #fcb_serial.close()
    #pprint(ste.__str__())
    #quit()

if __name__ == '__main__':
    ## Start Qt event loop unless running in interactive mode or using pyside.
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        pprint("Close window to exit program")
        QtGui.QApplication.instance().exec_()
        do_exit = True

myComReaderThread.join(cli_args.duration_s + 1)
if myComReaderThread.isAlive():
    do_exit = True
    fcb_serial.close()
    time.sleep(interval_s)
    
pprint("Please wait for program to exit")
if fcb_serial.is_open:
    fcb_serial.close()

