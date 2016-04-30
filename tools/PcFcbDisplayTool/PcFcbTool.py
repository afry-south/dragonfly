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
arg_parser.add_argument("--com", type=str, default="dev/ttyACM0", help="give the name of the COM port for STM32 Virtual Com Port")
cli_args = arg_parser.parse_args()

pprint("starting ...")
dprint("given args: interval_ms:%d duration_s:%d com:%s" 
       % (cli_args.interval_ms, cli_args.duration_s, cli_args.com))
dprint("imports successful, Captain!")

fcb_serial = 0;
do_exit = False;
sema = threading.Condition()

NBR_DISPLAYED_SAMPLES = 100
roll_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES) # append & pop from opposite ends is thread safe according to .py doc 
pitch_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES) # append & pop from opposite ends is thread safe according to .py doc 
yaw_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES) # append & pop from opposite ends is thread safe according to .py doc 
rollrate_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES) # append & pop from opposite ends is thread safe according to .py doc 
pitchrate_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES) # append & pop from opposite ends is thread safe according to .py doc 
yawrate_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES) # append & pop from opposite ends is thread safe according to .py doc 

start_time = timeit.default_timer()
time_data = deque([float(0)]*NBR_DISPLAYED_SAMPLES)

RAD_TO_DEG_FACTOR = 180/math.pi

def update_plot_data(new_roll_data, new_pitch_data, new_yaw_data, new_rollrate_data, new_pitchrate_data, new_yawrate_data):
    sema.acquire(blocking = 1)
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
    
    time_data.append(timeit.default_timer()-start_time)
    datatoplot = time_data.popleft()
    
    sema.notify() # notify plot that data is available
    sema.release()

# handy tip, type: python -m serial.tools.list_ports
# to list available COM ports at the terminal.

def comReader(sample_nbr):
    global do_exit
    pprint("%s starting" % comReader.__name__)
    dprint("comReader is executing in: " + threading.currentThread().name)
    state = dragonfly_fcb_pb2.FlightStatesProto()
    i = 0;
    
    byte_buffer = bytearray()
    DATA_HEADER_SIZE = 7 # Will be 7: 1 byte ID, 4 bytes CRC, 2 bytes msg length
    STATES_MSG_ID = 4
    
    last_send_time = start_time
    now_time = 0

    while (i < sample_nbr and not do_exit):
        
        # Send request for new data from FCB
        # TODO maybe we should flush receive buffer here?
        
        time.sleep(cli_args.interval_ms/1000.0)
        
        try:
            print("HERE WRITE")
            last_send_time = timeit.default_timer()
            fcb_serial.write("get-states p\r")
        except serial.SerialTimeoutException as ste:
            fcb_serial.close()
            pprint(ste.__str__())
            quit()
        
        del byte_buffer[:] # Reset the buffer
        byte_cnt = 0
        data = ""
        msg_type_id = -1
        msg_data_len = -1 # Holds data payload byte length
        msg_data_crc = -1
        
# TODO Below should search for data header and then read out length bytes, not try to find \r or \n...
# TODO CRC Check
# TODO New data message encapsulation

        # header + payload + 2 bytes msg end chars \n\r, TODO check for valid size and msg id
        while ((byte_cnt < msg_data_len+DATA_HEADER_SIZE+2 and msg_data_len > 0) or (msg_data_len < 0 and data != '\r')):
            now_time = timeit.default_timer()
            if(now_time >= last_send_time):
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
        
        if do_exit:
            break
        
        i += 1;
        
        # Only parsing state values here (ID 4)
        print("msg ID: " + str(msg_type_id))
        print("msg_CRC: " + str(msg_data_crc))
        print("msg len: " + str(msg_data_len))
        print("")
        
        if byte_cnt > DATA_HEADER_SIZE and msg_type_id == STATES_MSG_ID and byte_cnt >= msg_data_len:
            try:
                state.ParseFromString(str(byte_buffer[DATA_HEADER_SIZE : DATA_HEADER_SIZE+msg_data_len])); # Data offset 4
            except google.protobuf.message.DecodeError as de:
                pprint("sample %d discarded: %s" % (i, de.__str__()))
                fcb_serial.reset_input_buffer() # Flush input
                continue
            
            # sys.stdout.write(state.__str__()); # useful for debugging
            update_plot_data(state.rollAngle, state.pitchAngle, state.yawAngle, state.rollRate, state.pitchRate, state.yawRate)
        else:
            pprint("%s received: %s", comReader.__name__, byte_buffer)
            fcb_serial.reset_input_buffer() # Flush input

    fcb_serial.close()
    pprint("%s done - close graphics window to exit" % comReader.__name__)


#### Graphics objects & helper functions

app = QtGui.QApplication([])

win = pg.GraphicsWindow(title="Basic plotting examples")
win.resize(1000, 600)
win.setWindowTitle('pyqtgraph example: Plotting')
pg.setConfigOptions(antialias=True) # Enable antialiasing for prettier plots

roll_plot = win.addPlot()
roll_plot.setLabel('bottom', "Time [s]")
roll_plot.setLabel('left', "Roll angle [deg]")
roll_plot_curve = roll_plot.plot(pen='r', name='Roll [deg]')
roll_plot.showGrid(x=True, y=True)
roll_plot.setYRange(-180, 180)

rollrate_plot = win.addPlot()
rollrate_plot.setLabel('bottom', "Time [s]")
rollrate_plot.setLabel('left', "Roll rate [deg/s]")
rollrate_plot_curve = rollrate_plot.plot(pen='r', name='Roll rate [deg/s]')
rollrate_plot.showGrid(x=True, y=True)
rollrate_plot.setYRange(-180, 180)

win.nextRow()
pitch_plot = win.addPlot()
pitch_plot.setLabel('bottom', "Time [s]")
pitch_plot.setLabel('left', "Pitch angle [deg]")
pitch_plot_curve = pitch_plot.plot(pen='g', name='Pitch [deg]')
pitch_plot.showGrid(x=True, y=True)
pitch_plot.setYRange(-180, 180)

pitchrate_plot = win.addPlot()
pitchrate_plot.setLabel('bottom', "Time [s]")
pitchrate_plot.setLabel('left', "Pitch rate [deg/s]")
pitchrate_plot_curve = pitchrate_plot.plot(pen='g', name='Pitch rate [deg/s]')
pitchrate_plot.showGrid(x=True, y=True)
pitchrate_plot.setYRange(-180, 180)

win.nextRow()
yaw_plot = win.addPlot()
yaw_plot.setLabel('bottom', "Time [s]")
yaw_plot.setLabel('left', "Yaw angle [deg]")
yaw_plot_curve = yaw_plot.plot(pen='y', name='Yaw [deg]')
yaw_plot.showGrid(x=True, y=True)
yaw_plot.setYRange(-180, 180)

yawrate_plot = win.addPlot()
yawrate_plot.setLabel('bottom', "Time [s]")
yawrate_plot.setLabel('left', "Yaw rate [deg/s]")
yawrate_plot_curve = yawrate_plot.plot(pen='y', name='Yaw rate [deg/s]')
yawrate_plot.showGrid(x=True, y=True)
yawrate_plot.setYRange(-180, 180)

def plotUpdate():
    global ptr, roll_plot, pitch_plot, yaw_plot, rollrate_plot, pitchrate_plot, yawrate_plot
    
    roll_plot_curve.setData(time_data, roll_data)
    pitch_plot_curve.setData(time_data, pitch_data)
    yaw_plot_curve.setData(time_data, yaw_data)
    
    rollrate_plot_curve.setData(time_data, rollrate_data)
    pitchrate_plot_curve.setData(time_data, pitchrate_data)
    yawrate_plot_curve.setData(time_data, yawrate_data)


interval_s = cli_args.interval_ms/float(1000)
sample_nbr = int(float(cli_args.duration_s / interval_s))

timer = QtCore.QTimer()
timer.timeout.connect(plotUpdate)
timer.start(10)

fcb_serial = serial.Serial(cli_args.com, 115200, parity=serial.PARITY_NONE, rtscts=False, xonxoff=False, timeout=1, write_timeout=1)
myComReaderThread = threading.Thread(target=comReader, name="tComReader", args=(sample_nbr,))
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

