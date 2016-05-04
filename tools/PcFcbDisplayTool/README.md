# README for PcFcbTool.py

## Python installation

### For Windows:
* General python installation:See Dragonfly-FCB Wiki page "ToolTipsTricks", section "Python" and follow installation instructions.
* Open `cmd.exe`
* Install library dependencies: `C:\INSTALLATION_PATH\python.exe -m pip install pyserial numpy protobuf pyside pyqtgraph`
 * If the `error: Microsoft Visual C++ 9.0 is required (Unable to find vcvarsall.bat)` occurs, Get it from http://aka.ms/vcpython27
* Run `C:\INSTALLATION_PATH\python -m pip install pyside pyqtgraph`
* now it should be possible to run `C:\INSTALLATION_PATH\python.exe PcFcbTool.py 100 60` in `cmd.exe`.

NOTE: There has been some difficulties with the pyqtgraph plot window in Linux where axes are not showing correctly.

### For Linux:
* `sudo apt-get install python-dev python-pip`
* `sudo python -m pip install pyserial numpy protobuf pyside pyqtgraph crcmod`

Troubleshooting: Try to update the packages if they do not seem to be working correctly. Example:
sudo python -m pip install pyserial --upgrade pyserial

### Protobuf package
You may get an earlier version causing "syntax variable" errors when installing the default package with pip.

sudo pip install protobuf==3.0.0b2

## PcFcbTool usage
* Connect the FCB to the Windows 7 PC using a micro USB cable, it is assumed that "COM8" is the port.
* run `python PcFcbTool.py 200 30` to receive a graph which is sampled every 200 ms for 30 seconds.
* Close the graph window to exit the program.
* use the `--help` option to get usage instructions
* use the `--com PORT` to use COM port PORT for STM32 Virtual COM port (e.g. "COM2" in Windows or "/dev/ttyACM0" in Linux.

## Developing PcFcbDisplayTool
* In order to regenerate `dragonfly_fcb_pb2.py`:
 * Download `protoc-3.0.0-beta-2-win32.zip` from: https://github.com/google/protobuf/releases extract `protoc.exe` and make sure it is in the `Path` of `cmd.exe`
 * Then run: `protoc --plugin=generator\protoc-gen-nanopb.bat --proto_path=REPO_PATH\fcb-source\communication\protobuf\ REPO_PATH\fcb-source\communication\protobuf\dragonfly_fcb.proto  --python_out=REPO_PATH\Tools\fcb-tools\PcFcbDisplayTool
