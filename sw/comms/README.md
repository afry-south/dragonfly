
# Generate source & header files from the .proto file

Put "fcb\nanopb-0.3.3-windows-x86\generator-bin¨ from this cloned repository into
your PATH.

Cd to the directory of this README file, then execute:

protoc  --nanopb_out=. fcb-fsm.proto

# Usage
## Windows
Clone directory to PC workstation directory (example) CLONEDIR.

Then use the "subst" windows cmd.exe to make this into a drive letter:
     subst I:\ DRIVE:\CLONEDIR

The project files will use I:\ paths to build. In this way, a user may
clone a directory anywhere on the harddrive.

# Notes
* For more on the Protocol Buffers message de/serialisation utility, see here: https://developers.google.com/protocol-buffers/
