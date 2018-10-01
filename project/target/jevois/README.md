JeVois
======

## First time setup:

1. We install jevois libraries + SDK on your pc. This installs the packages so that we can create projects and flash the code to jevois.

```
sudo apt install jevois-host jevois-opencv jevois-platform jevois-sdk jevois-sdk-dev

```

2. We want to use the serial port for mavlink and also be able to see debug messages. That's why we had to change the basic jevois code.
   Hence, we compile our adapted jevois code with activated debug prints, first for desktop:
   ! If you don't have the folder you need to first get the submodules run ```git submodule update --init --recursive```!
   If you don't want to test code on the laptop this step can be skipped.
```
cd dronerace2018/lib/jevois
./rebuild-host.sh -D JEVOIS_LDEBUG_ENABLE=On -D JEVOIS_TRACE_ENABLE=On

```

3. The same thing we do for the jevois itsself. So we first mount it as usb then flash it.
```
jevois-cmd usbsd
./rebuild-platform.sh -D JEVOIS_LDEBUG_ENABLE=On -D JEVOIS_TRACE_ENABLE=On --microsd
```


4. Now we can flash our own code:
```
cd dronerace2018/target/jevois
./rebuild-platform.sh --microsd
```

5. And replace the config files. So copy the folder from target/jevois/config on your jevois.

6. Done! restart and start gucviewer to start streaming.

```
jevois-cmd restart
```

## Typical Dev Process:

### Running code on your PC/Laptop:

After changing code you can the compile the code for your laptop with:

```
./rebuild-host.sh
```

Then you can test it with a webcam by running a virtual jevois:
```
jevois-daemon 
```
Find the mapping of your module by typing in the newly opened terminal:
```
listmappings
```
Then restart the daemon with your mapping:
```
quit
jevois-daemon --videomapping=XX
```

### Running code on the Jevois:

There are multiple ways to flash the code. The most stable so far proofed to be:

```
jevois-cmd usbsd #This should mount the jevois as sdcard, video streaming has to be turned off
./rebuild-platform --microsd #This compiles the code and copies it to the jevois
jevois-cmd restart
```
Sometimes the jevois gets mounted in the wrong place e.g. /media/$USER/JEVOIS1 while the build script copies the code always to /media/$USER/JEVOIS. So double check this when sth is not working.

After a short reboot the code should run on the jevois. Test it with GTK UVC Viewer, you might have to select the appropriate resolution. If your module is not selected you can do it manually with:


```
jevois-cmd setmapping XX #video streaming has to be turned off
```

Prints from the jevois should be visible when reading the serial port e.g.:
```
screen /dev/ttyACM0
```
Note that while screen is running you can't send commands with jevois-cmd.

## Config Files

There are several config files for the jevois. They can be found after mounting the jevois in the /config folder.
This folder is also added to the git repo to be found in target/jevois/config. Preferably if anything important is changed on the jevois this is also changed in the folder on the repo. So that we can always just copy this folder to the jevois and everything is working.
The folder contains:
```
initscript.cfg    -->      The commands in this file are executed when the jevois is started. They are the same commands that can be sent with "jevois-cmd"
                           We use this file to select our module at startup and configure the ports: The usb port is used for debugging messages, the serial port is used for mavlink.
params.cfg        -->      This contains parameter settings. Currently we don't use this file.

videomappings.cfg -->      This file contains the videomappings. So for each module that is installed on the jevois there has to be a corresponding mapping that tells it which resolution, fps and color mode to use.
                           The file is structured as <PC Mapping> <Camera Mapping> <Vendor Name> <Module Name>.
                           If we want a module to run without PC we set it to None 0 0 0.0.
                           The vendor name for all our modules should be "MavLab".
```

## Other useful commands

```
jevois-cmd ping 			#Test connection
jevois-cmd listmappings 	#List available mappings
jevois-cmd info 			#Print current state e.g selected module
```

If a module crashes right after startup it might not be possible to talk to the jevois anymore. In that case the only way out is mounting the sd with an adapter and manually changing videomappings.cfg or deleting the buggy module.