## Ardupilot, MAVProxy and SITL installation in Ubuntu

### 1. Clone Ardupilot repository

Open a terminal and navigate to the directory where you would like the clone to be stored and then paste:

```
git clone git@github.com:ArduPilot/ardupilot.git
```

### 2. Install required packages

On the directory where you made the clone, go to the ardupilot directory:

```
cd ardupilot
```

Then paste:

```
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Reload the path with:
```
. ~/.profile
```

### 3. Build ArduCopter for SITL
From ardupilto directory:

```
./waf configure --board sitl           # software-in-the-loop simulator
./waf copter
```
### 4. Install MAVProxy
```
sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame
pip3 install PyYAML mavproxy --user
echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc
```
## Start SITL simulator
After the steps above you're now ready to run the SITL simulator.

Go to ardupilot dierectory and change to the vehicle directory:

```
cd ardupilot/ArduCopter
```

Then start the simulator using `sim_vehicle.py`.

The `first time you run` it you should use the -w option to wipe the virtual EEPROM and load the right default parameters for your vehicle:

```
sim_vehicle.py -w
```

If it's not the firs time, you can remove the -w option:

```
sim_vehicle.py
```

To kill any simulation, use `Ctrl + C`.

To run with a console and a map, use:
```
sim_vehicle.py --console --map
```

## Installing QGroundControl

Open a terminal and enter:

```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y
```

Logout and login again to enable the change to user permissions (Click on the power icon located in the top-right corner of the screen and then Log Out)

Now, click on this link to install the QGroundControl.AppImage
https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage

Move it to the directory of your preference and inside this repository enter:

```
chmod +x ./QGroundControl.AppImage
```

## SITL with QGroundControl

To run QGroundControl, go to the directory with the QGroundControl.AppImage file and enter:

```
./QGroundControl.AppImage
```

In another terminal, go to the `ardupilot/ArduCopter` directory and run the SITL:

```
sim_vehicle.py 
```

