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

### 3. Build ArduCopter
From ardupilto directory:

```
./waf configure --board CubeBlack
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
