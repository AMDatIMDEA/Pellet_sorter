# Example using pellet sorter

This notebook contains an example of how to run the code for collecting the material. Most cells can be run without any modification if the collector has been built without design modifications. The lines that need modifications are markedlike this:

`###############################################################################################################`



```python
# Import Libraries
import serial
import numpy as np
import time
import os
import serial.tools.list_ports
```

In this cell, we define the port name for the arduino and the balance. We do it by listing the connected ports and selecting the ones whose device serial is the corresponding to our device. You should find the serial of your devices and change it accordingly. You can do it by inspecting the devices detected by serial.tools.list_ports.comports(). In case of doubt, try disconnecting and connecting to see which ports are affected.


```python
ports = serial.tools.list_ports.comports()

###############################################################################################################

arduino_serial = "write_here_your_arduino_serial"
balance_serial = "write_here_your_balance_serial"

###############################################################################################################

for port in ports:
    if port.serial_number == arduino_serial:
        arduino_port = port.device
    elif port.serial_number == balance_serial:
        balance_port = port.device
        
### Serial initialization
## Arduino
arduino = serial.Serial(arduino_port, 9600)
## Balance
balance = serial.Serial(balance_port, 9600)
```

We import the functions from the pellet_sorter.py script. Note: you need to install the `ikpy` library


```python
import pellet_sorter as ps
```

Define the capacity of the cup in kg


```python
###############################################################################################################
cup_capacity = 0.16
###############################################################################################################
```

The next cell defines a series of parameters needed for the calculations. They don't have to be modified unless you have modified something in the design.


```python
### Initializations
# sorter_chain, sorter_parameters and arduino should be global variables

n_joints = 2
sorter_parameters = [{'steps_in_rotation':0,'gear_ratio':0,'length_mm':0,'bound_rad':0, 'cartesian':[0,0]} for k in range(n_joints)]
## Top
sorter_parameters[0]['steps_in_rotation'] = np.round(200*1.1)   # Calibrate multiplicative factor to compensate lost steps if necessary
sorter_parameters[0]['gear_ratio'] = 6
sorter_parameters[0]['length_mm'] = 180.5
sorter_parameters[0]['bound_rad'] = np.deg2rad(35)    # np.deg2rad(35)  # IK Rotation bound in either direction, not total
sorter_parameters[0]['cartesian'] = [sorter_parameters[0].get('length_mm'), 0]
## Bot
sorter_parameters[1]['steps_in_rotation'] = 812
sorter_parameters[1]['gear_ratio'] = 2.4
sorter_parameters[1]['length_mm'] = np.sqrt(32.7**2 + 79.2**2)  # 84.4
sorter_parameters[1]['bound_rad'] = np.deg2rad(10*360)
sorter_parameters[1]['cartesian'] = [sorter_parameters[1].get('length_mm'), 0]





# Positions of the cups in the tray

tray_1 = ps.Tray('1', [sorter_parameters[0]['length_mm'], 0], [
    ps.Slot([79.2,32.7]),
    ps.Slot([32.7,79.2]),
    ps.Slot([-32.7,79.2]),
    ps.Slot([-79.2,32.7]),
    ps.Slot([-79.2,-32.7]),
    ps.Slot([-32.7,-79.2]),
    ps.Slot([32.7,-79.2]),
    ps.Slot([79.2,-32.7]),
    ps.Slot([0,0]), 
    ])

tray_1.fill_all_slots_ez(0, cup_capacity)
tray_1.set_all_material('PLA####')
```

In the next cell, we use the Arduino Command Line Interface (arduino-cli) to compile and upload the arduino script. If you prefer to do it using the Graphical User Interface, skip this cell. For using it, update the location of your arduino-cli


```python
###############################################################################################################

os.system("/PATH/arduino-cli compile --fqbn arduino:avr:uno ../../Arduino/Pellet_sorter_v2/Pellet_sorter_v2.ino")
os.system("/PATH/arduino-cli upload -p "+arduino_port+" --fqbn arduino:avr:uno ../../Arduino/Pellet_sorter_v2/Pellet_sorter_v2.ino")

###############################################################################################################
```

The `fill_tray` function will run all the process. It uses the objects defined in the previous steps. The three parameters that allow interesting customization are: 

`:param seq: List with the order of filling of the cups. If None, they will all be filled in default order`

`:param purge_slot: Number of the slot used for purging. This slot can be left without cup so the material is discarded. Used when changing material. If None, no purging will be done.`
                        
`:param purge_time: Time for purge in minutes. Default: 0`

If `seq = None` and `purge_slot = None`, the cups will be filled sequentially (from 0 to 8) without any purging.

If `purge_slot` is not None, like `purge_slot = 0` for example, that hole will be used for purging. That hole should be left empty (without cup), and the tray should be in some kind of box or container so the pellets are not spilled everywhere. In that case, if seq is None the  purging will be made after each cup, so the cup 1 will be filled, then purge, then cup 2, then purge, etc.

For more detailed control, you can use `seq`. For example, if you want cups 1 and 2 filled with one material and cups 3 and 4 with another, you can use `purge_slot = 0` and `seq = [1,2,0,3,4,0]`. In this way, after filling cups 1 and 2 with the same material it will make a purge in slot 0, and the fill cups 3 and 4. 



```python
##################################################################################################################

ps.fill_tray(tray_1, sorter_parameters, arduino, balance, seq = None, purge_slot = 0, purge_time = 1)

###############################################################################################################
```

NOTE: As an alternative, you can use `fill_tray_SIMULATED` to check if the filling order is the desired


```python

ps.fill_tray_SIMULATED(tray_1, sorter_parameters, seq = None, purge_slot = 7, purge_time = 1)
```


```python

```
