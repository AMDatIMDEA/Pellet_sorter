#### PELLET DISTRIBUTOR SCRIPT
## Imports
import numpy as np
import math
from math import fabs, pi
import matplotlib.pyplot as plt
import ikpy.chain
import ikpy.link
import ikpy.utils.plot as plot_utils
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import serial
from dataclasses import dataclass
import time




### Pellet distributor functions
## Inverse kinematics
def pol2car(rho, phi):
    """
    Converts polar coordinates to cartesian

    :param rho: Float radius, modulus
    :param phi: Float argument, angle
    :return: Returns a list [x, y] of cartesian coordinates
    """

    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return[x, y]

def angle_to_pipi(input_angle):
    """
    Converts input angle to one in range [-pi, pi]

    :param input_angle: Float angle, any Real number
    :return: Returns a float angle in range [-pi, pi]
    """

    # revolutions = int((input_angle + np.sign(input_angle) * pi) / (2 * pi))
    p1 = truncated_remainder(input_angle + np.sign(input_angle) * pi, 2 * pi)
    p2 = (np.sign(np.sign(input_angle)
                  + 2 * (np.sign(fabs((truncated_remainder(input_angle + pi, 2 * pi))
                                      / (2 * pi))) - 1))) * pi
    output_angle = p1 - p2
    return output_angle #, revolutions

def angles_to_pipi(angles):
    """
    Converts list of angles to range [-pi, pi]

    :param input_angle: List of float angles
    :return: Returns a list of float angles in range [-pi, pi]
    """

    return list(map(angle_to_pipi, angles))

def truncated_remainder(dividend, divisor):
    """
    Gives truncated remainder of a division

    :param dividend: Float dividend
    :param divisor: Float divisor
    :return: Returns a float of the truncated remainder
    """
    divided_number = dividend / divisor
    divided_number = \
        -int(-divided_number) if divided_number < 0 else int(divided_number)
    remainder = dividend - divisor * divided_number
    return remainder

def distributor_chain_update(distributor_parameters):
    """
    Creates an IKPy Chain instance using the input parameters for the pellet distributor robot.
    Note that it currently ignores bounds in second joint

    :param distributor_parameters: Dictionary containing for every motor:
        Float/Int number of steps in a full revolution of the stepper motor
        Float gear ratio of the corresponding gear box
        Float length of the arm being rotated by the motor
        Float bound angle in radians such that the arm can only rotate within +-bound
        List [x, y] of floats of current cartesian coordinates of the end of the corresponding arm
        In the format: [{'steps_in_rotation':0,'gear_ratio':0,'length_mm':0,'bound_rad':0, 'cartesian':[0,0]} for k in range(n_joints)]
    :return: Returns the Chain instance for the distributor
    """
    distributor_chain = Chain(name='distributor', links=[
    OriginLink(),
    URDFLink(
        name="Base",
        bounds=(-distributor_parameters[0].get('bound_rad'),distributor_parameters[0].get('bound_rad')),
        origin_translation=[0, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1],
    ),
    URDFLink(
        name="Elbow",
        origin_translation=distributor_parameters[0].get('cartesian')+[0],
        origin_orientation=[0, 0, 1],
        rotation=[0, 0, 1],
    ),
    URDFLink(
        name="Tip",
        origin_translation=distributor_parameters[1].get('cartesian')+[0],
        origin_orientation=[0, 0, 1],
        rotation=[0, 0, 0],
    ),
    ])
    return distributor_chain

def IK_difference(cartesian_target, cartesian_initial, distributor_parameters, distributor_chain):
    """
    Gives the angle difference in radians of each joint for the poses corresponding to the initial and target tip coordinates for the pellet distributor robot

    :param cartesian_target: List [x, y] of floats corresponding to the target cartesian coordinates of the tip of the pellet distributor robot
    :param cartesian_initial: List [x, y] of floats corresponding to the initial cartesian coordinates of the tip of the pellet distributor robot
    :return: List of floats of the angle difference for each joint, in radians
    """
    joint_target = distributor_chain.inverse_kinematics(cartesian_target+[0])
    joint_initial = distributor_chain.inverse_kinematics(cartesian_initial+[0])
    return angles_to_pipi(joint_target[1:3]-joint_initial[1:3])

def rad2steps(theta, steps_in_rotation):
    """
    Converts angle in radians to the number of steps the motor needs to take to rotate by that angle

    :param theta: Float angle in radians
    :param steps_in_rotation: Float/int number of steps in a full revolution of the stepper motor
    :return: Int number of steps corresponding to the angle and motor
    """

    return (np.rint(theta*steps_in_rotation/(2*math.pi))).astype(int)

def IK_steps(cartesian_target, cartesian_initial, distributor_parameters, distributor_chain):
    """
    Gives number of steps each stepper motor must take to go from initial to target tip coordinates in the pellet distributor robot

    :param cartesian_target: List [x, y] of floats corresponding to the target cartesian coordinates of the tip of the pellet distributor robot
    :param cartesian_initial: List [x, y] of floats corresponding to the initial cartesian coordinates of the tip of the pellet distributor robot
    :return: List [n1, n2] of int of the number of steps each stepper motor must take for the pellet distributor robot to go from initial to target tip coordinates
    """

    angle_difference = IK_difference(cartesian_target, cartesian_initial, distributor_parameters, distributor_chain)
    steps = [0]*len(distributor_parameters)
    for n in range(len(distributor_parameters)):
        steps[n] = rad2steps(distributor_parameters[n].get('gear_ratio')*angle_difference[n], distributor_parameters[n].get('steps_in_rotation'))
    return steps

def IK_plot(cartesian_target, cartesian_initial, distributor_parameters, distributor_chain):
    """
    Plots the initial and target poses of the pellet distributor robot, given the tip coordinates, on the same plot

    :param cartesian_target: List [x, y] of floats corresponding to the target cartesian coordinates of the tip of the pellet distributor robot
    :param cartesian_initial: List [x, y] of floats corresponding to the initial cartesian coordinates of the tip of the pellet distributor robot
    """

    # %matplotlib inline
    fig, ax = plot_utils.init_3d_figure()
    ax.set_zlim(0, 1)
    renamed_distributor_chain = distributor_chain
    renamed_distributor_chain.name = 'Target'
    renamed_distributor_chain.plot(distributor_chain.inverse_kinematics(cartesian_target+[0]), ax)
    renamed_distributor_chain.name = 'Initial'
    renamed_distributor_chain.plot(distributor_chain.inverse_kinematics(cartesian_initial+[0]), ax)
    max_length = sum([distributor_parameters[n].get('length_mm') for n in range(len(distributor_parameters))])
    plt.xlim(-max_length, max_length)
    plt.ylim(-max_length, max_length)
    plt.legend()
    plt.show()



## Serial communications
def runSteps(steps, arduino):
    """
    Takes array of two ints, corresponding to number of steps for base and elbow motors, and requests to Arduino through serial
    The message is 10 chars: sddddsdddd where s is sign (0 or -) and d are the digits. Positive means CCW from top view if wired accordingly.

    :param steps: List [n1, n2] of int corresponding to the number of steps each stepper motor must take
    """ 

    number_len = 5
    message = str(steps[0]).zfill(number_len) + str(steps[1]).zfill(number_len)
    arduino.write(message.encode("utf-8"))

def goToCartesianFrom(cartesian_target, cartesian_initial, distributor_parameters, distributor_chain, arduino):
    """
    Requests Arduino through serial to run necessary steps for pellet distributor robot cartesian tip coordinates to go from initial to target

    :param cartesian_target: List [x, y] of floats corresponding to the target cartesian coordinates of the tip of the pellet distributor robot
    :param cartesian_initial: List [x, y] of floats corresponding to the initial cartesian coordinates of the tip of the pellet distributor robot
    """

    runSteps(IK_steps(cartesian_target, cartesian_initial, distributor_parameters, distributor_chain), arduino)

def runValve(state, arduino):
    """
    Requests Arduino through serial to open or close pellet distributor robot valve

    :param state: Bool, true for open valve and false for close valve
    """

    message = 'v'+str(int(state))
    arduino.write(message.encode("utf-8"))

def waitForSerialStr(string, arduino):
    """
    Reads Arduino serial until input string is found

    :param string: Str, line to wait for until read in Arduino serial, should not be terminated by any special charachers 
    """

    arduino.reset_input_buffer()
    waiting = True
    while waiting:
        line = arduino.readline()   # read a '\n' terminated line
        if line:
            decoded_line = line.decode().strip()
            print(decoded_line)
            if decoded_line == string:
                waiting = False

def waitUntilWeight(number, balance):
    """
    Reads Balance serial until stable weight larger than input number is found
    Note that the 'ASNG/W+ ' term might need to be changed depending of what mode balance operates in; monitor serial to find out (Eg: Net weight would be 'ASNN/W+ ')

    :param number: Float in kg to be surpassed by Balance reading
    """

    balance.reset_input_buffer()
    waiting = True
    while waiting:
        line = balance.readline()   # read a '\n' terminated line
        if line:
            decoded_line = line.decode('ascii').strip()
            print(decoded_line)
            if decoded_line[0:8]=="ASNG/W+ " and decoded_line[13:17]=="  kg" and decoded_line[8:13].replace(".", "").isnumeric() and float(decoded_line[8:13]) >= number:
                waiting = False

def tareBalance(balance):
    """
    Requests Balance through serial to tare/zero its current reported weight
    """

    balance.write("Z\r\n".encode("ascii"))




### Tray class definitions
## Cup
@dataclass
class Cup:
    cup_id: str
    capacity_kg: float
    material: str = None
    fill_kg: float = 0
    ## TODO: implement contents method to redefine, assume completely full unless float given

## Slot
@dataclass
class Slot:
    cartesian: list # Measured from center of tray to center of slots
    cup: Cup = None

    def add_cup(self, cup):
        if self.cup is not None:
            print(f"Slot is already occupied.")
        else:
            self.cup = cup

    def remove_cup(self):
        if self.cup is not None:
            self.cup = None
        else:
            print(f"No cup found in slot.")
    
    def take_cup(self):
        if self.cup is not None:
            taken_cup = self.cup
            self.cup = None
            return taken_cup
        else:
            print(f"No cup found in slot.")
            return None
        
    def set_cup_material(self, material):
        if self.cup is not None:
            self.cup.material = material
        else:
            print(f"No cup found in slot.")

## Tray
@dataclass
class Tray:
    tray_id: str
    cartesian: list # Measured from axis of rotation of top joint to center of tray
    slots: list

    def add_cup(self, cup, slot_number):
        self.slots[slot_number].add_cup(cup)

    def remove_cup(self, slot_number):
        self.slots[slot_number].remove_cup()

    def take_cup(self, slot_number):
        return self.slots[slot_number].take_cup()

    def get_slot_cartesian(self, slot_number):
        return list(np.array(self.slots[slot_number].cartesian) + np.array(self.cartesian))
    
    def fill_all_slots_ez(self, first_cup_id, cup_capacity_kg):
        [self.slots[i].add_cup(Cup(cup_id=(first_cup_id + i), capacity_kg=cup_capacity_kg)) for i in range(len(self.slots))]

    def slots_occupied(self):
        return [slot.cup is not None for slot in self.slots]

    def set_all_material(self, material):
        for i in range(len(self.slots)):
            if self.slots_occupied()[i]:
                self.slots[i].set_cup_material(material)

## Tray Manager
class TrayManager:
    def __init__(self):
        self.trays = []

    def add_tray(self, tray):
        self.trays.append(tray)

    def remove_tray(self, tray):
        self.trays.remove(tray)

    def add_cup_data(self, cup):
        self.cups_data[cup.cup_id] = cup
   
    ## TODO: Implement way of fettching a cup by ID, eg: for tray in trays > for slot in slots > compare IDs, get




### Script definition
def fill_tray(tray, distributor_parameters, arduino, balance, seq = None, purge_slot = None, purge_time = 0):
    """
    Function to fill the cups in the tray with the ammount of material previously defined. 
    
    :param tray: Object with the information about the tray and its cups
    :param distributor_parameters: Object with information about the arm
    :param arduino: serial connection to arduino
    :param balance: serial connection to balance
    :param seq: List with the order of filling of the cups. If None, they will all be filled in default order
    :param purge_slot: Number of the slot used for purging. This slot can be left without cup so the material is
                        discarded. Used when changing material. If None, no purging will be done.
    :param purge_time: Time for purge in minutes. Default: 0
    
    """
    # Assumes valve starts closed
    cartesian_initial = [sum([distributor_parameters[n].get('length_mm') for n in range(len(distributor_parameters))]), 0]
    cartesian_prev = cartesian_initial
    
    distributor_chain = ps.distributor_chain_update(distributor_parameters)

    # Bound is overridden for Inverse Kinematics to prioritize bottom joint over top joint rotation
    bound_rad_initial = distributor_parameters[0]['bound_rad']
    distributor_parameters[0]['bound_rad'] = np.deg2rad(0.1)
    distributor_chain = distributor_chain_update(distributor_parameters)
    
    if seq is None:
        sequence = range(len(tray.slots))
    else:
        sequence = seq
    
    for slot_number in sequence:
        if tray.slots_occupied()[slot_number]:  # TODO: Add check for cup not being full & matching materials
            tareBalance(balance)
            

            if slot_number == 8:
                distributor_parameters[0]['bound_rad'] = bound_rad_initial
                distributor_chain = distributor_chain_update(distributor_parameters)
            
            IK_plot(tray.get_slot_cartesian(slot_number), cartesian_prev, distributor_parameters, distributor_chain)
            goToCartesianFrom(tray.get_slot_cartesian(slot_number), cartesian_prev, distributor_parameters, distributor_chain, arduino)
            waitForSerialStr("pos finished", arduino)
            runValve(True, arduino)
            waitForSerialStr("val finished", arduino)
            # print('Waiting for Balance')
            if slot_number == purge_slot:
                print("PURGING.......................................................................")
                time.sleep(purge_time*60)
            else:
                print("FILLING CUP ", slot_number)
                waitUntilWeight(tray.slots[slot_number].cup.capacity_kg, balance)
            runValve(False, arduino)
            waitForSerialStr("val finished", arduino)
            cartesian_prev = tray.get_slot_cartesian(slot_number)
            
            if purge_slot is not None and seq is None:
                IK_plot(tray.get_slot_cartesian(purge_slot), cartesian_prev, distributor_parameters, distributor_chain)
                goToCartesianFrom(tray.get_slot_cartesian(purge_slot), cartesian_prev, distributor_parameters, distributor_chain, arduino)
                waitForSerialStr("pos finished", arduino)
                runValve(True, arduino)
                waitForSerialStr("val finished", arduino)
                # print('Waiting for Balance')
                #waitUntilWeight(tray.slots[slot_number].cup.capacity_kg, balance)
                print("PURGING.......................................................................")
                time.sleep(purge_time*60)
                runValve(False, arduino)
                waitForSerialStr("val finished", arduino)
                cartesian_prev = tray.get_slot_cartesian(purge_slot)
            
    IK_plot(cartesian_initial, cartesian_prev, distributor_parameters, distributor_chain)
    goToCartesianFrom(cartesian_initial, cartesian_prev, distributor_parameters, distributor_chain, arduino)
    waitForSerialStr("pos finished", arduino)


def fill_tray_SIMULATED(tray, distributor_parameters, seq = None, purge_slot = None, purge_time = 0):
    
    # Assumes valve starts closed
    cartesian_initial = [sum([distributor_parameters[n].get('length_mm') for n in range(len(distributor_parameters))]), 0]
    cartesian_prev = cartesian_initial
    
    distributor_chain = distributor_chain_update(distributor_parameters)

    # Bound is overridden for Inverse Kinematics to prioritize bottom joint over top joint rotation
    bound_rad_initial = distributor_parameters[0]['bound_rad']
    distributor_parameters[0]['bound_rad'] = np.deg2rad(0.1)
    distributor_chain = distributor_chain_update(distributor_parameters)
    
    if seq is None:
        sequence = range(len(tray.slots))
    else:
        sequence = seq
    
    for slot_number in sequence:
        if tray.slots_occupied()[slot_number]:  # TODO: Add check for cup not being full & matching materials
            

            if slot_number == 8:
                distributor_parameters[0]['bound_rad'] = bound_rad_initial
                distributor_chain = distributor_chain_update(distributor_parameters)
            
            if slot_number == purge_slot:
                print("PURGING.......................................................................")
                time.sleep(purge_time)
            else:
                print("FILLING CUP ", slot_number)
            
            IK_plot(tray.get_slot_cartesian(slot_number), cartesian_prev, distributor_parameters, distributor_chain)
            cartesian_prev = tray.get_slot_cartesian(slot_number)
            
            if purge_slot is not None and seq is None:
                print("PURGING.......................................................................")
                time.sleep(purge_time)
                
                IK_plot(tray.get_slot_cartesian(purge_slot), cartesian_prev, distributor_parameters, distributor_chain)
                cartesian_prev = tray.get_slot_cartesian(purge_slot)
            
    
    
