#!/usr/bin/env python

# HOW TO RUN

# - run the ARMOR server
#          $ rosrun armor execute it.emarolab.armor.ARMORMainService
# - run the 'roscore' and then you can launch the SW architecture
#          $ roslaunch exprob_first_assignment software_architecture.launch ontology_name:="pippo"

"""
.. module:: state_machine
    :platform: Unix
    :synopsis: Python module that implements a state_machine to manage the behaviour of the robot

.. moduleauthor:: Emanuele Rambaldi <emanuele.rambaldi3@studio.unibo.it>

This node interacts with the other nodes of the software architecture in order to determine the desired behaviour of the robot. 
Specifically, first of all it sets the initial position of the robot, by communicating it to the 'robot-states' node. The proper state-machine, composed of 6 states, is then started.
Once the node enters the firs state ('BuildEnvironment'), a GUI is printed on the screen, asking for the environment's characteristics. After that, the desired environment is built, by communicating with the ARMOR server.
In other words, a series of requests is issued to the ARMOR server, which takes care of the actual creation of the ontology. The control is then passed to the second state ('Reason'). 
This is in charge of querying the ontology about the rooms adjacent to the one the robot is in. Based on the decision taken in this state, the robot is instructed to go either in an adjacent urgent room or in an adjacent corridor. 
In order to accomplish this task, in a new state ('Navigate'), first a request is issued to the planner server ('planner' node) so as to retrieve a path towards the desired destinetion. 
Then, the evaluated path is forwarded to the controller server ('controller node') via another request, in order to guide the robot towards the location at issue. 
Once the desired location has been reached, the state changes again ('Wait') and the robot 'sleeps' for a little time, simulating the room exploration. 
Whatever the state the robot is in, if a 'battery_low' message is sent on the corresponding topic by the 'robot-states' node, the robot is instructed to drop what it is doing and navigate to the charging room to recharge its battery.
Specifically, first the state changes to 'NavigatetoCharge', whereby the same mechanism as the state 'Navigate' is carried out. Then, the node enters the state called 'Charge', that simulates the act of recharging the robot's battery.
After that, the control is passed again to the state 'Reason' and the cycle repeats.


Subscribes to:
    - /state/battery_low

Client:
    - /state/set_pose
    - /armor_interface_srv

Action client:
    - motion/planner
    - motion/controller

"""

# IMPORTS

import sys
from custom_classes import EnvironmentOntology # import the EnvironmentOntology class to define a new ontology and interact with it

import numpy as np
from http.client import USE_PROXY
from threading import Lock
import roslib
import rospy
import smach
import smach_ros
import time
import random
import math
import functools


from std_msgs.msg import String, Float64, Bool, Float32
from geometry_msgs.msg import Twist
from exprob_first_assignment.msg import Point
from exprob_first_assignment.srv import SetPose, SetPoseResponse
import exprob_first_assignment  # This is required to pass the `PlanAction` and `ControlAction` type for instantiating the `SimpleActionClient`.

from armor_msgs.srv import ArmorDirective, ArmorDirectiveRequest, ArmorDirectiveResponse
from armor_msgs.srv import ArmorDirectiveList, ArmorDirectiveListRequest, ArmorDirectiveListResponse

import actionlib
import actionlib.msg
from exprob_first_assignment.msg import PlanAction, PlanGoal
from exprob_first_assignment.msg import ControlAction, ControlGoal

#==================================================================================================================

# GLOBAL VARIABLES

# global action clients
actcli_plan = actionlib.SimpleActionClient('motion/planner', PlanAction) 
#initialize and define the global action client that sends requests belonging to the 'motion/planner' action
"""
Global action client the task of which is to ask the 'planner' server to generate a path towards a desired location
"""

actcli_control = actionlib.SimpleActionClient('motion/controller', ControlAction) #initialize and define the global action client that sends requests belonging to the 'motion/controller' action
"""
Global action client the task of which is to ask the 'controller' server to guide the robot towards a desired location along a given path
"""

# global variables and constants
transition = 'no_transition' #global variable containing the current transition
"""
Global string variable containing the current transition
"""

# mutexes
mutex=Lock() #initialize and define a mutex that will manage the access to the global variable 'transition'
"""
Global mutex to manage the access to the global variable 'transition'
"""

#==================================================================================================================

# SUBSCRIBERS CALLBACK FUNCTIONS

def clbk_battery(msg):

    """Function that is called every time that a new message is published on the '/state/battery_low' topic.

    The function is aimed at catching the messages published on the topic by the 'robot-states' node and changing the value of the global variable 'transition' so as to make the state-machine evolve into the 'NavigatetoCharge' state.

    Args:
        msg (Bool): in priciple always set to 'True' to warn the state-mchine about the fact that the battery is low

    """

    global transition

    if (msg.data == True):
        mutex.acquire()
        try:
            # print('\033[93m' + "\nclbk_battery got the mutex" + '\033[0m') # DEBUG
            transition = 'battery_low'
        finally:
            mutex.release()
    else:
        print('\033[91m' + "\nUnrecognized message coming from the 'robot-states' node" + '\033[0m')

#==================================================================================================================

# GUI FUNCTION 

def gui_function():

    """Function that prints on the screen a GUI to guide the user in building the desired environment.

    """

    while(1):
        while(1):
            tot_n_rooms = input("How many rooms does the flat have? [max 10] ")
            try:
                tot_n_rooms = int(tot_n_rooms)
                break
            except:
                print('\033[91m' + "The input is not an integer number" + '\033[0m' + " -> try again")

        if(tot_n_rooms < 1 or tot_n_rooms > 10):
            print('\033[91m' + "The input is out of bounds" + '\033[0m' + " -> try again")
        else:
            break
    n_rooms_left = tot_n_rooms
    while(1):
        while(1):
            tot_n_corridors = input("How many corridors does the flat have? [max 5] ")
            try:
                tot_n_corridors = int(tot_n_corridors)
                break
            except:
                print('\033[91m' + "The input is not an integer number" + '\033[0m' + " -> try again")

        if(tot_n_corridors < 0 or tot_n_corridors > 5): # if the total number of corridors is 0, there is no problem because the charging room is present anyway and it is a corridor -> that's why i put tot_n_corridors < 0 and not tot_n_corridors < 1
            print('\033[91m' + "The input is out of bounds" + '\033[0m' + " -> try again")
        elif(tot_n_corridors > tot_n_rooms):
            print('\033[91m' + "The input cannot be more than the number of rooms" + '\033[0m' + " -> try again")
        else:
            break
    
    if (tot_n_corridors != 0):
        rooms_corridor = np.empty((tot_n_corridors,1),dtype=int)
        for i in range(0, tot_n_corridors):
            while(1):
                while(1):
                    n_rooms = input("How many rooms does corridor {0} have? [min 1 ; max {1}] ".format(i+1, (n_rooms_left-(tot_n_corridors-(i+1)))) )
                    try:
                        n_rooms = int(n_rooms)
                        break
                    except:
                        print('\033[91m' + "The input is not an integer number" + '\033[0m' + " -> try again")

                if(n_rooms < 1 or n_rooms > (n_rooms_left-(tot_n_corridors-(i+1)))):
                    print('\033[91m' + "The input is out of bounds" + '\033[0m' + " -> try again")
                else:
                    break

            if(n_rooms != (n_rooms_left-(tot_n_corridors-(i+1)))):
                rooms_corridor[i,0]=n_rooms
                n_rooms_left=n_rooms_left-n_rooms
            else:
                rooms_corridor[i,0]=n_rooms
                n_rooms_left=n_rooms_left-n_rooms
                print('\033[94m' + "\nAll the remaining corridors have necessarily 1 room"  + '\033[0m')
                for j in range(i+1,tot_n_corridors):
                    rooms_corridor[j,0]=1
                    n_rooms_left=n_rooms_left-1
                break

        print("")
        for i in range(0,tot_n_corridors):
            print("The " + '\033[92m' + "corridor {0} ".format(i+1) + '\033[0m' + "has " + '\033[92m' + "{0} rooms ".format(rooms_corridor[i,0]) + '\033[0m' + "then")
    else:
        rooms_corridor = -1
        print("")

    print("The " + '\033[92m' + "charging room " + '\033[0m' + "has all the remaining rooms, therefore " + '\033[92m' + "{0} rooms".format(n_rooms_left) + '\033[0m')


    if (tot_n_corridors > 1):
        print("")
        print("Now answer 'yes' or 'no' to the following question:")
        connection = np.empty((tot_n_corridors-1,1),dtype=bool)
        for i in range(0,tot_n_corridors-1):
            while(1):
                answer = input("Is corridor {0} connected to corridor {1}? ".format(i+1, i+2))

                if (answer == 'yes' or answer == 'Yes' or answer == 'y' or answer == 'Y'):
                    connection[i,0] = True
                    break
                elif (answer == 'no' or answer == 'No' or answer == 'n' or answer == 'N'):
                    connection[i,0] = False
                    break
                else:
                    print('\033[91m' + "The input is neither 'yes' nor 'no'" + '\033[0m' + " -> try again")

        print("")
        for i in range(0,tot_n_corridors-1):
            if (connection[i,0] == True):
                print("The " + '\033[92m' + "corridor {0} ".format(i+1) + '\033[0m' + "is connected to the " + '\033[92m' + "corridor {0} ".format(i+2) + '\033[0m' + "then")
            else:
                print("The " + '\033[92m' + "corridor {0} ".format(i+1) + '\033[0m' + "is " + '\033[94m' + "NOT " + '\033[0m' + "connected to the " + '\033[92m' + "corridor {0} ".format(i+2) + '\033[0m' + "then")
    else:
        connection = -1
        print("")

    print("The " + '\033[92m' + "charging room " + '\033[0m' + "is connected to all the corridors by hypothesis")

    return tot_n_rooms, tot_n_corridors, rooms_corridor, n_rooms_left, connection

#==================================================================================================================

# REGULAR FUNCTIONS

def set_initial_pose():

    """Function that is called in order to send requests belonging to the '/state/set_pose' service.

    After the service is defined and initialised, the inital pose of the robot is retrieved from the parameter server. Then, this pose is sent in form of request to the server. 
    The user is notified both if the service call fails and if it succeds.

    """

    cli_setpose = rospy.ServiceProxy('/state/set_pose',SetPose) #initialize and define the client that sends requests belonging to the service '/state/set_pose' of type 'SetPose'

    print("")
    print('\033[92m' + "Communicating to the 'robot-states' node the initial position of the robot..." + '\033[0m')

    initial_pose = rospy.get_param('/state/initial_pose', [0.0,0.0]) # retrieve the initial pose of the robot from the parameter server.
    # This pose should be within the environmet_size -> [0.0,0.0] is the default initial pose.

    setpose_req = Point()

    setpose_req.x = initial_pose[0]
    setpose_req.y = initial_pose[1]

    print("- The robot initial coordinates are:\n  x: {0}\n  y: {1}".format(setpose_req.x, setpose_req.y))

    rospy.wait_for_service('/state/set_pose')
    try:
        cli_setpose(setpose_req) # initial_pose is retrieved from the state/initial_pose parameter and it is hereafter set in the robot-state node as the initial position of the robot
    except rospy.ServiceException as e:
        print('Service call failed: %s' %e)
        sys.exit(1)

    print("- The robot initial coordinates have been communicated correctly")
    print("\n")

#----------------------------------------------------------------------------

def generate_lists(tot_n_corridors, tot_n_rooms, connection):

    """Function that given some pieces of information about the desired environment, generates a list of labels for each kind of item.
    
    Args:
        tot_n_corridors (int): total number of corridors in the desired environment
        tot_n_rooms (int): total number of rooms in the desired environment
        connection (Bool array): array containing information about the connection between adjacent corridors in the desired environment

    Returns:
        doors_list (str list): list containing the lables associated with all the doors in the environment
        rooms_list (str list): list containing the lables associated with all the rooms in the environment
        corridors_list (str list): list containing the lables associated with all the corridors in the environment
    """

    print("")
    print('\033[92m' + "The desired environment is composed by the following instances:" + '\033[0m')

    # Create a list containing all the doors
    print("- Doors:")
    doors_list = [] # define an empty list
    if tot_n_corridors > 1:
        for i in range(0,tot_n_rooms+tot_n_corridors+np.count_nonzero(connection)): # np.count_nonzero() counts the number of True slots in the array
            doors_list.append("D"+"{0}".format(i+1))
            print(doors_list[i])
    else:
        for i in range(0,tot_n_rooms+tot_n_corridors):
            doors_list.append("D"+"{0}".format(i+1))
            print(doors_list[i])
    
    # Create a list containing all the rooms
    print("- Rooms:")
    rooms_list = [] # define an empty list
    for i in range(0,tot_n_rooms):
        rooms_list.append("R"+"{0}".format(i+1))
        print(rooms_list[i])

    # Create a list containing all the corridors
    print("- Corridors:")
    corridors_list = [] # define an empty list
    for i in range(0,tot_n_corridors):
        corridors_list.append("C"+"{0}".format(i+1))
        print(corridors_list[i])

    return(doors_list, rooms_list ,corridors_list)

#----------------------------------------------------------------------------

def cancel_control_goals():

    """Function that is called in order to send requests belonging to the 'motion/controller' action service.

    The function is simply aimed at cancelling all pending control goals.

    """

    print("")
    print('\033[92m' + "Cancelling previous control goals..." + '\033[0m')

    actcli_control.wait_for_server()
    actcli_control.cancel_all_goals() #cancel all previous control goals

#----------------------------------------------------------------------------

def plan(location):

    """Function that is called in order to send requests belonging to the 'motion/planner' action service.

    First, the goal location passed as argument is sent in form of request to the action server. Then, the process waits for the path towards the location to be generated.
    When the action server provides the path, it is stored in a variable and returned.

    Args:
        location (str): label of the location that the robot has to reach

    Returns:
        via_points_list (Point list): list containing random-generated via-points towards the goal location

    """

    print("")
    print('\033[92m' + "Evaluating a path towards the location " + location + "..." + '\033[0m')

    # set the desired planning goal
    goal_planner = PlanGoal()
    goal_planner.location = location

    actcli_plan.wait_for_server()
    # Send a new `goal_planner`, which is a message of type `PlanGoal`
    actcli_plan.send_goal(goal_planner)

    # Waits for the server to finish performing the action
    actcli_plan.wait_for_result()

    # Gets the result of executing the action
    via_points_list = ControlGoal()
    via_points_list = actcli_plan.get_result()

    return via_points_list

#----------------------------------------------------------------------------

def control(via_points_list):

    """Function that is called in order to send requests belonging to the 'motion/controller' action service.

    The function sends the list of via-points passed as argument in form of request to the action server.

    Args:
        via_points_list (Point list): list containing random-generated via-points towards the goal location

    """

    print("")
    print('\033[92m' + "Controlling the robot towards the desired location..." + '\033[0m')

    actcli_control.wait_for_server()
    # Send the via points list retrieved by the planner to the controller
    actcli_control.send_goal(via_points_list)


#==================================================================================================================   

# STATES 

# define state Reason
class BuildEnvironment(smach.State,EnvironmentOntology):

    """Class that defines a state whereby the environment is created as the user desires.

    .. note::

        The only possible outcome of the implemented state is the string 'environment_built'.
        The variable 'sm_targetloc' shared among the states here is referred as: 'buildenvironment_targetloc_in' (as far as the input value is concerned) and 'buildenvironment_targetloc_out' (as far as the output value is concerned).
        It contains the location the robot should reach.
        The variable 'sm_prevstate' shared among the states here is referred as: 'buildenvironment_prevstate_in' (as far as the input value is concerned) and 'buildenvironment_prevstate_out' (as far as the output value is concerned).
        It contains the previous state identifier.
    """

    def __init__(self):

        """ (constructor) Function that is called whenever an instance of this class is defined.

        The function mainly executes the constructor of the father class 'EnvironmentOntology' so as to inherit the attributes defined in there.

        Args:
            self: variable that refers to the class instance
            outcomes (str list): list containing all the possible strings that make the process exit this state
            input_keys (str list): list containing the names of the input variables used in this state
            output_keys (str list): list containing the names of the output variables used in this state

        """

        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['environment_built'],
                             input_keys=['buildenvironment_targetloc_in','buildenvironment_prevstate_in'],
                             output_keys=['buildenvironment_targetloc_out','buildenvironment_prevstate_out'])
        EnvironmentOntology.__init__(self) # Execute the constructor of the father class 'EnvironmentOntology' so as to inherit the attributes defined in there (self.cli_armordirective)
        
    def execute(self, userdata):

        """Function that is called every time that this state is executed.

        First, the GUI function is called in order to let the user decide how the environment should be and the retrieved information are stored in lists. Then, the desired environment is built, thanks to a series of requests issued to the ARMOR server, which takes care of the actual creation of the ontology.
        This task is carried out by means of the 'build_environment' method belonging to the imported class named 'EnvironmentOntology'. Finally the global variable 'transition' is assigned with the string 'environment_built' and it is returned.

        Args:
            self: variable that refers to the class instance
            userdata (struct): structure containing the input and output variables of the state

        """

        # function called when executing the node, it can be blocking

        global transition
        global mutex

        print('----------------------------------------------------------------')
        rospy.loginfo('Executing state BUILDENVIRONMENT (the previous state was: %d)'%userdata.buildenvironment_prevstate_in)
        print('----------------------------------------------------------------')


        print("")
        print('\033[92m' + "Preprocessing..." + '\033[0m')

        # retrieve the desired ontology name passed as argument from command line
        args = rospy.myargv(argv = sys.argv)
        if len(args) != 2: #check on the number of arguments -> one argument(the path to the file) is provided by the system by default -> so it is 1+2 in this case
            print('\033[91m' + "\ntoo many or not enough arguments provided" + '\033[0m' + " -> exiting")
            sys.exit(1)

        ontology_name = args[1]

        # execute the GUI function
        tot_n_rooms, tot_n_corridors, rooms_corridor, n_rooms_left, connection = gui_function()
        
        # create the lists of each class' individuals starting from the information retrieved from the user
        doors_list, rooms_list, corridors_list = generate_lists(tot_n_corridors, tot_n_rooms, connection)

        # retrieve and communicate to the 'robot-states' node the robot's initial position
        set_initial_pose()

        # --------------------

        print("")
        print('\033[92m' + "Building the desired environment..." + '\033[0m')

        self.build_environment(ontology_name, tot_n_corridors, rooms_corridor, n_rooms_left, connection, doors_list, corridors_list, rooms_list)

        mutex.acquire()
        try:
            # print('\033[93m' + "\nstate BuildEnvironment got the mutex (1)" + '\033[0m') # DEBUG
            transition = 'environment_built' # I reset the transition so that from this time on I can catch new transitions
        finally:
            mutex.release()

        userdata.buildenvironment_prevstate_out = 0
        print("\n")

        return transition

# define state Reason
class Reason(smach.State,EnvironmentOntology):

    """Class that defines a state whereby the created ontology is interrogated in order to retrieve information about the locations adjacent to the room that the robot is in.

    .. note::

        The possible outcomes of the implemented state are the strings 'battery_low' and 'done_reasoning'.
        The variable 'sm_targetloc' shared among the states here is referred as: 'reason_targetloc_in' (as far as the input value is concerned) and 'reason_targetloc_out' (as far as the output value is concerned).
        It contains the location the robot should reach.
        The variable 'sm_prevstate' shared among the states here is referred as: 'reason_prevstate_in' (as far as the input value is concerned) and 'reason_prevstate_out' (as far as the output value is concerned).
        It contains the previous state identifier.
    """

    def __init__(self):

        """ (constructor) Function that is called whenever an instance of this class is defined.

        The function mainly executes the constructor of the father class 'EnvironmentOntology' so as to inherit the attributes defined in there.

        Args:
            self: variable that refers to the class instance
            outcomes (str list): list containing all the possible strings that make the process exit this state
            input_keys (str list): list containing the names of the input variables used in this state
            output_keys (str list): list containing the names of the output variables used in this state

        """

        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['battery_low','done_reasoning'],
                             input_keys=['reason_targetloc_in','reason_prevstate_in'],
                             output_keys=['reason_targetloc_out','reason_prevstate_out'])
        EnvironmentOntology.__init__(self) 
    
    def execute(self, userdata):

        """Function that is called every time that this state is executed.

        First, the 'urgent_check' method belonging to the imported class named 'EnvironmentOntology' is executed to find out if there is an urgent room among the adjacent ones.
        Based on this check, the method at issue returns the location that the robot should reach, which is then stored in the output variable 'reason_targetloc_out'. 
        Finally, if the value of the global variable 'transition' has not changed into 'battery low', it is assigned with the string 'done_reasoning' and it is returned.

        Args:
            self: variable that refers to the class instance
            userdata (struct): structure containing the input and output variables of the state

        """

        # function called when executing the node, it can be blocking

        global transition
        global mutex

        print('----------------------------------------------------------------')
        rospy.loginfo('Executing state REASON (the previous state was: %d)'%userdata.reason_prevstate_in)
        print('----------------------------------------------------------------')

        mutex.acquire()
        try:
            # print('\033[93m' + "\nstate Reason got the mutex (1)" + '\033[0m') # DEBUG
            transition = 'no_transition' # I reset the transition so that from this time on I can catch new transitions
        finally:
            mutex.release()

        # --------------------

        print("")
        print('\033[92m' + "Reasoning..." + '\033[0m')

        target_location = self.urgent_check() # this function is atomic: can't be interrupted by a battery_low signal


        mutex.acquire()
        if(transition == 'no_transition'):
            # print('\033[93m' + "\nstate Reason got the mutex (2)" + '\033[0m') # DEBUG
            transition = 'done_reasoning'
            userdata.reason_targetloc_out = target_location # yield as output shared variable the target location (URGENT or CORRIDOR)
        mutex.release()
        
        userdata.reason_prevstate_out = 1
        print("\n")

        return transition

# define state Charge
class Charge(smach.State,EnvironmentOntology):

    """Class that defines a state that simulates the robot's battery recharging.

    .. note::

        The possible outcomes of the implemented state are the strings 'battery_low' and 'battery_full'.
        The variable 'sm_targetloc' shared among the states here is referred as: 'charge_targetloc_in' (as far as the input value is concerned) and 'charge_targetloc_out' (as far as the output value is concerned).
        It contains the location the robot should reach.
        The variable 'sm_prevstate' shared among the states here is referred as: 'charge_prevstate_in' (as far as the input value is concerned) and 'charge_prevstate_out' (as far as the output value is concerned).
        It contains the previous state identifier.
    """

    def __init__(self):

        """ (constructor) Function that is called whenever an instance of this class is defined.

        The function initialises a loop counter and executes the constructor of the father class 'EnvironmentOntology' so as to inherit the attributes defined in there.

        Args:
            self: variable that refers to the class instance
            outcomes (str list): list containing all the possible strings that make the process exit this state
            input_keys (str list): list containing the names of the input variables used in this state
            output_keys (str list): list containing the names of the output variables used in this state

        """

        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['battery_low','battery_full'],
                             input_keys=['charge_targetloc_in','charge_prevstate_in'],
                             output_keys=['charge_targetloc_out','charge_prevstate_out'])
        self.loop_count = 0 # variable containing the number of times the robot waited 0.5 sec
        self.charge_time = rospy.get_param('/state_machine/charge_time', 5.0) # retrieve the charging time from the parameter serve -> 5.0 s is the default value.
        EnvironmentOntology.__init__(self)
        
    def execute(self, userdata):

        """Function that is called every time that this state is executed.

        The function simply makes the process sleep for the desired charging time (5 s by default). However, during this period of time, the global variable 'tansition' is checked 10 times in order to detect possible 'battery_low' signals that have been issued.
        After the total amount of time, if the value of the global variable 'transition' has not changed into 'battery low', it is assigned with the string 'battery_full'.
        At the end of the fuction, the 'update_room_stamp' method belonging to the imported class named 'EnvironmentOntology' is executed so as to update the timestamp that takes into account the last time a location was visited.
        Finally the global variable 'transition' is returned.

        Args:
            self: variable that refers to the class instance
            userdata (struct): structure containing the input and output variables of the state

        """

        # function called when exiting from the node, it can be blocking

        global transition
        global mutex

        print('----------------------------------------------------------------')
        rospy.loginfo('Executing state CHARGE (the previous state was: %d)'%userdata.charge_prevstate_in)
        print('----------------------------------------------------------------')

        mutex.acquire()
        try:
            # print('\033[93m' + "\nstate Charge got the mutex (1)" + '\033[0m') # DEBUG
            transition = 'no_transition' #otherwise if I don't enter the if the transition won't be reset
        finally:
            mutex.release()

        # --------------------

        if(userdata.charge_prevstate_in != 2): # if the previous state was not this one ('Charge')
            self.loop_count = 0 # restart the loop counter
        
        # Wait some time (5 seconds by default) if the transition global variable remains 'no_transition'
        # This procedure of waiting is implemented to be NOT atomic, since hypothetically it can happen that a battery_low signal randomly arrives
        print("")
        print('\033[92m' + "Waiting {0} seconds for letting the battery recharge...".format(self.charge_time) + '\033[0m')
        while(self.loop_count < 10):
            mutex.acquire()
            # print('\033[93m' + "\nstate Charge got the mutex (2)" + '\033[0m') # DEBUG
            if(transition == 'no_transition'): # wait charge_time/10 sec and check the transition global variable again
                mutex.release()
                time.sleep(self.charge_time/10)
                self.loop_count = self.loop_count+1
            else:
                mutex.release()
                break

        mutex.acquire()
        if(transition == 'no_transition'):
            # print('\033[93m' + "\nstate Charge got the mutex (3)" + '\033[0m') # DEBUG
            transition = 'battery_full'
        mutex.release()

        # --------------------

        # Update the visitedAt property of the charging room
        print("")
        print('\033[92m' + "Updating the visitedAt timestamp of the charging room to the instant the robot exits the state 'Charge'..." + '\033[0m')
        self.update_room_stamp()

        # --------------------

        userdata.charge_targetloc_out = "" # yield as output shared variable an empty string, since no target location has to be passed to the next state
        userdata.charge_prevstate_out = 2
        print("\n")

        return transition


# define state Navigate
class Navigate(smach.State,EnvironmentOntology):

    """Class that defines a state whereby a path towards the goal location is generated and the robot is guided along such path.

    .. note::

        The possible outcomes of the implemented state are the strings 'battery_low' and 'target_reached'.
        The variable 'sm_targetloc' shared among the states here is referred as: 'navigate_targetloc_in' (as far as the input value is concerned) and 'navigate_targetloc_out' (as far as the output value is concerned).
        It contains the location the robot should reach.
        The variable 'sm_prevstate' shared among the states here is referred as: 'navigate_prevstate_in' (as far as the input value is concerned) and 'navigate_prevstate_out' (as far as the output value is concerned).
        It contains the previous state identifier.
    """

    def __init__(self):

        """ (constructor) Function that is called whenever an instance of this class is defined.

        The function mainly executes the constructor of the father class 'EnvironmentOntology' so as to inherit the attributes defined in there.

        Args:
            self: variable that refers to the class instance
            outcomes (str list): list containing all the possible strings that make the process exit this state
            input_keys (str list): list containing the names of the input variables used in this state
            output_keys (str list): list containing the names of the output variables used in this state

        """

        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['battery_low','target_reached'],
                             input_keys=['navigate_targetloc_in','navigate_prevstate_in'],
                             output_keys=['navigate_targetloc_out','navigate_prevstate_out'])
        EnvironmentOntology.__init__(self)
        
    def execute(self, userdata):

        """Function that is called every time that this state is executed.

        First, a request to the 'planner' server, containing the desired location that has been determined in the 'Reason' state, is sent. The server generates and returns a series of via-point towards the location at issue. 
        This list is passed on in form of a request to the 'controller' server which guides the robot along such path. During the accomplishment of this task the global variable 'transition' is continuously checked.
        If, by the time the 'controller' server finished performing the task, the value of the global variable 'transition' has not changed into 'battery low', it is assigned with the string 'target_reached'.
        At the beginning of the fuction, the 'update_room_stamp' method belonging to the imported class named 'EnvironmentOntology' is executed so as to update the timestamp that takes into account the last time a location was visited.
        At the end of the function instead, once the 'controller' server returned succesfully, the 'update_robot_location' and 'update_robot_stamp' methods are executed so as to update both the robot timestamp related to the last time it moved and its location.
        Finally the global variable 'transition' is returned.

        Args:
            self: variable that refers to the class instance
            userdata (struct): structure containing the input and output variables of the state

        """

        # function called when exiting from the node, it can be blocking
        
        global transition
        global mutex

        print('----------------------------------------------------------------')
        rospy.loginfo('Executing state NAVIGATE (the previous state was: %d)'%userdata.navigate_prevstate_in)
        print('----------------------------------------------------------------')

        mutex.acquire()
        try:
            # print('\033[93m' + "\nstate Navigate got the mutex (1)" + '\033[0m') # DEBUG
            transition = 'no_transition' # I reset the transition so that from this time on I can catch new transitions
        finally:
            mutex.release()

        # --------------------

        # Update the visitedAt property of the location that the robot is leaving
        print("")
        print('\033[92m' + "Updating the visitedAt timestamp of the location to the instant the robot starts leaving it..." + '\033[0m')
        self.update_room_stamp()

        # --------------------

        # Make a request to the planning server
        via_points_list = plan(userdata.navigate_targetloc_in) # this function is atomic: can't be interrupted by a battery_low signal

        print("The path to location " + userdata.navigate_targetloc_in + " has been generated...")
        print(via_points_list)

        # --------------------

        # Make a request to the controlling server
        control(via_points_list)

        # Wait for the end of the controlling task unless a new transition arrives
        mutex.acquire()
        while(transition == 'no_transition'):
            mutex.release()
            if (actcli_control.get_state() == 3): # aka the random point has been reached

                print("Location " + userdata.navigate_targetloc_in + " has been reached")

                # Update the location and now property of the robot (timestamp of the last time that the robot moved)
                self.update_robot_location(userdata.navigate_targetloc_in)
                self.update_robot_stamp()

                # -------------------

                mutex.acquire()
                try:
                    # print('\033[93m' + "\nstate Navigate got the mutex (2)" + '\033[0m') # DEBUG
                    transition = 'target_reached'
                finally:
                    mutex.release()

            mutex.acquire()
        mutex.release()

        userdata.navigate_targetloc_out = "" # yield as output shared variable an empty string, since no target location has to be passed to the next state
        userdata.navigate_prevstate_out = 3
        print("\n")

        return transition


# define state NavigatetoCharge
class NavigatetoCharge(smach.State,EnvironmentOntology):

    """Class that defines a state whereby a path towards the charging room is generated and the robot is guided along such path.

    .. note::

        The possible outcomes of the implemented state are the strings 'battery_low' and 'target_reached'.
        The variable 'sm_targetloc' shared among the states here is referred as: 'navigatetocharge_targetloc_in' (as far as the input value is concerned) and 'navigatetocharge_targetloc_out' (as far as the output value is concerned).
        It contains the location the robot should reach.
        The variable 'sm_prevstate' shared among the states here is referred as: 'navigatetocharge_prevstate_in' (as far as the input value is concerned) and 'navigatetocharge_prevstate_out' (as far as the output value is concerned).
        It contains the previous state identifier.
    """

    def __init__(self):

        """ (constructor) Function that is called whenever an instance of this class is defined.

        The function mainly executes the constructor of the father class 'EnvironmentOntology' so as to inherit the attributes defined in there.

        Args:
            self: variable that refers to the class instance
            outcomes (str list): list containing all the possible strings that make the process exit this state
            input_keys (str list): list containing the names of the input variables used in this state
            output_keys (str list): list containing the names of the output variables used in this state

        """

        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['battery_low','target_reached'],
                             input_keys=['navigatetocharge_targetloc_in','navigatetocharge_prevstate_in'],
                             output_keys=['navigatetocharge_targetloc_out','navigatetocharge_prevstate_out'])
        EnvironmentOntology.__init__(self)
        
    def execute(self, userdata):

        """Function that is called every time that this state is executed.

        If the previous state was not 'NavigatetoCharge', possible control goals are cancelled and a request to the 'planner' server, containing the charging room, is sent. The server generates and returns a series of via-point towards the location at issue. 
        This list is passed on in form of a request to the 'controller' server which guides the robot along such path. During the accomplishment of this task the global variable 'transition' is continuously checked.
        If, by the time the 'controller' server finished performing the task, the value of the global variable 'transition' has not changed into 'battery low', it is assigned with the string 'target_reached'.
        At the beginning of the fuction, the 'update_room_stamp' method belonging to the imported class named 'EnvironmentOntology' is executed so as to update the timestamp that takes into account the last time a location was visited.
        At the end of the function instead, once the 'controller' server returned succesfully, the 'update_robot_location' and 'update_robot_stamp' methods are executed so as to update both the robot timestamp related to the last time it moved and its location.
        Finally the global variable 'transition' is returned.

        Args:
            self: variable that refers to the class instance
            userdata (struct): structure containing the input and output variables of the state

        """

        # function called when exiting from the node, it can be blocking
        
        global transition
        global mutex

        print('----------------------------------------------------------------')
        rospy.loginfo('Executing state NAVIGATETOCHARGE (the previous state was: %d)'%userdata.navigatetocharge_prevstate_in)
        print('----------------------------------------------------------------')

        mutex.acquire()
        try:
            # print('\033[93m' + "\nstate NavigatetoCharge got the mutex (1)" + '\033[0m') # DEBUG
            transition = 'no_transition' # I reset the transition so that from this time on I can catch new transitions
        finally:
            mutex.release()

        # --------------------

        if (userdata.navigatetocharge_prevstate_in != 4): # if the previous state was not this one ('NavigatetoCharge')
        
            # Cancel previous control goals
            cancel_control_goals()

            # --------------------

            # Update the visitedAt property of the location that the robot is leaving
            print("")
            print('\033[92m' + "Updating the visitedAt timestamp of the location to the instant the robot starts leaving it..." + '\033[0m')
            self.update_room_stamp()

            # --------------------

            # Make a request to the planning server
            via_points_list = plan('E0') # this function is atomic: can't be interrupted by a battery_low signal

            print("The path to location E0 has been generated...")
            print(via_points_list)

            # --------------------

            # Make a request to the controlling server
            control(via_points_list)

        # Wait for the end of the controlling task unless a new transition arrives
        mutex.acquire()
        while(transition == 'no_transition'):
            mutex.release()
            if (actcli_control.get_state() == 3): # aka the random point has been reached

                print("Location E0 has been reached")

                # Update the location and now property of the robot (timestamp of the last time that the robot moved)
                self.update_robot_location('E0')
                self.update_robot_stamp()

                # -------------------

                mutex.acquire()
                try:
                    # print('\033[93m' + "\nstate NavigatetoCharge got the mutex (2)" + '\033[0m') # DEBUG
                    transition = 'target_reached'
                finally:
                    mutex.release()
                
            mutex.acquire()
        mutex.release()

        userdata.navigatetocharge_targetloc_out = "" # yield as output shared variable an empty string, since no target location has to be passed to the next state
        userdata.navigatetocharge_prevstate_out = 4
        print("\n")

        return transition


# define state Wait
class Wait(smach.State,EnvironmentOntology):

    """Class that defines a state that simulates the robot exploring the reached location.

    .. note::

        The possible outcomes of the implemented state are the strings 'battery_low' and 'waited_enough'.
        The variable 'sm_targetloc' shared among the states here is referred as: 'wait_targetloc_in' (as far as the input value is concerned) and 'wait_targetloc_out' (as far as the output value is concerned).
        It contains the location the robot should reach.
        The variable 'sm_prevstate' shared among the states here is referred as: 'wait_prevstate_in' (as far as the input value is concerned) and 'wait_prevstate_out' (as far as the output value is concerned).
        It contains the previous state identifier.
    """

    def __init__(self):

        """ (constructor) Function that is called whenever an instance of this class is defined.

        The function initialises a loop counter and executes the constructor of the father class 'EnvironmentOntology' so as to inherit the attributes defined in there.

        Args:
            self: variable that refers to the class instance
            outcomes (str list): list containing all the possible strings that make the process exit this state
            input_keys (str list): list containing the names of the input variables used in this state
            output_keys (str list): list containing the names of the output variables used in this state

        """

        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['battery_low','waited_enough'],
                             input_keys=['wait_targetloc_in','wait_prevstate_in'],
                             output_keys=['wait_targetloc_out','wait_prevstate_out'])
        self.loop_count = 0 # variable containing the number of times the robot waited 0.5 sec
        self.explore_time = rospy.get_param('/state_machine/explore_time', 5.0) # retrieve the exploring time from the parameter server -> 5.0 s is the default value.
        EnvironmentOntology.__init__(self)
        
    def execute(self, userdata):

        """Function that is called every time that this state is executed.

        The function simply makes the process sleep for the desired 'exploring' time (5 s by default). However, during this period of time, the global variable 'tansition' is checked 10 times in order to detect possible 'battery_low' signals that have been issued.
        After the total amount of time, if the value of the global variable 'transition' has not changed into 'battery low', it is assigned with the string 'waited_enough'.
        At the end of the fuction, the 'update_room_stamp' method belonging to the imported class named 'EnvironmentOntology' is executed so as to update the timestamp that takes into account the last time a location was visited.
        Finally the global variable 'transition' is returned.

        Args:
            self: variable that refers to the class instance
            userdata (struct): structure containing the input and output variables of the state

        """

        # function called when exiting from the node, it can be blocking
        
        global transition
        global mutex
        

        print('----------------------------------------------------------------')
        rospy.loginfo('Executing state WAIT (the previous state was: %d)'%userdata.wait_prevstate_in)
        print('----------------------------------------------------------------')

        mutex.acquire()
        try:
            # print('\033[93m' + "\nstate Wait got the mutex (1)" + '\033[0m') # DEBUG
            transition = 'no_transition' # I reset the transition so that from this time on I can catch new transitions
        finally:
            mutex.release()

        # --------------------
        
        # Wait some time (5 seconds by default) if the transition global variable remains 'no_transition'
        # This procedure of waiting is implemented to be NOT atomic, since hypothetically the robot doesn't just wait, but explores the room, so it can
        # happen that a battery_low signal arrives
        print("")
        print('\033[92m' + "Waiting {0} seconds for exploring the reached location...".format(self.explore_time) + '\033[0m')
        while(self.loop_count < 10):
            mutex.acquire()
            # print('\033[93m' + "\nstate Wait got the mutex (2)" + '\033[0m') # DEBUG
            if(transition == 'no_transition'): # wait explore_time/10 sec and check the transition global variable again
                mutex.release()
                time.sleep(self.explore_time/10)
                self.loop_count = self.loop_count+1
            else:
                mutex.release()
                break

        mutex.acquire()
        if(transition == 'no_transition'):
            # print('\033[93m' + "\nstate Wait got the mutex (3)" + '\033[0m') # DEBUG
            transition = 'waited_enough'
        mutex.release()

        # --------------------

        # Update the visitedAt property of the location that the robot reached
        print("")
        print('\033[92m' + "Updating the visitedAt timestamp of the location to the instant the robot exits the state 'Wait'..." + '\033[0m')
        self.update_room_stamp()

        # --------------------

        userdata.wait_targetloc_out = "" # yield as output shared variable an empty string, since no target location has to be passed to the next state       
        userdata.wait_prevstate_out = 5
        print("\n")

        return transition

#==================================================================================================================

# MAIN

def main():

    """Function that first initializes and defines the subscriber to the '/state/battery_low' topic, then it starts the 'smach' state-machine and finally spins to allow the cyclical execution of these mechanisms.
    """

    rospy.init_node('state_machine')

    # DEFINE SUBSCRIBERS ---------------------------------------------------------------------------
    sub_batterylow = rospy.Subscriber('/state/battery_low', Bool, clbk_battery) #define and initialize the subscriber to the topic '/state/battery_low'

    # CREATE A SMACH STATE MACHINE -----------------------------------------------------------------------------------------
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_targetloc = "" # variable shared among all states -> it contains the target location to reach. At the beginning it is "".
    sm.userdata.sm_prevstate = -1 # variable shared among all states -> it contains the previous state. At the beginning it is 0.

    # Open the container
    with sm:
        # Add states to the container
        # 'done_reasoning', 'target_reached', 'waited_enough' and 'battery_full' are not transitions that can randomly happen like 'battery_low', so they don't appear in every state
        smach.StateMachine.add('BUILDENVIRONMENT', BuildEnvironment(), 
                               transitions={'environment_built':'REASON'},
                               remapping={'buildenvironment_targetloc_in':'sm_targetloc',
                                          'buildenvironment_targetloc_out':'sm_targetloc',
                                          'buildenvironment_prevstate_in':'sm_prevstate',
                                          'buildenvironment_prevstate_out':'sm_prevstate'})
        smach.StateMachine.add('REASON', Reason(), 
                               transitions={'battery_low':'NAVIGATETOCHARGE',
                                            'done_reasoning':'NAVIGATE'},
                               remapping={'reason_targetloc_in':'sm_targetloc', 
                                          'reason_targetloc_out':'sm_targetloc',
                                          'reason_prevstate_in':'sm_prevstate',
                                          'reason_prevstate_out':'sm_prevstate'})
        smach.StateMachine.add('CHARGE', Charge(), 
                               transitions={'battery_low':'CHARGE',
                                            'battery_full':'REASON'},
                               remapping={'charge_targetloc_in':'sm_targetloc',
                                          'charge_targetloc_out':'sm_targetloc',
                                          'charge_prevstate_in':'sm_prevstate',
                                          'charge_prevstate_out':'sm_prevstate'})
        smach.StateMachine.add('NAVIGATE', Navigate(), 
                               transitions={'battery_low':'NAVIGATETOCHARGE',
                                            'target_reached':'WAIT'},
                               remapping={'navigate_targetloc_in':'sm_targetloc',
                                          'navigate_targetloc_out':'sm_targetloc',
                                          'navigate_prevstate_in':'sm_prevstate',
                                          'navigate_prevstate_out':'sm_prevstate'})
        smach.StateMachine.add('NAVIGATETOCHARGE', NavigatetoCharge(), 
                               transitions={'battery_low':'NAVIGATETOCHARGE',
                                            'target_reached':'CHARGE'},
                               remapping={'navigatetocharge_targetloc_in':'sm_targetloc',
                                          'navigatetocharge_targetloc_out':'sm_targetloc',
                                          'navigatetocharge_prevstate_in':'sm_prevstate',
                                          'navigatetocharge_prevstate_out':'sm_prevstate'})
        smach.StateMachine.add('WAIT', Wait(), 
                               transitions={'battery_low':'NAVIGATETOCHARGE',
                                            'waited_enough':'REASON'},
                               remapping={'wait_targetloc_in':'sm_targetloc',
                                          'wait_targetloc_out':'sm_targetloc',
                                          'wait_prevstate_in':'sm_prevstate',
                                          'wait_prevstate_out':'sm_prevstate'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
