#!/usr/bin/env python

"""
.. module:: custom_classes
    :platform: Unix
    :synopsis: Python script that implements a custom class for the creation of ontologies and for the interaction with them.

.. moduleauthor:: Emanuele Rambaldi <emanuele.rambaldi3@studio.unibo.it>

This script implements a custom class for the creation of ontologies and for the interaction with them.

"""

# IMPORTS

import sys

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

#----------------------------------------------------------------------------------------------------------------

class EnvironmentOntology():

    """Class that allows for the creation of instances representing ontologies and for the interaction with the ARMOR server, which manages the corresponding ontologies.

    """

    # CLASS CONSTRUCTOR (function that is run one time every time that an instance of the class is instatiated)

    def __init__(self): 

        """ (constructor) Function that is called whenever an instance of this class is defined.

        The function simply defines and initializes the client that sends requests belonging to the service '/armor_interface_srv' of type 'ArmorDirective'.

        Args:
            self: variable that refers to the class instance

        """

        self.cli_armordirective = rospy.ServiceProxy('/armor_interface_srv',ArmorDirective) #initialize and define the client that sends requests belonging to the service '/armor_interface_srv' of type 'ArmorDirective'

    # CLASS METHODS

    def send_armor_request(self, command, primary_command_spec, secondary_command_spec, args):

        """Function that is called in order to send requests belonging to the '/armor_interface_srv' service.

        The function simply fills the 'ArmorDirectiveRequest' message fields with the arguments that have been passed to it and sends the request.

        Args:
            self: variable that refers to the class instance
            command (str): string that specifies the action that the server should perform
            primary_command_spec (str): string that encodes a specification regarding the command
            secondary_command_spec (str): string that encodes a specification regarding the command
            args (str list): list containing further arguments for the request

        Returns:
            res (ArmorDirectiveResponse): complex struct containing the server response to the request

        """

        armordirective_req = ArmorDirectiveRequest()

        # Fill the fields of the request
        armordirective_req.armor_request.client_name = 'example'
        armordirective_req.armor_request.reference_name = 'ontoRef'
        armordirective_req.armor_request.command = command
        armordirective_req.armor_request.primary_command_spec = primary_command_spec
        armordirective_req.armor_request.secondary_command_spec = secondary_command_spec
        armordirective_req.armor_request.args = args
        
        rospy.wait_for_service('/armor_interface_srv')
        try:
            res = self.cli_armordirective(armordirective_req)
        except rospy.ServiceException as e:
            print('Service call failed: %s' %e)
            sys.exit(1)

        return res

    #----------------------------------------------------------------------------

    def build_environment(self, ontology_path, ontology_name, tot_n_corridors, rooms_corridor, n_rooms_left, connection, doors_list, corridors_list, rooms_list):

        """Function that is called in order to construct the desired ontology.

        |  First, the plain ontology (ontology without the Abox) is loaded. Then, based on the arguments that have been passed to the function, some manipulations of the ontology are performed.
        |  In particular for each location the doors are specified and all the defined individuals are made disjointed between each other.
        |  In addition to that, the robot is positioned in the charging room and the timestamp of the last time a location has been visited is initialised for each location.
        |  Finally, the reasoner is instructed to reason about the changes that were made and the constructed ontology (ontology with the Abox) is saved with the name passed as argument.
        |  Note that all these actions are performed by interacting with the ontology via the ARMOR server.

        Args:
            self: variable that refers to the class instance
            ontology_path (str): string that specifies the path to the folder that contains the ontologies
            ontology_name (str): string that specifies the name the contructed ontology is saved with
            tot_n_corridors (int): total number of corridors in the desired environment
            rooms_corridor (int array): array containing the number of rooms connected to each corridor in the desired environment
            n_rooms_left (int): number of rooms that have not been assigned to a corridor in the desired environment and therefore are connected to the charging room 
            connection (Bool array): array containing information about the connection between adjacent corridors in the desired environment
            doors_list (str list): list containing the lables associated with all the doors in the environment
            rooms_list (str list): list containing the lables associated with all the rooms in the environment
            corridors_list (str list): list containing the lables associated with all the corridors in the environment

        """

        # LOAD THE ONTOLOGY
        self.send_armor_request('LOAD','FILE','',[ontology_path + 'topological_map.owl', 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])

        door_to_be_used = 0

        # QUERY ROOMS FOR EACH CORRIDOR (if any)
        if tot_n_corridors != 0:
            print("- Connecting rooms to corridors (if any):")
            for i in range(0,tot_n_corridors): # consider one of the corridors out of the list
                for j in range(door_to_be_used,door_to_be_used+rooms_corridor[i,0]): # add a connection to all the rooms
                    #---
                    # print("query (hasDoor;" + corridors_list[i] + ";" + doors_list[j] + ")") # DEBUG
                    self.send_armor_request('ADD','OBJECTPROP','IND',['hasDoor',corridors_list[i],doors_list[j]])
                    #---
                    # print("query (hasDoor;" + rooms_list[j] + ";" + doors_list[j] + ")") # DEBUG
                    self.send_armor_request('ADD','OBJECTPROP','IND',['hasDoor',rooms_list[j],doors_list[j]])
                    #---
                door_to_be_used = door_to_be_used + rooms_corridor[i,0]

        # QUERY ROOMS FOR THE CHARGING ROOM
        if n_rooms_left != 0:
            print("- Connecting rooms to the charging room:")
            for i in range(0,n_rooms_left): #consider the remaining rooms
                #---
                # print("query (hasDoor;" + rooms_list[door_to_be_used] + ";" + doors_list[door_to_be_used] + ")") # add the connection to the charging room # DEBUG
                self.send_armor_request('ADD','OBJECTPROP','IND',['hasDoor',rooms_list[door_to_be_used],doors_list[door_to_be_used]])
                #---
                # print("query (hasDoor;E0;" + doors_list[door_to_be_used] + ")") # DEBUG
                self.send_armor_request('ADD','OBJECTPROP','IND',['hasDoor','E0',doors_list[door_to_be_used]])
                #---
                door_to_be_used = door_to_be_used + 1
        
        # QUERY CORRIDORS (if any) FOR THE CHARGING ROOM
        if tot_n_corridors != 0:
            print("- Connecting corridors (if any) to the charging room:")
            for i in range(0,tot_n_corridors): #consider one of the corridors out of the list
                #---
                # print("query (hasDoor;" + corridors_list[i] + ";" + doors_list[door_to_be_used] + ")") # add the connection to the charging room # DEBUG
                self.send_armor_request('ADD','OBJECTPROP','IND',['hasDoor',corridors_list[i],doors_list[door_to_be_used]])
                #---
                # print("query (hasDoor;E0;" + doors_list[door_to_be_used] + ")") # DEBUG
                self.send_armor_request('ADD','OBJECTPROP','IND',['hasDoor','E0',doors_list[door_to_be_used]])
                #---
                door_to_be_used = door_to_be_used + 1

        # QUERY CORRIDORS (if more than 1) FOR OTHER CORRIDORS (if more than 1)
        if tot_n_corridors > 1:
            print("- Connnecting corridors (if more than 1) to other corridors (if more than 1)")
            for i in range(0,len(connection)):
                if connection[i,0]==True: # add the connections between corridors
                    #---
                    # print("query (hasDoor;" + corridors_list[i] + ";" + doors_list[door_to_be_used] + ")") # DEBUG
                    self.send_armor_request('ADD','OBJECTPROP','IND',['hasDoor',corridors_list[i],doors_list[door_to_be_used]])
                    #---
                    # print("query (hasDoor;" + corridors_list[i+1] + ";" + doors_list[door_to_be_used] + ")") # DEBUG
                    self.send_armor_request('ADD','OBJECTPROP','IND',['hasDoor',corridors_list[i+1],doors_list[door_to_be_used]])
                    #---
                    door_to_be_used = door_to_be_used + 1

        #---
        print("- Finished adding locations and doors... Making them disjointed...")
        # print("door to be used next {0}".format(door_to_be_used)) # DEBUG

        # print(['E0',*doors_list,*corridors_list,*rooms_list]) # DEBUG
        self.send_armor_request('DISJOINT','IND','',['E0',*doors_list,*corridors_list,*rooms_list])

        #---
        print("- Positioning the robot in the charging room...")

        self.send_armor_request('ADD','OBJECTPROP','IND',['isIn', 'Robot1', 'E0'])

        #---
        print("- Initialising the timestamp associated to the visitedAt property of all the locations...")

        initial_visited_stamp="{}".format(0)

        for location in ['E0',*corridors_list,*rooms_list]:
            self.send_armor_request('ADD','DATAPROP','IND',['visitedAt', location, 'Long', initial_visited_stamp])

        #---
        print("- Reasoning...")

        self.send_armor_request('REASON','','',[''])

        #---
        print("- Saving the ontology...")

        self.send_armor_request('SAVE','','',[ontology_path + ontology_name + '.owl'])

    #----------------------------------------------------------------------------

    def urgent_check(self):

        """Function that is called in order to determine the location that the robot should reach next.

        |  First, the ontology is queried so as to retrieve the locations that the robot can reach. Then, the labels associated to reachable locations are checked in order to detect urgent locations and corridors. 
        |  If no urgent location has been detected, then a corridor, randomly chosen among the reachable locations, is returned. Otherwise an urgent location, randomly chosen among the reachable locations, is returned.
        |  Note that all these actions are performed by interacting with the ontology via the ARMOR server.

        Args:
            self: variable that refers to the class instance

        Returns:
            inspected_location/inspected_corridor (str): string that specifies the location that the robot should reach next

        """

        print("")
        print("> Checking whether there are URGENT adjacent locations or not...")

        #---
        print("- Evaluating the locations that the robot can reach...")

        res1 = self.send_armor_request('QUERY','OBJECTPROP','IND',['canReach', 'Robot1'])

        adjacent_locations_list = res1.armor_response.queried_objects
        random.shuffle(adjacent_locations_list) # shuffle the list of adjacent locations so as to avoid to select always the same urgent location/corridor

        for i in range(0, len(adjacent_locations_list)):
            print ('   ' + adjacent_locations_list[i][32:34])

        inspected_location=""
        inspected_corridor=""

        #---
        # print("- Checking the urgency of the locations that the robot can reach...") # DEBUG
        for i in range(0, len(adjacent_locations_list)):
            inspected_location = adjacent_locations_list[i][32:34]
            # print ("Checking location " + inspected_location + " urgency...") # DEBUG

            res2 = self.send_armor_request('QUERY','CLASS','IND',[inspected_location, 'false'])

            for j in range(0, len(res2.armor_response.queried_objects)):
                inspected_class = res2.armor_response.queried_objects[j].split('#')[1][:-1] # consider the part of the string after the '#' symbol, without the last term
                # print(inspected_class) # DEBUG
                if(inspected_class == 'CORRIDOR'):
                    inspected_corridor = inspected_location # keep track of the last corridor you inspected
                if(inspected_class == 'URGENT'):
                    print("")
                    print('\033[91m' +"URGENT " + '\033[0m' + "location detected -> setting " + inspected_location + " as target")
                    return(inspected_location)

        print("")
        print('\033[91m' +"NO URGENT " + '\033[0m' + "location  detected -> setting " + inspected_corridor + " as target")
        return(inspected_corridor) # if I exited both for loops it means that I didn't find any urgent room

    #----------------------------------------------------------------------------

    def update_room_stamp(self):

        """Function that is called in order to update the timestamp that takes into account the last time a location was visited.

        |  First, the ontology is queried in order to retrieve both the location that the robot is in and the timestamp of the last time the robot visited such location. 
        |  Then, the just-retrieved timestamp is replaced with the current time. Finally, the reasoner is instructed to reason about the changes that were made.
        |  Note that all these actions are performed by interacting with the ontology via the ARMOR server.
        |  The function returns the current robot location that it retrieved from the ontology.

        Args:
            self: variable that refers to the class instance

        Returns:
            current_location (str): string that specifies the current robot location

        """
    
        #---
        # print("- Retrieving the current robot position...") # DEBUG

        res = self.send_armor_request('QUERY','OBJECTPROP','IND',['isIn', 'Robot1'])

        # print(res.armor_response.queried_objects[0][32:34]) #from the string extract the second to last and third to last term # DEBUG

        current_location = res.armor_response.queried_objects[0][32:34]

        #---
        # print("- Retrieving the timestamp of the last time the robot visited location " + current_location + "...") # DEBUG

        res = self.send_armor_request('QUERY','DATAPROP','IND',['visitedAt', current_location])

        old_visited_stamp = res.armor_response.queried_objects[0].split('"')[1]
        new_visited_stamp = "{}".format(int(time.time()))
        # print("Before: "+old_visited_stamp+"\nNow: "+new_visited_stamp) # DEBUG

        #---
        # print("- Updating the timestamp of the last time the robot visited location " + current_location + "...") # DEBUG

        res = self.send_armor_request('REPLACE','DATAPROP','IND',['visitedAt', current_location, 'Long', new_visited_stamp, old_visited_stamp])

        #---
        # print("- Reasoning...") # DEBUG

        self.send_armor_request('REASON','','',[''])

        return(current_location)

    #----------------------------------------------------------------------------

    def update_robot_stamp(self):

        """Function that is called in order to update the timestamp that takes into account the last time the robot moved.

        |  First, the ontology is queried in order to retrieve the timestamp of the last time the robot moved. Then, the just-retrieved timestamp is replaced with the current time. 
        |  Finally, the reasoner is instructed to reason about the changes that were made.
        |  Note that all these actions are performed by interacting with the ontology via the ARMOR server.

        Args:
            self: variable that refers to the class instance

        """

        print("")
        print("> Updating the Now timestamp of the robot to the instant the robot reached the location...")

        #---
        # print("- Retrieving the timestamp of the last time the robot moved...") # DEBUG

        res = self.send_armor_request('QUERY','DATAPROP','IND',['now', 'Robot1'])

        old_now_stamp = res.armor_response.queried_objects[0].split('"')[1]
        new_now_stamp = "{}".format(int(time.time()))
        # print("Before: "+old_now_stamp+"\nNow: "+new_now_stamp) # DEBUG

        #---
        # print("- Updating the timestamp of the last time the robot moved...") # DEBUG

        res = self.send_armor_request('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', new_now_stamp, old_now_stamp])

        #---
        # print("- Reasoning...") # DEBUG

        self.send_armor_request('REASON','','',[''])

    #----------------------------------------------------------------------------

    def update_robot_location(self, reached_location):

        """Function that is called in order to update where the robot is.

        |  First, the ontology is queried in order to retrieve the location that the robot is in. Then, the just-retrieved location is replaced with the location passed as argument. 
        |  Finally, the reasoner is instructed to reason about the changes that were made.
        |  Note that all these actions are performed by interacting with the ontology via the ARMOR server.

        Args:
            self: variable that refers to the class instance
            reached_location (str): string that specifies the location reached by the robot

        """

        print("")
        print("> Updating the robot location...")

        #---
        # print("- Retrieving the past robot location...") # DEBUG

        res = self.send_armor_request('QUERY','OBJECTPROP','IND',['isIn', 'Robot1'])

        # print(res.armor_response.queried_objects[0][32:34]) #from the string extract the second to last and third to last term # DEBUG

        starting_location = res.armor_response.queried_objects[0][32:34]

        #---
        # print("- Updating the robot location from " + starting_location + " to "+ reached_location) # DEBUG

        res = self.send_armor_request('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1',  reached_location, starting_location])

        #---
        # print("- Reasoning...") # DEBUG

        self.send_armor_request('REASON','','',[''])
