#!/usr/bin/env python

"""
.. module:: robot_states
    :platform: Unix
    :synopsis: Python module that keeps track of the robot position and issues 'battery_low' messages either randomly or under the user request

.. moduleauthor:: Emanuele Rambaldi <emanuele.rambaldi3@studio.unibo.it>

This node defines two services to get and set the current robot pose, and a publisher to notify that the battery is low.

Publishes to:
    - /state/battery_low

Service:
    - /state/get_pose
    - /state/set_pose

"""

import threading
import random
import rospy
# Import constant name defined to structure the architecture.
import sys
sys.path.append('/home/emanuelerambaldi/ROS/my_ros_ws/src/exprob_first_assignment/scripts')
import architecture_name_mapper as anm
# Import the messages used by services and publishers.
from std_msgs.msg import Bool
# from arch_skeleton.srv import GetPose, GetPoseResponse, SetPose, SetPoseResponse
from exprob_first_assignment.srv import GetPose, GetPoseResponse, SetPose, SetPoseResponse


# A tag for identifying logs producer.
LOG_TAG = anm.NODE_ROBOT_STATE


# The node manager class.
# This class defines two services to get and set the current 
# robot pose, and a publisher to notify that the battery is low.
class RobotState:

    """Class that is composed of several methods aimed at keeping track of the robot state (position and battery level).

    """

    def __init__(self):

        """ (constructor) Function that is called whenever an instance of this class is defined.

        |  The function initalises the robot position and battery level and starts a parallel thread to run the method that manages the battery.
        |  This management is carried out either manually or under request, based on the value of a parameter, here retrieved from the parameter server and stored in th '_randomness' variable.

        Args:
            self: variable that refers to the class instance

        """

        # Initialise this node.
        rospy.init_node(anm.NODE_ROBOT_STATE, log_level=rospy.INFO)
        # Initialise robot position.
        self._pose = None
        # Initialise battery level.
        self._battery_low = True # low battery
        # Initialise randomness, if enabled.
        self._randomness = rospy.get_param(anm.PARAM_RANDOM_ACTIVE, True)
        if self._randomness:
            self._random_battery_time = rospy.get_param(anm.PARAM_BATTERY_TIME, [40.0, 90.0])
            log_msg = 'Random-based battery low notification active: the battery gets low with a ' \
                      'delay in the range of [%f, %f) seconds.' % (self._random_battery_time[0], self._random_battery_time[1])
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        # Define services.
        rospy.Service(anm.SERVER_GET_POSE, GetPose, self.get_pose)
        rospy.Service(anm.SERVER_SET_POSE, SetPose, self.set_pose)
        # Start publisher on a separate thread.
        th = threading.Thread(target=self.is_battery_low)
        th.start()
        # Log information.
        log_msg = 'Initialise node `%s` with services `%s` and `%s`, and topic %s.' \
                  % (anm.NODE_ROBOT_STATE, anm.SERVER_GET_POSE, anm.SERVER_SET_POSE, anm.TOPIC_BATTERY_LOW)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    # The `robot/set_pose` service implementation.
    # The `request` input parameter is the current robot pose to be set,
    # as given by the client. This server returns an empty `response`.
    def set_pose(self, request):

        """ Function that is called every time that a new client request related to the '/state/set_pose' service is received.

        The robot position to be set is stored in a variable belonging to the class and an empty response is returned.

        Args:
            self: variable that refers to the class instance
            request (Point): service request message containing the robot position to be set
        
        Returns:
            response (empty): empty service response message

        """

        if request.pose is not None:
            # Store the new current robot position.
            self._pose = request.pose
            # Log information.
            self.print_info('Set current robot position through `%s` as (%f, %f)' \
                             % (anm.SERVER_SET_POSE, self._pose.x, self._pose.y))
        else:
            rospy.logerr(anm.tag_log('Cannot set an unspecified robot position', LOG_TAG))
        # Return an empty response.
        return SetPoseResponse()

    # The `robot/get_pose` service implementation.
    # The `request` input parameter is given by the client as empty. Thus, it is not used.
    # The `response` returned to the client contains the current robot pose.
    def get_pose(self, request):

        """ Function that is called every time that a new-client request related to the '/state/get_pose' service is received.

        First, the required robot position is retrieved from a variable belonging to the class. Then, a response message is filled with such position and sent to the client.

        Args:
            self: variable that refers to the class instance
            request (empty): empty service request message
        
        Returns:
            response (Point): service response message containing the robot position to be retrieved

        """

        # Log information.
        if self._pose is None:
            rospy.logerr(anm.tag_log('Cannot get an unspecified robot position', LOG_TAG))
        else:
            log_msg = 'Get current robot position through `%s` as (%f, %f)' \
                      % (anm.SERVER_GET_POSE, self._pose.x, self._pose.y)
            self.print_info(log_msg)
        # Create the response with the robot pose and return it.
        response = GetPoseResponse()
        response.pose = self._pose
        return response

    # Publish changes of battery levels. This method runs on a separate thread.
    def is_battery_low(self):

        """ Function that is called in the class constructor and is run in a parallel thread.

        It simply defines and initialises the publisher that publishes on the '/state/battery_low' topic and invokes one of the 'battery-management' functions based on the value of the '_randmoness' variable.

        Args:
            self: variable that refers to the class instance

        """

        # Define a `latched` publisher to wait for initialisation and publish immediately.
        publisher = rospy.Publisher(anm.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)
        if self._randomness:
            # Publish battery level changes randomly.
            self.random_battery_notifier(publisher)
        else:
            # Publish battery level changes through a keyboard-based interface.
            self.manual_battery_notifier(publisher)

    # Publish when the battery change state (i.e. low) based on a random
    # delay within the interval [`self._random_battery_time[0]`, `self._random_battery_time[1]`).
    # The message is published through the `publisher` input parameter and is a
    # boolean value, i.e. `True`: battery low
    def random_battery_notifier(self, publisher):

        """ Function that is called by the 'is_battery_low()' method if the '_randomness' variable has value 'True'.

        It simply publishes 'battery_low' messages on the '/state/battery_low' topic with a delay that, at every iteration, is randomly chosen in a predefined interval.

        Args:
            self: variable that refers to the class instance
            publisher (rospy.Publisher): publisher that publishes on the '/state/battery_low' topic

        """

        delay = 0  # Initialised to 0 just for logging purposes.
        while not rospy.is_shutdown():

            # Wait for simulate battery usage.
            delay = random.uniform(self._random_battery_time[0], self._random_battery_time[1])
            rospy.sleep(delay)
            # Publish battery level (low).
            publisher.publish(Bool(self._battery_low))
            # Log state.
            log_msg = '\033[91m' + 'Robot got low battery after %f seconds.' % delay + '\033[0m'
            self.print_info(log_msg)


    # Allow keyboard interaction to emulate battery level changes.
    # The message is published through the `publisher` input parameter and is a
    # boolean value, i.e., `True`: battery low, `False`: battery high.
    def manual_battery_notifier(self, publisher):

        """ Function that is called by the 'is_battery_low()' method if the '_randomness' variable has value 'False'.

        It simply prints a GUI on the screen and publishes 'battery_low' messages on the '/state/battery_low' topic whenever the user says so.

        Args:
            self: variable that refers to the class instance
            publisher (rospy.Publisher): publisher that publishes on the '/state/battery_low' topic

        """

        # Explain keyboard-based interaction.
        print('  # Type `Low` (`L`) to notify that the battery is low.')
        print('  # Type `cnt+C` and `Enter` to quit.')
        # Loop to enable multiple interactions.
        while not rospy.is_shutdown():
            # Wait for the user to enter a battery state.
            user_input = input(' > ')
            user_input = user_input.lower()
            # Understand the entered text.
            error = False
            if user_input == 'low' or user_input == 'l':
                rospy.loginfo(anm.tag_log('Robot got low battery.', LOG_TAG))
            else:
                # Cannot understand the entered command.
                print('*** USER INPUT ERROR! Try again:')
                error = True
            # Publish the massage based on the entered command.
            if not error:
                publisher.publish(Bool(self._battery_low))

    # Print logging only when random testing is active.
    # This is done to allow an intuitive usage of the keyboard-based interface.
    def print_info(self, msg):

        """ Function with logging purposes that is called after a 'battery_low' message is issued when the 'battery-management' is random.

        Args:
            self: variable that refers to the class instance
            msg (str): message to be logged

        """

        if self._randomness:
            rospy.loginfo(anm.tag_log(msg, LOG_TAG))


if __name__ == "__main__":

    """ Function that instantiates the 'RobotState()' class and waits.
    """
    
    # Instantiate the node manager class and wait.
    RobotState()
    rospy.spin()
