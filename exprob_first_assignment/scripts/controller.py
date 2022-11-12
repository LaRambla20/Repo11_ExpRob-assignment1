#! /usr/bin/env python

"""
.. module:: controller
    :platform: Unix
    :synopsis: Python module that implements a dummy controller that simulates the control of the robot along a generated path

.. moduleauthor:: Emanuele Rambaldi <emanuele.rambaldi3@studio.unibo.it>

|  Given a path as a set of via-points, via a client request, the node at issue simulates the movements to reach each point with a random delay. 
|  Furthermore the node updates the current robot position stored in the 'robot-states' node every time that a via-point has supposedly been reached.

Client:
    - /state/set_pose

Action service:
    - motion/controller

"""

import random
import rospy
# Import constant name defined to structure the architecture.
import sys
sys.path.append('/home/emanuelerambaldi/ROS/my_ros_ws/src/exprob_first_assignment/scripts')
import architecture_name_mapper as anm
# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from exprob_first_assignment.msg import ControlFeedback, ControlResult
from exprob_first_assignment.srv import SetPose
import exprob_first_assignment  # This is required to pass the `ControlAction` type for instantiating the `SimpleActionServer`.

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_CONTROLLER


# An action server to simulate motion controlling.
# Given a plan as a set of via points, it simulate the movements
# to reach each point with a random delay. This server updates
# the current robot position stored in the `robot-state` node.
class ControllingAction(object):

    """Class that is composed of several methods aimed at simulating the control of the robot along a generated path.

    """

    def __init__(self):

        """ (constructor) Function that is called whenever an instance of this class is defined.

        |  First, a parameter, needed for determining the functioning of the dummy control algorithm, is retrieved from the parameter server. 
        |  Then, the actions server that answers to requests belonging to the 'motion/controller' action service and assigns the 'execute_callback' function to it.
        
        Args:
            self: variable that refers to the class instance

        """

        # Get random-based parameters used by this server
        self._random_motion_time = rospy.get_param(anm.PARAM_CONTROLLER_TIME, [0.1, 2.0])
        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(anm.ACTION_CONTROLLER,
                                      # arch_skeleton.msg.ControlAction,
                                      exprob_first_assignment.msg.ControlAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self._as.start()
        # Log information.
        log_msg = '`%s` Action Server initialised. It will navigate trough the plan with a delay between each via point ' \
                  'spanning in [%f, %f).' % (anm.ACTION_CONTROLLER, self._random_motion_time[0], self._random_motion_time[1])
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    # The callback invoked when a client set a goal to the `controller` server.
    # This function requires a list of via points (i.e., the plan), and it simulate
    # a movement through each point with a delay spanning in 
    # ['self._random_motion_time[0]`, `self._random_motion_time[1]`).
    # As soon as each via point is reached, the related robot position is updated
    # in the `robot-state` node.
    def execute_callback(self, goal):

        """ Function that is called every time that a client goal-request related to the 'motion/controller' action service is received.

        |  This function requires a list of via points (i.e., the path), and it simulates a movement through each point with a delay spanning in a predefined range.
        |  As soon as each via point is reached, the related robot position is updated in the 'robot-state' node.

        Args:
            self: variable that refers to the class instance
            goal (Point list): goal request message containing the path to follow as a list of via-points
        
        Returns:
            result (Point): response message consisting in the reached point

        """

        # Check if the provided plan is processable. If not, this service will be aborted.
        if goal is None or goal.via_points is None or len(goal.via_points) == 0:
            rospy.logerr(anm.tag_log('No via points provided! This service will be aborted!', LOG_TAG))
            self._as.set_aborted()
            return

        # Construct the feedback and loop for each via point.
        feedback = ControlFeedback()
        rospy.loginfo(anm.tag_log('CONTROLLER: guiding the robot towards the desired location', LOG_TAG)) 
        for point in goal.via_points:
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                rospy.loginfo(anm.tag_log('Service has been cancelled by the client!', LOG_TAG))
                # Actually cancel this service.
                self._as.set_preempted()
                return
                # Wait before to reach the following via point. This is just for testing purposes.
            delay = random.uniform(self._random_motion_time[0], self._random_motion_time[1])
            rospy.sleep(delay)
            # Publish a feedback to the client to simulate that a via point has been reached. 
            feedback.reached_point = point
            self._as.publish_feedback(feedback)
            # Set the new current position into the `robot-state` node.
            _set_pose_client(point)
            # Log current robot position.
            log_msg = 'Reaching point (%f, %f).' % (point.x, point.y)
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        # Publish the results to the client.
        result = ControlResult()
        result.reached_point = feedback.reached_point
        rospy.loginfo(anm.tag_log('Motion control successes.', LOG_TAG))
        self._as.set_succeeded(result)
        return  # Succeeded.


# Update the current robot `pose` stored in the `robot-state` node.
# This method is performed for each point provided in the action's server feedback.
def _set_pose_client(pose):

    """ Function that is called whenever the action server simulates the reaching of a path via-point.

    This function simply sends a request belonging to the '/state/set_pose' service to the 'robot-states' node to update the current position of the robot.
    
    Args:
        pose (Point): current position of the robot

    """

    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service(anm.SERVER_SET_POSE)
    try:
        # Log service call.
        log_msg = 'Set current robot position to the `%s` node.' % anm.SERVER_SET_POSE
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        # Call the service and set the current robot position.
        service = rospy.ServiceProxy(anm.SERVER_SET_POSE, SetPose)
        service(pose)  # The `response` is not used.
    except rospy.ServiceException as e:
        log_msg = 'Server cannot set current robot position: %s' % e
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))


if __name__ == '__main__':

    """ Function that instantiates the 'COntrollingAction()' class and waits.
    """

    # Initialise the node, its action server, and wait.   
    rospy.init_node(anm.NODE_CONTROLLER, log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()
