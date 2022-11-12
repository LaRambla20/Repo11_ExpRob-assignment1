#! /usr/bin/env python

"""
.. module:: planner
    :platform: Unix
    :synopsis: Python module that implements a dummy planner that simulates the generation of a path towards a desired location

.. moduleauthor:: Emanuele Rambaldi <emanuele.rambaldi3@studio.unibo.it>

Given a target location, via a client request, the node at issue retrieves the current robot position from the 'robot-states' node, and returns a plan as a set of via-points.

Client:
    - /state/get_pose

Action service:
    - motion/planner

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
from exprob_first_assignment.msg import Point, PlanFeedback, PlanResult
from exprob_first_assignment.srv import GetPose
import exprob_first_assignment # This is required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.


# A tag for identifying logs producer.
LOG_TAG = anm.NODE_PLANNER


# An action server to simulate motion planning.
# Given a target position, it retrieve the current robot position from the 
# `robot-state` node, and return a plan as a set of via points.
class PlaningAction(object):

    """Class that is composed of several methods aimed at simulating the generation of a path towards a desired location.

    """

    def __init__(self):

        """ (constructor) Function that is called whenever an instance of this class is defined.

        |  First, some parameters, needed for determining the functioning of the dummy planning algorithm, are retrieved from the parameter server. 
        |  Then, the actions server that answers to requests belonging to the 'motion/planner' action service and assigns the 'execute_callback' function to it.
        
        Args:
            self: variable that refers to the class instance

        """

        # Get random-based parameters used by this server
        self._random_plan_points = rospy.get_param(anm.PARAM_PLANNER_POINTS, [2, 8])
        self._random_plan_time = rospy.get_param(anm.PARAM_PLANNER_TIME, [0.1, 1])
        self._environment_size = rospy.get_param(anm.PARAM_ENVIRONMENT_SIZE)
        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(anm.ACTION_PLANNER, 
                                      exprob_first_assignment.msg.PlanAction,  
                                      execute_cb=self.execute_callback, 
                                      auto_start=False)
        self._as.start()
        # Log information.
        log_msg = '`%s` Action Server initialised. It will create random path with a number of point ' \
                  'spanning in [%d, %d). Each point will be generated with a delay spanning in [%f, %f).' \
                  % (anm.ACTION_PLANNER, self._random_plan_points[0], self._random_plan_points[1],
                     self._random_plan_time[0], self._random_plan_time[1])
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
      
    # The callback invoked when a client set a goal to the `planner` server.
    # This function will return a list of random points (i.e., the plan) when the fist point
    # is the current robot position (retrieved from the `robot-state` node), while the last 
    # point is the `goal` position (randomly determined since it is a dummy implementation). The plan will contain 
    # a random number of other points, which spans in the range 
    # [`self._random_plan_points[0]`, `self._random_plan_points[1]`). To simulate computation,
    # each point is added to the plan with a random delay spanning in the range 
    # [`self._random_plan_time[0]`, `self._random_plan_time[1]`).
    def execute_callback(self, goal):

        """ Function that is called every time that a client goal-request related to the 'motion/planner' action service is received.

        |  This function will return a list of random points (i.e., the path), where the first point is the current robot position (retrieved from the 'robot-state' node), while the last 
        |  point is the 'goal' position (randomly determined since it is a dummy implementation). The path will contain a random number of other points, which spans in a predefined range. 
        |  To simulate computation, each point is added to the plan with a random delay spanning in a predefined range as well.
        |  The request message contains the name of the supposed target location, which here is simply printed on the screen, but will be needed in a proper implementation of the planner. 

        Args:
            self: variable that refers to the class instance
            goal (str): goal request message containing the target location to generate the path to
        
        Returns:
            result (Point list): response message containing the generated path as a list of via-points

        """

        # Get the input parameters to compute the plan, i.e., the start (or current) and target positions.
        start_point = _get_pose_client()
        # target_point = goal.target
        target_location = goal.location

        # EVVALUATE A RANDOM TARGET POINT
        target_point = Point()
        target_point.x = random.uniform(0, self._environment_size[0]) #extract randomly a float number inside the boundaries
        target_point.y = random.uniform(0, self._environment_size[1]) #extract randomly a float number inside the boundaries
        print('\033[96m' + "Start and target points" + '\033[0m')
        print(start_point)
        print(target_point)

        # Check if the start and target positions are correct. If not, this service will be aborted.
        if start_point is None or target_point is None:
            log_msg = 'Cannot have `None` start point nor target_point. This service will be aborted!.'
            rospy.logerr(anm.tag_log(log_msg, LOG_TAG))
            # Close service by returning an `ABORT` state to the client.
            self._as.set_aborted()
            return
        
        # Initialise the `feedback` with the starting point of the plan.
        feedback = PlanFeedback()
        feedback.via_points = []
        feedback.via_points.append(start_point)
        # Publish the feedback and wait to simulate computation.
        self._as.publish_feedback(feedback)
        delay = random.uniform(self._random_plan_time[0], self._random_plan_time[1])
        rospy.sleep(delay)

        # Get a random number of via points to be included in the plan.
        number_of_points = random.randint(self._random_plan_points[0], self._random_plan_points[1] + 1)
        log_msg = 'PLANNER: evaluating a path to the desired location (%s) ' % (target_location)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        log_msg = 'Server is planning %i points...' % (number_of_points + 1)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        # Generate the points of the plan.
        for i in range(1, number_of_points):
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                rospy.loginfo(anm.tag_log('Server has been cancelled by the client!', LOG_TAG))
                # Actually cancel this service.
                self._as.set_preempted()  
                return
            # Generate a new random point of the plan.
            new_point = Point()
            new_point.x = random.uniform(0, self._environment_size[0])
            new_point.y = random.uniform(0, self._environment_size[1])
            feedback.via_points.append(new_point)
            if i < number_of_points - 1:
                # Publish the new random point as feedback to the client.
                self._as.publish_feedback(feedback)
                # Wait to simulate computation.
                delay = random.uniform(self._random_plan_time[0], self._random_plan_time[1])
                rospy.sleep(delay)
            else:
                # Append the target point to the plan as the last point.
                feedback.via_points.append(target_point)

        # Publish the results to the client.        
        result = PlanResult()
        result.via_points = feedback.via_points
        self._as.set_succeeded(result)
        log_msg = 'The motion plan to the desired location (%s) has been generated! ' % (target_location)
        log_msg += ''.join('(' + str(point.x) + ', ' + str(point.y) + '), ' for point in result.via_points)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))


# Retrieve the current robot pose by the `state/get_pose` server of the `robot-state` node.
def _get_pose_client():

    """ Function that is called whenever the action server determines a path towards a target location.

    This function simply sends a request belonging to the '/state/get_pose' service to the 'robot-states' node to retrieve the current position of the robot.
    
    Returns:
        pose (Point): current position of the robot

    """

    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service(anm.SERVER_GET_POSE)
    try:
        # Call the service and get a response with the current robot position.
        service = rospy.ServiceProxy(anm.SERVER_GET_POSE, GetPose)
        response = service()
        pose = response.pose
        # Log service response.
        log_msg = 'Retrieving current robot position from the `%s` node as: (%f, %f).' \
                  % (anm.NODE_ROBOT_STATE, pose.x, pose.y)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        return pose
    except rospy.ServiceException as e:
        log_msg = 'Server cannot get current robot position: %s' % e
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))


if __name__ == '__main__':

    """ Function that instantiates the 'PlanningAction()' class and waits.
    """

    # Initialise the node, its action server, and wait.    
    rospy.init_node(anm.NODE_PLANNER, log_level=rospy.INFO)
    server = PlaningAction()
    rospy.spin()