#!/usr/bin/env python
# -*- coding: utf-8 -*-
################################################


import rospy
from std_msgs.msg import String
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from geometry_msgs.msg import Point
import tf
from tf.transformations import euler_from_quaternion


#######################################################################
# FUNCTIONS

#######################################################################
# print if it's in DEBUG mode
def printlog(s, type=0):
    global DEBUG
    if DEBUG:
        if(type == 0):  # info mensage
            rospy.loginfo(s)
        elif type == 1:  # warning mensage
            rospy.logwarn(s)
        elif type == 2:
            rospy.logerr(s)  # error mensage
#######################################################################


#######################################################################
# path controller class:
# a new instance can be obtained by: x = PathController()
class PathController:

    def __filter_and_update_previous(self, current):
        if not self.__previous_x or not self.__previous_y:
            self.__previous_x = current.x
            self.__previous_y = current.y
            return current

        x = 0.1 * current.x + 0.9 * self.__previous_x
        y = 0.5 * current.y + 0.5 * self.__previous_y

        self.__previous_x = x
        self.__previous_y = y

        return x, y

#######################################################################
# Callback para a leitura da posição   (private)
    def __callback_pose(self, msg):
        # .... ADICIONAR FILTRO PARA X e Y

        self.pose_robot = msg
        self.current_pose.x, self.current_pose.y = self.__filter_and_update_previous(
            msg.pose.pose.position)

        # obtem angulo de orientacao
        quaternion = (msg.pose.pose.orientation.x,
                      msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z,
                      msg.pose.pose.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        self.current_pose.theta = yaw

#######################################################################
# Callback para a leitura da posição
    def control(self):
        if self.__init_control:

            self.__updateGoalPosition()  # atualiza o alvo

            if self.CONTROL_TYPE == 'FEEDBACK':
                self.__feedbacklinearization()

            else:  # 'OMNI'
                self.__omnidirecional()

            self.__publishVel()

#######################################################################

#######################################################################
# Inicializa ou Para o controle
    def set_control(self, init):
        if init:
            self.__init_control = 1
            self.__iterator_path = 0
            printlog("Iniciando controle...." + str(self.CONTROL_TYPE), 1)
        else:
            self.__init_control = 0
            printlog("Terminando controle....", 1)


#######################################################################
# Status controle
    def check_control(self):
        return self.__init_control

#######################################################################

#######################################################################
# Controle feedbacklinearization
    def __feedbacklinearization(self):
        # Feedbacklinearization: [v; w] = A*[vx; vy]
        err_x = self.goal_position.x - self.current_pose.x
        err_y = self.goal_position.y - self.current_pose.y

        kp = 1

        vx = kp * err_x
        vy = kp * err_y

        theta = self.current_pose.theta
        d = self.D
        v = (np.cos(theta) * vx + np.sin(theta) * vy)  # v
        w = (-np.sin(theta) / d * vx + np.cos(theta) / d * vy)  # w

        self.__norm_error_pose = (err_x**2 + err_y**2) ** (0.5)

        self.vel.linear.x = v if v <= self.MAX_VEL else self.MAX_VEL
        self.vel.angular.z = w if w <= self.MAX_VEL else self.MAX_VEL
        return v, w
#######################################################################

#######################################################################
# Controle ominidirecional
    def __omnidirecional(self):
        # ....

        printlog("x = " + str(self.current_pose.x) +
                 " y = " + str(self.current_pose.y))
        printlog("norma do erro = " + str(self.__norm_error_pose))
        printlog("vx = " + str(self.vel.linear.x) +
                 " vy = " + str(self.vel.linear.y))
#######################################################################

#######################################################################
# Atualiza o ponto objetivo do robo baseado na posição atual
    def __updateGoalPosition(self):

        self.__updateWaypoints()
        try:
            # ... alterar o __iterator_path de acordo com o necessario
            if self.__norm_error_pose < 0.4:
                self.__iterator_path += 1

            if self.__iterator_path == self.__size_path:
                self.__iterator_path = 0

            self.goal_position.x = self.waypoints[self.__iterator_path][0]
            self.goal_position.y = self.waypoints[self.__iterator_path][1]
            self.goal_position.theta = self.waypoints[self.__iterator_path][2]

            printlog("UpdateGoal new goal position x= " + str(self.goal_position.x) + " y=" +
                     str(self.goal_position.y) + " th=" + str(self.goal_position.theta * 180 / np.pi))

        except IndexError:
            printlog("size = " + str(self.__size_path) +
                     " iterator_path = " + str(self.__iterator_path))
            self.set_control(False)  # STOP
            printlog("ERROR UPDATE GOAL POSITION", 2)

#######################################################################

#######################################################################
# Atualiza se houver novos waypoints (Private)
    def __updateWaypoints(self):
        new_waypoints = rospy.get_param(
            "~waypoints", np.array([[self.D, 0.0, 0.0]]))
        new_waypoints = np.array(new_waypoints)

        if not (new_waypoints == self.waypoints).all():
            self.new_waypoints = new_waypoints
            self.__size_path = len(self.new_waypoints)
            self.isreceived_newpath = 1
            self.set_control(True)
            printlog("updateWaypoints user", 1)
#######################################################################

#######################################################################
# Publica velocidade (Private)
    def __publishVel(self):
        printlog(self.vel)
        # vel = Twist()
        # vel.linear.x = 10
        # vel.linear.y = 10
        # self.vel = vel
        self.pub_vel.publish(self.vel)
        # ...

#######################################################################

#######################################################################
# construtor da classe
    def __init__(self):

        self.TOPIC_ODOM_SUB = rospy.get_param(
            "~TOPIC_ODOM_SUB", '/robot_0/odom')
        self.TOPIC_VEL_PUB = rospy.get_param("~TOPIC_VEL_PUB", '/cmd_vel')
        self.D = rospy.get_param("~D_FEEDBACK", 0.5)
        self.MAX_VEL = rospy.get_param("~MAX_VEL", 1.0)
        self.CONTROL_TYPE = rospy.get_param("~CONTROL_TYPE", 'FEEDBACK')
        self.MAX_NORM_ERROR = rospy.get_param("~MAX_NORM_ERROR", 0.15)

        self.waypoints = rospy.get_param(
            "~waypoints", np.array([[self.D, 0.0, 0.0]]))
        self.waypoints = np.array(self.waypoints)
        self.__size_path = len(self.waypoints)

        self.pose_robot = Odometry()
        self.vel = Twist()
        self.goal_position = Pose2D()
        self.current_pose = Pose2D()
        self.goal_position = Pose2D()

        self.__init_control = 0
        self.__isreceived_newpath = 0
        self.__iterator_path = 0
        self.__norm_error_pose = self.D + 0.5

        self.__previous_x = None
        self.__previous_y = None

        # Subscriber and Publisher
        rospy.Subscriber(self.TOPIC_ODOM_SUB, Odometry,
                         self.__callback_pose, queue_size=1)
        self.pub_vel = rospy.Publisher(self.TOPIC_VEL_PUB, Twist, queue_size=1)

        printlog("############## WAYPOINTS ##############")
        printlog("size path = " + str(self.__size_path), 1)
        printlog(self.waypoints)
        printlog("#######################################")

#######################################################################
# END CLASS PathController
#######################################################################


#######################################################################
# MAIN MODULE
if __name__ == '__main__':
    try:

        rospy.init_node('path_controller')
        rate = rospy.Rate(rospy.get_param("~RATE", 30))

        DEBUG = rospy.get_param("~DEBUG", 1)
        REPEAT = rospy.get_param("~REPEAT", 1)

        # cria uma instância da classe PathController
        pathController = PathController()
        pathController.set_control(True)

        while not rospy.is_shutdown():

            pathController.control()
            # acrescentar algo para ficar em loop
            rate.sleep()

    except rospy.ROSInterruptException():
        pass
