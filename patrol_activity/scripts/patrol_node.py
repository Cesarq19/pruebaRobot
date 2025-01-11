#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class PatrolNode:
    def __init__(self):
        rospy.init_node('patrol_node', anonymous=True)

        # Definir los puntos de patrullaje (en formato [x, y, theta])
        self.waypoints = [
            [1.0, 1.0, 0.0],
            [2.0, 1.0, 0.0],
            [2.0, 2.0, 1.57],
            [1.0, 2.0, 3.14]
        ]

        # Cliente para enviar metas a move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Esperando el servidor de move_base...")
        self.client.wait_for_server()
        rospy.loginfo("Servidor de move_base conectado.")

        self.run_patrol()

    def create_goal(self, x, y, theta):
        """Crea un objetivo MoveBaseGoal dado x, y, y theta."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = rospy.sin(theta / 2.0)
        goal.target_pose.pose.orientation.w = rospy.cos(theta / 2.0)

        return goal

    def run_patrol(self):
        """Patrulla los puntos en bucle."""
        while not rospy.is_shutdown():
            for point in self.waypoints:
                x, y, theta = point
                goal = self.create_goal(x, y, theta)

                rospy.loginfo(f"Enviando meta: x={x}, y={y}, theta={theta}")
                self.client.send_goal(goal)

                self.client.wait_for_result()
                if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Meta alcanzada.")
                else:
                    rospy.logwarn("Falló al alcanzar la meta. Intentando la siguiente.")
            
            rospy.loginfo("Ciclo de patrullaje completado. Repitiendo...")

if __name__ == '__main__':
    try:
        PatrolNode()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo de patrullaje terminado.")
