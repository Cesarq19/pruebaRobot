#!/usr/bin/env python

import rospy
import actionlib
import cv2
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction
from cv_bridge import CvBridge, CvBridgeError

class PersonFollowNode:
    def __init__(self):
        rospy.init_node('person_follow_node', anonymous=True)

        # Inicializa el cliente para move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        # Suscribirse a los tópicos
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # Publicador para controlar directamente el robot
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Variables
        self.bridge = CvBridge()
        self.person_detected = False
        self.safe_distance = 1.5  # Distancia segura en metros
        self.target_distance = None

        rospy.loginfo("Nodo de seguimiento de personas inicializado.")
        rospy.spin()

    def image_callback(self, msg):
        """Callback para procesar las imágenes de la cámara."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("Error al convertir la imagen: %s", e)
            return

        # Usar un modelo simple de detección de personas (ejemplo con HOG + SVM de OpenCV)
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        boxes, _ = hog.detectMultiScale(cv_image, winStride=(8, 8))

        if len(boxes) > 0:
            rospy.loginfo("Persona detectada.")
            self.person_detected = True
        else:
            self.person_detected = False

        # Dibujar las cajas en la imagen (opcional para depuración)
        for (x, y, w, h) in boxes:
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Mostrar la imagen con las detecciones (opcional para depuración)
        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(1)

    def scan_callback(self, msg):
        """Callback para procesar los datos del LiDAR."""
        if self.person_detected:
            # Calcular la distancia más cercana (0 grados hacia adelante)
            ranges = msg.ranges
            forward_distance = min(ranges) if ranges else None
            self.target_distance = forward_distance

            if forward_distance is not None:
                self.adjust_distance(forward_distance)

    def adjust_distance(self, distance):
        """Ajustar la distancia al objetivo."""
        twist = Twist()
        if distance > self.safe_distance:
            # Acercarse a la persona
            twist.linear.x = 0.2
        elif distance < self.safe_distance:
            # Retroceder si está demasiado cerca
            twist.linear.x = -0.2
        else:
            # Detenerse si está en la distancia segura
            twist.linear.x = 0.0

        self.cmd_vel_pub.publish(twist)

    def cancel_patrol(self):
        """Cancelar el patrullaje actual."""
        rospy.loginfo("Cancelando patrullaje para seguir a la persona.")
        self.move_base_client.cancel_all_goals()

    def resume_patrol(self):
        """Reanudar el patrullaje."""
        rospy.loginfo("Persona no detectada. Reanudando patrullaje.")
        # Aquí puedes implementar la lógica para volver a activar el nodo de patrullaje.


if __name__ == '__main__':
    try:
        PersonFollowNode()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo de seguimiento de personas terminado.")
