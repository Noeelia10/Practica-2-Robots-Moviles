#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time

class Aparcador:
    def __init__(self):
        rospy.init_node('nodo_aparcamiento')
        self.scan = None
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.tasa = rospy.Rate(10)
        self.estado = "buscando"

    def scan_callback(self, msg):
        self.scan = msg

    def mover(self, vel_lineal, vel_angular, duracion):
        cmd = Twist()
        cmd.linear.x = vel_lineal
        cmd.angular.z = vel_angular
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < duracion and not rospy.is_shutdown():
            self.pub_cmd.publish(cmd)
            self.tasa.sleep()
        self.pub_cmd.publish(Twist())

    def aparcar(self):
        # Avanza 0.2 m hacia el hueco
        self.mover(0.1, 0.0, 2)
        # Gira 180 grados
        self.mover(0.0, math.pi / 8, 8)
        # Espera aparcado
        rospy.sleep(10)
        # Sale del hueco
        self.mover(0.1, 0.0, 2)
        # Gira 90 grados para continuar
        self.mover(0.0, -math.pi / 8, 4)

    def ejecutar(self):
        while not rospy.is_shutdown():
            if self.scan:
                ranges = self.scan.ranges
                angle_min = self.scan.angle_min
                angle_increment = self.scan.angle_increment

                # Ángulo de 90º (lateral derecho)
                index_90 = int((math.pi / 2 - angle_min) / angle_increment)

                # Zona frontal del hueco a detectar
                inicio = max(index_90 - 5, 0)
                fin = min(index_90 + 5, len(ranges) - 1)

                medidas_laterales = [r for r in ranges[inicio:fin] if r > 0.3 and r < float('inf')]

                if len(medidas_laterales) >= 10:
                    rospy.loginfo("Hueco detectado. Iniciando maniobra de aparcamiento.")
                    self.aparcar()
            # Si no hay hueco, sigue recto
            self.mover(0.05, 0.0, 0.1)

if __name__ == '__main__':
    try:
        aparcador = Aparcador()
        aparcador.ejecutar()
    except rospy.ROSInterruptException:
        pass
