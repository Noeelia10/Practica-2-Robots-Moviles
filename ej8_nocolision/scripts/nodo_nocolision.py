#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SeguridadLIDAR:
    def __init__(self):
        rospy.init_node('filtro_seguridad')

        self.umbral_seguridad = 0.4  # metros
        self.rangos_visibles = 10    # muestras del frontal

        self.lidar = None
        self.cmd_actual = Twist()

        rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        rospy.Subscriber('/cmd_vel', Twist, self.callback_cmd)

        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # directamente a cmd_vel

        rospy.Timer(rospy.Duration(0.1), self.controlar_movimiento)
        rospy.loginfo("Nodo de seguridad activado.")

    def callback_lidar(self, msg):
        self.lidar = msg

    def callback_cmd(self, msg):
        self.cmd_actual = msg

    def controlar_movimiento(self, event):
        if self.lidar is None:
            return

        ranges = self.lidar.ranges
        frontal_izq = ranges[:self.rangos_visibles]
        frontal_der = ranges[-self.rangos_visibles:]
        frontal = [r for r in (frontal_izq + frontal_der) if 0.0 < r < float('inf')]

        peligro = frontal and min(frontal) < self.umbral_seguridad

        if peligro and self.cmd_actual.linear.x > 0:
            # Detener movimiento inmediatamente
            stop = Twist()
            self.pub_cmd.publish(stop)
            rospy.logwarn(f"Obstáculo a {min(frontal):.2f} m. Movimiento cancelado.")
        else:
            self.pub_cmd.publish(self.cmd_actual)
            rospy.loginfo(f"Movimiento permitido: avance {self.cmd_actual.linear.x:.2f} m/s")

if __name__ == '__main__':
    try:
        SeguridadLIDAR()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



