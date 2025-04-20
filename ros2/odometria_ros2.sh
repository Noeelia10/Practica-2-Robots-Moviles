#!/bin/bash
# -----------------------------------------------
# Script explicativo para la odometría con ROS2
# Archivo: odometria_ros2.sh
# -----------------------------------------------

echo ""
echo "========= ODOMETRÍA EN ROS 2 ========="
echo ""

echo "[1] Crear paquete en ROS2"
echo "   > ros2 pkg create --build-type ament_python mi_odometria"
echo "   ROS1: catkin_create_pkg mi_odometria std_msgs rospy roscpp"

echo ""
echo "[2] Crear el nodo para leer orientación (leer_orientacion.py)"
echo "   Este nodo se suscribe a /odom y convierte el cuaternión en ángulos de Euler."
echo "   Se utiliza tf_transformations para el cálculo."
echo "   ROS1: Igual, usando tf.transformations.euler_from_quaternion"

echo ""
echo "[3] Añadir el punto de entrada en setup.py:"
echo "   entry_points={"
echo "       'console_scripts': ['leer_orientacion = mi_odometria.leer_orientacion:main']"
echo "   }"
echo "   ROS1: No se usa setup.py en catkin, se usan archivos launch o directamente rosrun"

echo ""
echo "[4] Instalar dependencias necesarias"
echo "   > sudo apt install ros-humble-tf-transformations"
echo "   > sudo pip3 install transforms3d"
echo "   ROS1: sudo apt install ros-noetic-tf"

echo ""
echo "[5] Compilar el workspace"
echo "   > cd ~/ros2_ws"
echo "   > colcon build --packages-select mi_odometria"
echo "   > source install/setup.bash"
echo "   ROS1: catkin_make + source devel/setup.bash"

echo ""
echo "[6] Ejecutar el nodo"
echo "   > ros2 run mi_odometria leer_orientacion"
echo "   ROS1: rosrun mi_odometria leer_orientacion.py"

echo ""
echo "[7] Control de orientación (control_orientacion.py)"
echo "   Nodo que utiliza la orientación deseada (yaw) para ajustar el giro del robot."
echo "   Se usa un control proporcional sobre el ángulo de error."

echo ""
echo "[8] Comandos adicionales usados durante el proceso:"
echo "   - ros2 topic echo /odom"
echo "   - ros2 node list"
echo "   - ros2 interface show nav_msgs/msg/Odometry"
echo ""
echo "   ROS1 equivalente:"
echo "   - rostopic echo /odom"
echo "   - rosnode list"
echo "   - rosmsg show nav_msgs/Odometry"

echo ""
echo "========= FIN ========="
