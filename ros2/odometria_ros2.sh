#!/bin/bash

echo "============================"
echo "  INICIO DE SESIÓN EN EL ROBOT"
echo "============================"

echo "1. Conectarse al TurtleBot3 por SSH"
echo "   IP del robot: 192.168.1.230"
echo "   Comando:"
echo "   ssh ubuntu@192.168.1.230"
echo "   Usuario: ubuntu | Contraseña: turtlebot"
echo "--------------------------------------------"

echo "2. Configurar el ROS_DOMAIN_ID en el robot"
echo "   nano ~/.bashrc"
echo "   Añadir al final: export ROS_DOMAIN_ID=30 (o 31 según el robot)"
echo "   Luego ejecutar:"
echo "   source ~/.bashrc"
echo "   (Equivalente en ROS 1: export ROS_MASTER_URI + export ROS_HOSTNAME)"
echo "--------------------------------------------"

echo "3. Definir el modelo del robot"
echo "   Añadir en ~/.bashrc:"
echo "   export TURTLEBOT3_MODEL=burger"
echo "   source ~/.bashrc"
echo "   (Mismo en ROS 1)"
echo "--------------------------------------------"

echo "4. Lanzar bringup para arrancar los sensores del robot"
echo "   ros2 launch turtlebot3_bringup robot.launch.py"
echo "   (ROS 1: roslaunch turtlebot3_bringup turtlebot3_robot.launch)"
echo "--------------------------------------------"


echo "============================"
echo "  EN LA ESTACIÓN LOCAL (TU ORDENADOR)"
echo "============================"

echo "5. Conectar terminal local a la misma red y configurar ROS_DOMAIN_ID"
echo "   nano ~/.bashrc"
echo "   Añadir: export ROS_DOMAIN_ID=30 (igual que en el robot)"
echo "   source ~/.bashrc"
echo "   (ROS 1: export ROS_MASTER_URI=http://<robot_ip>:11311)"
echo "--------------------------------------------"

echo "6. Comprobar la conexión con el robot"
echo "   ros2 topic list"
echo "   (ROS 1: rosnode list)"
echo "--------------------------------------------"

echo "7. Visualización con RViz2"
echo "   ros2 launch turtlebot3_bringup rviz2.launch.py"
echo "   (ROS 1: roslaunch turtlebot3_bringup turtlebot3_rviz.launch)"
echo "--------------------------------------------"

echo "8. Teleoperación del robot"
echo "   ros2 run turtlebot3_teleop teleop_keyboard"
echo "   (ROS 1: roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch)"
echo "--------------------------------------------"


echo "============================"
echo "  GENERACIÓN Y USO DEL MAPA"
echo "============================"

echo "9. Crear un mapa con SLAM"
echo "   ros2 launch turtlebot3_cartographer cartographer.launch.py"
echo "   Mover el robot mientras se va construyendo el mapa"
echo "   (ROS 1: roslaunch turtlebot3_slam turtlebot3_slam.launch)"
echo "--------------------------------------------"

echo "10. Guardar el mapa generado"
echo "    ros2 run nav2_map_server map_saver_cli -f ~/map"
echo "    (ROS 1: rosrun map_server map_saver -f ~/map)"
echo "--------------------------------------------"

echo "11. Ejecutar navegación en el mapa"
echo "    ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml"
echo "    (ROS 1: roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml)"
echo "--------------------------------------------"

echo "12. En RViz2: Localizar el robot manualmente y establecer la meta"
echo "    - Utilizar la herramienta '2D Pose Estimate' para posicionar el robot"
echo "    - Usar 'Nav Goal' para darle un destino"
echo "--------------------------------------------"


echo "============================"
echo "  FINALIZAR SESIÓN"
echo "============================"

echo "13. Apagar el robot de forma segura"
echo "    sudo shutdown -h now"
echo "--------------------------------------------"

echo " SCRIPT COMPLETADO ✅ "

