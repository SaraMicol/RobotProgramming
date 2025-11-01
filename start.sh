#!/bin/bash
# Start the 2D multi-robot simulator and optional tools

CONFIG_PATH="/home/lattinone/ros-multi-robot-sim/src/ros_2d_multi_robot_simulator/configs/config1.rviz"

# Controlla che il file esista davvero
if [ ! -f "$CONFIG_PATH" ]; then
    echo "ERRORE: il file $CONFIG_PATH non esiste!"
    echo "Crea o sposta qui il tuo config1.rviz prima di continuare."
    exit 1
fi

echo "Do you want to start rviz? (y/n)"
read start_rviz

if [ "$start_rviz" == "y" ]; then
    # Apri RViz caricando la tua configurazione specifica
    xterm -hold -e "bash -i -c 'echo Loading RViz config: $CONFIG_PATH; rviz -d \"$CONFIG_PATH\"'" &
fi

echo "Do you want to start rqt_graph? (y/n)"
read start_rqt_graph

if [ "$start_rqt_graph" == "y" ]; then
    xterm -hold -e "bash -i -c 'rqt_graph'" &
fi

# Start roscore
xterm -hold -e "bash -i -c 'roscore'" &

sleep 2

# Start simulator
xterm -hold -fa 'Monospace' -e "bash -i -c './devel/lib/ros_2d_multi_robot_simulator/robsim_node'" &

sleep 3

# Chiedi se far muovere i robot
echo "Do you want to start robot movement? (y/n)"
read start_movement

if [ "$start_movement" == "y" ]; then
    echo "Starting robot movement commands..."
    
    # Robot 1 - movimento in avanti con rotazione
    xterm -title "Robot 1 cmd_vel" -hold -e "bash -i -c 'rostopic pub /1/cmd_vel geometry_msgs/Twist \"linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2\" -r 10'" &
    
    # Robot 2 - movimento in avanti con rotazione opposta
    xterm -title "Robot 2 cmd_vel" -hold -e "bash -i -c 'rostopic pub /2/cmd_vel geometry_msgs/Twist \"linear:
  x: 0.3
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.1\" -r 10'" &
    
    echo "Robot movement commands started!"
    echo "You can modify velocities in the xterm windows or close them to stop movement."
fi

echo ""
echo "All processes started!"
echo "Close the xterm windows to stop individual processes."