#!/bin/bash
# Start the 2D multi-robot simulator and optional tools

CONFIG_PATH="/home/lattinone/ros-multi-robot-sim/src/ros_2d_multi_robot_simulator/configs/config1.rviz"

# Controlla che il file esista davvero
if [ ! -f "$CONFIG_PATH" ]; then
    echo " ERRORE: il file $CONFIG_PATH non esiste!"
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

# Start simulator
xterm -hold -fa 'Monospace' -e "bash -i -c './devel/lib/ros_2d_multi_robot_simulator/robsim_node'" &
