#!/bin/bash
# Start the 2D multi-robot simulator and optional tools automatically

CONFIG_PATH="src/ros_2d_multi_robot_simulator/configs/config1.rviz"

# Controlla che il file esista davvero
if [ ! -f "$CONFIG_PATH" ]; then
    echo "ERRORE: il file $CONFIG_PATH non esiste!"
    echo "Crea o sposta qui il tuo config1.rviz prima di continuare."
    exit 1
fi

echo "========================================"
echo "  Avvio simulatore multi-robot 2D"
echo "========================================"

# ---- PARAMETRI AUTOMATICI ----
START_RVIZ=true
START_RQT_GRAPH=true
START_MOVEMENT=true
# Puoi disabilitare temporaneamente qualcosa mettendo "false"
# Es: START_RVIZ=false

# ---- AVVIO ROSCORE ----
echo "[1/5] Avvio roscore..."
xterm -hold -title "roscore" -e "bash -i -c 'roscore'" &
sleep 2

# ---- AVVIO SIMULATORE ----
echo "[2/5] Avvio simulatore..."
xterm -hold -fa 'Monospace' -title "Simulator" -e "bash -i -c './devel/lib/ros_2d_multi_robot_simulator/robsim_node'" &
sleep 3

# ---- RVIZ ----
if [ "$START_RVIZ" = true ]; then
    echo "[3/5] Avvio RViz con config: $CONFIG_PATH"
    xterm -hold -title "RViz" -e "bash -i -c 'rviz -d \"$CONFIG_PATH\"'" &
fi

# ---- RQT_GRAPH ----
if [ "$START_RQT_GRAPH" = true ]; then
    echo "[4/5] Avvio rqt_graph..."
    xterm -hold -title "rqt_graph" -e "bash -i -c 'rqt_graph'" &
fi


echo ""
echo "========================================"
echo " Tutti i processi sono stati avviati!"
echo "   Puoi chiudere le finestre xterm per terminare singoli processi."
echo "========================================"
