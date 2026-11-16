#!/bin/bash
# Enchaîne plusieurs vols de la mission par waypoints, puis lance l'analyse.
# Prérequis : simulation, agent micro-xrce-dds-agent et QGroundControl lancés,
# QGroundControl affichant « Ready To Fly ».
NB_VOLS=${1:-3}
PAUSE_S=15

source /opt/ros/jazzy/setup.bash
source ~/drone_ws/install/setup.bash

for i in $(seq 1 "$NB_VOLS"); do
    echo "=============== Vol $i/$NB_VOLS ==============="
    python3 ~/drone_ws/offboard_mission.py
    dernier=$(ls -t ~/drone_ws/logs/trajectoire_*.csv 2>/dev/null | head -1)
    if [ -z "$dernier" ] || ! grep -q ",HOLD,5," "$dernier"; then
        echo "Vol $i incomplet : arrêt. Vérifiez QGroundControl (« Ready To Fly » ?)."
        exit 1
    fi
    if [ "$i" -lt "$NB_VOLS" ]; then
        echo "Vol $i terminé. Pause de ${PAUSE_S} s avant le suivant..."
        sleep "$PAUSE_S"
    fi
done

echo
echo "=============== Analyse ==============="
PYTHONNOUSERSITE=1 python3 ~/drone_ws/analyse_repetabilite.py
