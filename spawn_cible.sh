#!/bin/bash
# Place une cible rouge (boîte de 1 x 1 x 3 m) dans la simulation Gazebo en cours.
# Usage : spawn_cible.sh X Y   (X = est, Y = nord, en mètres, repère Gazebo)
X=${1:-8}
Y=${2:-0}
NOM=$(echo "cible_rouge_${X}_${Y}" | tr '.-' '__')
FICHIER="/tmp/${NOM}.sdf"
cat > "$FICHIER" << SDF
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="${NOM}">
    <static>true</static>
    <pose>${X} ${Y} 1.5 0 0 0</pose>
    <link name="link">
      <visual name="visuel">
        <geometry><box><size>1 1 3</size></box></geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
SDF
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean --timeout 3000 --req "sdf_filename: \"${FICHIER}\""
