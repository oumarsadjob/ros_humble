#!/bin/bash

# Fonction pour lancer Cartographer
launch_cartographer() {
    echo "Lancement de Cartographer..."
    ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
}

# Fonction pour lancer Gazebo
launch_gazebo() {
    echo "Lancement de Gazebo..."
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
}

# Fonction pour lancer Cartographer et Gazebo
launch_cartographer_and_gazebo() {
    echo "Lancement de Cartographer et Gazebo..."
    ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
}

# Fonction pour lancer Teleop
launch_teleop() {
    echo "Lancement de Teleop Keyboard..."
    ros2 run turtlebot3_teleop teleop_keyboard
}

# Fonction pour lancer Navigation2
launch_navigation() {
    echo "Lancement de Navigation2..."
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map_test2.yaml
}

# Fonction pour lancer RViz
launch_rviz() {
    echo "Lancement de RViz..."
    ros2 launch turtlebot3_bringup rviz2.launch.py
}

# Fonction pour lancer Gazebo avec la maison
launch_gazebo_house() {
    echo "Lancement de Gazebo avec la maison..."
    ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
}

# Fonction pour lancer le contrôle du robot (turtlebot3_drive)
launch_drive() {
    echo "Lancement du contrôle du robot (turtlebot3_drive)..."
    ros2 run turtlebot3_gazebo turtlebot3_drive
}

# Fonction pour sauvegarder la carte dans un dossier spécifique et avec un nom personnalisé
save_map() {
    # Vérifier si le dossier est spécifié
    if [ -z "$1" ]; then
        echo "Erreur : Aucun dossier spécifié pour enregistrer la carte."
        echo "Utilisation : save_map <dossier> <nom_fichier>"
        exit 1
    fi
    
    # Vérifier si le nom du fichier est spécifié
    if [ -z "$2" ]; then
        echo "Erreur : Aucun nom de fichier spécifié pour enregistrer la carte."
        echo "Utilisation : save_map <dossier> <nom_fichier>"
        exit 1
    fi
    
    # Sauvegarder la carte dans le dossier et avec le nom de fichier spécifié
    echo "Sauvegarde de la carte dans le dossier : $1 avec le nom de fichier : $2"
    ros2 run nav2_map_server map_saver_cli -f "$1/$2"
}

# Vérifier si un paramètre a été passé
if [ $# -lt 1 ]; then
    echo "Erreur : Aucun paramètre spécifié. Utilisation : $0 <action>"
    echo "Les actions possibles sont :"
    echo "  cartographer      - Lancer Cartographer (cartographer.launch.py)"
    echo "  gazebo            - Lancer Gazebo (turtlebot3_world.launch.py)"
    echo "  cartographer_gazebo - Lancer Cartographer et Gazebo (cartographer.launch.py + turtlebot3_world.launch.py)"
    echo "  teleop            - Lancer Teleop (teleop_keyboard)"
    echo "  navigation        - Lancer Navigation2 (navigation2.launch.py)"
    echo "  rviz              - Lancer RViz (rviz2.launch.py)"
    echo "  gazebo_house      - Lancer Gazebo avec la maison (turtlebot3_house.launch.py)"
    echo "  drive             - Lancer le contrôle du robot (turtlebot3_drive)"
    echo "  save_map          - Sauvegarder la carte dans un dossier spécifique avec un nom personnalisé"
    exit 1
fi

action=$1
folder=$2  # Le dossier pour sauvegarder la carte
file_name=$3  # Le nom du fichier pour la carte

# Exécuter l'action en fonction du paramètre
case $action in
    "cartographer")
        launch_cartographer
        ;;
    "gazebo")
        launch_gazebo
        ;;
    "cartographer_gazebo")
        launch_cartographer_and_gazebo
        ;;
    "teleop")
        launch_teleop
        ;;
    "navigation")
        launch_navigation
        ;;
    "rviz")
        launch_rviz
        ;;
    "gazebo_house")
        launch_gazebo_house
        ;;
    "drive")
        launch_drive
        ;;
    "save_map")
        save_map "$folder" "$file_name"  # Appel de la fonction save_map avec le dossier et le nom de fichier
        ;;
    *)
        echo "Paramètre non reconnu. Les valeurs possibles sont : cartographer, gazebo, cartographer_gazebo, teleop, navigation, rviz, gazebo_house, drive, save_map."
        exit 1
        ;;
esac

