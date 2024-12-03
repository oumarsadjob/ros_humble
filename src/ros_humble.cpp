#include <iostream>
#include <cstdlib>
#include <string>

void launch_cartographer() {
    std::cout << "Lancement de Cartographer..." << std::endl;
    system("ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True");
}

void launch_teleop() {
    std::cout << "Lancement de Teleop Keyboard..." << std::endl;
    system("ros2 run turtlebot3_teleop teleop_keyboard");
}

void launch_navigation() {
    std::cout << "Lancement de Navigation2..." << std::endl;
    system("ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map_test2.yaml");
}

void launch_cartographer_and_gazebo() {
    std::cout << "Lancement de Cartographer et Gazebo..." << std::endl;
    system("ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True");
    system("ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py");
}

void launch_rviz() {
    std::cout << "Lancement de RViz..." << std::endl;
    system("ros2 launch turtlebot3_bringup rviz2.launch.py");
}

void launch_gazebo_house() {
    std::cout << "Lancement de Gazebo avec la maison..." << std::endl;
    system("ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py");
}

void launch_drive() {
    std::cout << "Lancement du contrôle du robot (turtlebot3_drive)..." << std::endl;
    system("ros2 run turtlebot3_gazebo turtlebot3_drive");
}

int main(int argc, char* argv[]) {
    // Vérifier si un paramètre a été passé
    if (argc != 2) {
        std::cout << "Erreur : Aucun paramètre spécifié. Utilisation : ./turtlebot3_launcher <action>" << std::endl;
        std::cout << "Les actions possibles sont :" << std::endl;
        std::cout << "  cartographer      - Lancer Cartographer (cartographer.launch.py)" << std::endl;
        std::cout << "  teleop            - Lancer Teleop (teleop_keyboard)" << std::endl;
        std::cout << "  navigation        - Lancer Navigation2 (navigation2.launch.py)" << std::endl;
        std::cout << "  cartographer_gazebo - Lancer Cartographer et Gazebo (cartographer.launch.py + turtlebot3_world.launch.py)" << std::endl;
        std::cout << "  rviz              - Lancer RViz (rviz2.launch.py)" << std::endl;
        std::cout << "  gazebo_house      - Lancer Gazebo avec la maison (turtlebot3_house.launch.py)" << std::endl;
        std::cout << "  drive             - Lancer le contrôle du robot (turtlebot3_drive)" << std::endl;
        return 1;
    }

    std::string action = argv[1];

    // Exécuter l'action en fonction du paramètre
    if (action == "cartographer") {
        launch_cartographer();
    } else if (action == "teleop") {
        launch_teleop();
    } else if (action == "navigation") {
        launch_navigation();
    } else if (action == "cartographer_gazebo") {
        launch_cartographer_and_gazebo();
    } else if (action == "rviz") {
        launch_rviz();
    } else if (action == "gazebo_house") {
        launch_gazebo_house();
    } else if (action == "drive") {
        launch_drive();
    } else {
        std::cout << "Paramètre non reconnu. Les valeurs possibles sont : cartographer, teleop, navigation, cartographer_gazebo, rviz, gazebo_house, drive." << std::endl;
        return 1;
    }

    return 0;
}
