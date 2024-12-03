
# **TurtleBot3 ROS 2 Automation Script**

Ce projet contient un script Bash permettant de gérer différentes fonctionnalités pour le robot **TurtleBot3** sous **ROS 2 Humble**, comme la cartographie, la navigation, la simulation avec Gazebo, et plus encore.

## **Prérequis**

Avant de lancer le script, assurez-vous que les éléments suivants sont correctement installés et configurés :

### 1. **Système et environnement**
- **ROS 2 Humble** : Suivez [la documentation officielle](https://docs.ros.org/en/humble/index.html) pour l'installation.
- Ajoutez cette ligne à votre fichier `~/.bashrc` :
  ```bash
  export TURTLEBOT3_MODEL=burger  # Ou waffle, selon votre modèle
  source ~/.bashrc
  ```

### 2. **Packages nécessaires**
Installez les outils ROS 2 requis :
```bash
sudo apt update
sudo apt install -y \
  ros-humble-turtlebot3* \
  ros-humble-turtlebot3-simulations \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 \
  ros-humble-cartographer-ros \
  python3-colcon-common-extensions
```

### 3. **Permissions d'exécution**
Assurez-vous que le script est exécutable :
```bash
chmod +x script_name.sh
```

---

## **Usage**

Lancez le script en spécifiant une action. Par exemple :
```bash
./script_name.sh <action>
```

### **Actions disponibles**
| Action               | Description                                                                 |
|----------------------|-----------------------------------------------------------------------------|
| `cartographer`       | Lancer la cartographie avec Cartographer.                                  |
| `gazebo`             | Lancer Gazebo avec le monde par défaut.                                    |
| `cartographer_gazebo`| Lancer simultanément Cartographer et Gazebo.                               |
| `teleop`             | Contrôler le robot avec le clavier.                                        |
| `navigation`         | Lancer Navigation2 avec une carte.                                         |
| `rviz`               | Ouvrir RViz pour la visualisation.                                         |
| `gazebo_house`       | Lancer Gazebo avec le monde "maison".                                      |
| `drive`              | Contrôler le robot dans Gazebo avec un script prédéfini.                  |
| `save_map <dossier> <nom>` | Sauvegarder une carte dans un dossier avec un nom de fichier spécifique. |

---

## **Exemples**

1. Lancer la cartographie avec simulation dans Gazebo :
   ```bash
   ./script_name.sh cartographer_gazebo
   ```

2. Contrôler le robot via le clavier :
   ```bash
   ./script_name.sh teleop
   ```

3. Sauvegarder une carte après la génération :
   ```bash
   ./script_name.sh save_map ~/maps my_map
   ```

---

## **Structure du script**

Le script est structuré autour de plusieurs fonctions :
- **Cartographer** : Pour la cartographie SLAM.
- **Gazebo** : Simulation dans différents mondes.
- **Navigation2** : Navigation autonome avec une carte existante.
- **RViz** : Visualisation des données du robot et du SLAM.
- **Téléopération** : Contrôle manuel du robot.
- **Sauvegarde de carte** : Génération de cartes utilisables pour la navigation.

---

## **Contributions**
Les contributions sont les bienvenues ! Si vous souhaitez améliorer ce script ou ajouter des fonctionnalités, veuillez créer une issue ou une pull request.

---

## **Licence**
Ce projet est sous licence MIT. Consultez le fichier `LICENSE` pour plus de détails.

---

## **Ressources supplémentaires**
- [Documentation officielle de TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [Documentation ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
