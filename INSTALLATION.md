# Réinstaller le PC de développement

Remettre la simulation en marche sur un PC neuf.

Ce qui n'est pas sur GitHub est perdu avec le PC, dont les cartes créées depuis la page
web. Les envoyer régulièrement : `git add maps && git commit && git push`.

## 1. Ubuntu 24.04 LTS

Télécharger « Ubuntu 24.04 LTS Desktop » : https://ubuntu.com/download/desktop
Créer une clé USB d'installation (sous Windows : Rufus, https://rufus.ie), démarrer
dessus, installer.

## 2. ROS 2 Jazzy

```
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade -y
sudo apt install ros-jazzy-desktop ros-dev-tools
```

## 3. Gazebo Harmonic, Nav2, simulation TurtleBot3, clavier

```
sudo apt install ros-jazzy-ros-gz ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-minimal-tb3-sim ros-jazzy-key-teleop
```

## 4. Le projet

```
sudo apt install git
git clone https://github.com/SergeKenS/argos-robot.git ~/Documents/Projet/argos_ws
```

## 5. Dépendances et compilation

```
source /opt/ros/jazzy/setup.bash
sudo rosdep init
rosdep update
cd ~/Documents/Projet/argos_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## 6. Vérifier

```
ros2 launch argos_mission argos.launch.py rviz:=true
```
Gazebo et RViz s'ouvrent, la page web répond sur http://localhost:8080.

## Sources

- [ROS 2 Jazzy, installation Ubuntu](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Nav2, démarrage rapide](https://docs.nav2.org/jazzy/getting_started/quickstart/quickstart/)
- [Gazebo avec ROS 2](https://gazebosim.org/docs/latest/ros_installation/)
