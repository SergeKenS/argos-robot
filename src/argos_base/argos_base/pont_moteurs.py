"""Nœud « pont_moteurs » : relie ROS 2 à la carte moteurs Yahboom du robot réel.

  reçoit  cmd_vel (geometry_msgs/Twist)   → consigne de vitesse des deux roues
  publie  odom (nav_msgs/Odometry)        ← position calculée avec les encodeurs
          transformation odom → base_link (pour Nav2 et RViz)

Conduite, comme une voiture (la carte garde la dernière vitesse reçue, sans
limite de temps : c'est ce nœud qui décide) :
  - ordre reçu : la vitesse rejoint l'ordre par une rampe. Elle monte d'au plus
    « acceleration » par seconde ; sans rampe, un ordre brutal fait dépasser la
    consigne au régulateur de la carte (0,35 m/s pour 0,28).
  - ordre de sens opposé, ou ordre 0 (STOP, Nav2 qui s'arrête) : le robot FREINE,
    d'au plus « freinage » par seconde, puis repart dans l'autre sens si l'ordre dure.
  - plus aucun ordre depuis « delai_chien_de_garde » s (touche lâchée, page fermée,
    réseau perdu) : ROUE LIBRE. Moteurs sans courant ($pwm:0), seuls le poids et
    les frottements arrêtent le robot.
  - vitesse des roues limitée à « vitesse_max »
  - arrêt des moteurs quand le nœud s'arrête (Ctrl+C, ros2 launch arrêté)

Usage :
  ros2 run argos_base pont_moteurs --ros-args --params-file <config/base.yaml>
Paramètres et valeurs mesurées : config/base.yaml ; mesures : essais/moteurs/README.md.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from argos_base.carte_yahboom import CarteYahboom, trouver_port
from argos_base.cinematique import approcher, limiter, Odometrie, vitesses_roues


class PontMoteurs(Node):
    """Traduit cmd_vel pour la carte, et ses compteurs en odométrie."""

    def __init__(self):
        super().__init__('pont_moteurs')
        p = self.declare_parameter
        port = p('port', '').value                        # vide : recherche automatique
        self.entraxe = p('entraxe', 0.0).value             # m, milieu à milieu des roues
        diametre = p('diametre_roue', 0.067).value         # m
        impulsions_par_tour = p('impulsions_par_tour', 1760).value
        self.sortie_gauche = p('sortie_gauche', 4).value   # Motor 1 à 4 de la carte
        self.sortie_droite = p('sortie_droite', 3).value
        self.sens_gauche = p('sens_gauche', -1).value      # -1 : moteur monté en miroir
        self.sens_droite = p('sens_droite', 1).value
        self.vitesse_max = p('vitesse_max', 0.3).value     # m/s par roue
        self.acceleration = p('acceleration', 0.4).value   # m/s²
        self.freinage = p('freinage', 1.0).value            # m/s²
        self.acceleration_rotation = p('acceleration_rotation', 3.0).value   # rad/s²
        self.freinage_rotation = p('freinage_rotation', 6.0).value           # rad/s²
        self.delai_chien = p('delai_chien_de_garde', 0.5).value  # s
        frequence = p('frequence', 20.0).value             # Hz
        self.periode = 1.0 / frequence
        self.publier_tf = p('publier_tf', True).value
        self.repere_odom = p('repere_odom', 'odom').value
        self.repere_robot = p('repere_robot', 'base_link').value

        if self.entraxe <= 0:
            raise ValueError("Paramètre « entraxe » manquant : le mesurer (m, milieu à milieu "
                             "des roues) et le mettre dans config/base.yaml")
        port = port or trouver_port()
        if not port:
            raise RuntimeError('Carte moteurs Yahboom introuvable (USB 1a86:7522)')

        self.carte = CarteYahboom(port)
        self.odometrie = Odometrie(math.pi * diametre / impulsions_par_tour, self.entraxe)
        self.voulu = (0.0, 0.0)          # vitesse demandée (avance m/s, rotation rad/s)
        self.applique = (0.0, 0.0)       # vitesse envoyée, qui rejoint « voulu » par la rampe
        self.heure_consigne = None       # heure du dernier cmd_vel reçu
        self.arrete = False              # arrêt rapide déjà envoyé

        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.diffuseur_tf = TransformBroadcaster(self) if self.publier_tf else None
        self.create_subscription(Twist, 'cmd_vel', self.recevoir_cmd_vel, 10)
        self.create_timer(self.periode, self.boucle)
        self.get_logger().info(f'Carte moteurs sur {port}, entraxe {self.entraxe} m, '
                               f'vitesse max {self.vitesse_max} m/s, '
                               f'accélération {self.acceleration} m/s²')

    def recevoir_cmd_vel(self, msg):
        self.voulu = (msg.linear.x, msg.angular.z)
        self.heure_consigne = self.get_clock().now()

    def boucle(self):
        """Appelée « frequence » fois par seconde : ordre aux moteurs, puis odométrie."""
        maintenant = self.get_clock().now()
        perime = (self.heure_consigne is None or
                  (maintenant - self.heure_consigne).nanoseconds / 1e9 > self.delai_chien)
        if not perime:
            # Rampe : la vitesse envoyée rejoint la vitesse voulue sans à-coup.
            avance = approcher(self.applique[0], self.voulu[0],
                               self.acceleration, self.freinage, self.periode)
            rotation = approcher(self.applique[1], self.voulu[1],
                                 self.acceleration_rotation, self.freinage_rotation, self.periode)
            self.applique = (avance, rotation)
        if perime or self.applique == (0.0, 0.0):
            # Roue libre (ou arrêt atteint) : moteurs sans courant, envoyé une seule fois
            if not self.arrete:
                self.carte.arret_rapide()
                self.arrete = True
            # En roue libre, la vitesse réelle est celle des encodeurs : un nouvel
            # ordre repartira de là, sans à-coup.
            if perime:
                self.applique = (self.odometrie.lineaire, self.odometrie.angulaire)
        else:
            gauche, droite = limiter(*vitesses_roues(*self.applique, self.entraxe),
                                     self.vitesse_max)
            vitesses = [0, 0, 0, 0]
            vitesses[self.sortie_gauche - 1] = gauche * 1000 * self.sens_gauche
            vitesses[self.sortie_droite - 1] = droite * 1000 * self.sens_droite
            self.carte.vitesses(vitesses)
            self.arrete = False
        self.publier_odometrie()

    def publier_odometrie(self):
        heure, compteurs = self.carte.derniers_compteurs()
        if compteurs is None:
            return
        self.odometrie.mettre_a_jour(heure,
                                     compteurs[self.sortie_gauche - 1] * self.sens_gauche,
                                     compteurs[self.sortie_droite - 1] * self.sens_droite)
        o = self.odometrie
        stamp = self.get_clock().now().to_msg()
        # Orientation : rotation autour de l'axe vertical seulement (quaternion z, w)
        qz, qw = math.sin(o.theta / 2), math.cos(o.theta / 2)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.repere_odom
        msg.child_frame_id = self.repere_robot
        msg.pose.pose.position.x = o.x
        msg.pose.pose.position.y = o.y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = o.lineaire
        msg.twist.twist.angular.z = o.angulaire
        # Incertitudes (diagonale) : valeurs prudentes, à affiner avec l'essai au sol
        for i, valeur in zip((0, 7, 14, 21, 28, 35), (0.01, 0.01, 1e6, 1e6, 1e6, 0.05)):
            msg.pose.covariance[i] = valeur
            msg.twist.covariance[i] = valeur
        self.pub_odom.publish(msg)

        if self.diffuseur_tf is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.repere_odom
            tf.child_frame_id = self.repere_robot
            tf.transform.translation.x = o.x
            tf.transform.translation.y = o.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.diffuseur_tf.sendTransform(tf)

    def fermer(self):
        self.carte.fermer()


def main():
    rclpy.init()
    noeud = None
    try:
        noeud = PontMoteurs()
        rclpy.spin(noeud)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Quoi qu'il arrive, les moteurs s'arrêtent avant que le nœud quitte
        if noeud is not None:
            noeud.fermer()
            noeud.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
