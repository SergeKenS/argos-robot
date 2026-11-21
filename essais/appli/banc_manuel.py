"""Banc d'essai PROVISOIRE du mode Manuel. Référence pour écrire le vrai rôle Manuel,
le nœud tourelle et le nœud caméra (voir README.md de ce dossier), puis à supprimer.

Joue à la place du futur rôle Manuel, du nœud tourelle et du nœud caméra, pour
essayer la page avant de les écrire. Remplace le répartiteur pendant l'essai.
  - /argos/commande : manuel, veille, recentrer, aligner
  - /argos/etat      : VEILLE ou MANUEL
  - /argos/manuel/corps (Twist)   -> cmd_vel (pont_moteurs), seulement en MANUEL
  - /argos/manuel/tete  (Vector3) -> vitesses de la tête (°/s), intégrées en angles
  - /argos/tete/angles  (Vector3) <- angles annoncés par l'ESP32, par rapport au centre
  - image en direct : http://<robot>:8081/flux
"""
import json
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

sys.path.insert(0, '/home/serge/Documents/Projet/argos_ws/essais/tourelle')
from flux_video import FluxVideo  # noqa: E402
from suivi_fluide import CENTRE_BAS, CENTRE_HAUT, Camera, Tourelle, limiter  # noqa: E402

AVANCE_MAX, ROTATION_MAX = 0.70, 2.0   # m/s, rad/s (la rampe est dans pont_moteurs)
CHIEN_DE_GARDE = 0.3                   # s sans ordre : arrêt
ROTATION_ALIGNER = 1.0                 # rad/s au plus pendant « aligner »


class Banc(Node):
    def __init__(self):
        super().__init__('banc_manuel')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_etat = self.create_publisher(String, '/argos/etat', qos)
        self.pub_journal = self.create_publisher(String, '/argos/journal', 10)
        self.pub_vitesse = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_angles = self.create_publisher(Vector3, '/argos/tete/angles', 10)
        self.create_subscription(String, '/argos/commande', self.commande, 10)
        self.create_subscription(Twist, '/argos/manuel/corps', self.corps, 10)
        self.create_subscription(Vector3, '/argos/manuel/tete', self.tete, 10)
        self.create_subscription(Odometry, 'odom', self.odom, 10)

        self.mode = 'VEILLE'
        self.ordre_corps, self.heure_corps = Twist(), 0.0
        self.vitesse_tete, self.heure_tete = (0.0, 0.0), 0.0
        self.cible = [float(CENTRE_BAS), float(CENTRE_HAUT)]
        self.lacet = None            # lacet du corps (odométrie), radians
        self.alignement = None       # (lacet de départ, angle à tourner en radians)
        self.arrete = True

        self.tourelle = Tourelle()
        self.tourelle.viser(*self.cible)
        self.camera = Camera(0)
        self.flux = FluxVideo(8081)
        threading.Thread(target=self.diffuser, daemon=True).start()

        self.derniere = time.perf_counter()
        self.create_timer(0.02, self.boucle)        # 50 Hz
        self.create_timer(0.05, self.publier_angles)
        self.publier_etat()
        self.dire('Banc Manuel prêt (essai provisoire).')

    # ----- Réception -----
    def commande(self, msg):
        mot = msg.data.strip().split()[0] if msg.data.strip() else ''
        if mot == 'manuel':
            self.mode = 'MANUEL'
        elif mot == 'veille':
            self.mode = 'VEILLE'
            self.alignement = None
            self.stop()
        elif mot == 'recentrer' and self.mode == 'MANUEL':
            self.cible = [float(CENTRE_BAS), float(CENTRE_HAUT)]
            self.tourelle.viser(*self.cible)   # oubli corrigé : l'ordre partait seulement au mouvement suivant
            self.dire('Tête recentrée.')
        elif mot == 'aligner' and self.mode == 'MANUEL' and self.lacet is not None:
            angle = math.radians(self.cible[0] - CENTRE_BAS)
            self.alignement = (self.lacet, angle, self.cible[0] - CENTRE_BAS)
            self.dire(f'Alignement du corps : {math.degrees(angle):+.0f}°.')
        else:
            self.dire(f'Commande ignorée par le banc : « {msg.data} »', 'avertissement')
        self.publier_etat()

    def corps(self, msg):
        # Relais immédiat, message par message : quand la page se tait (touche lâchée),
        # cmd_vel se tait aussi et pont_moteurs passe en roue libre.
        if self.mode != 'MANUEL' or self.alignement is not None:
            return
        t = Twist()
        t.linear.x = max(-AVANCE_MAX, min(AVANCE_MAX, msg.linear.x))
        t.angular.z = max(-ROTATION_MAX, min(ROTATION_MAX, msg.angular.z))
        self.pub_vitesse.publish(t)

    def tete(self, msg):
        self.vitesse_tete, self.heure_tete = (msg.x, msg.y), time.perf_counter()

    def odom(self, msg):
        q = msg.pose.pose.orientation
        self.lacet = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    # ----- Boucle à 50 Hz -----
    def boucle(self):
        maintenant = time.perf_counter()
        dt, self.derniere = maintenant - self.derniere, maintenant
        if self.mode != 'MANUEL':
            return
        if self.alignement is not None:
            self.aligner()
            return
        # Tête : vitesse demandée intégrée en angle, dans le débattement.
        if maintenant - self.heure_tete < CHIEN_DE_GARDE and any(self.vitesse_tete):
            self.cible[0] = limiter(self.cible[0] + self.vitesse_tete[0] * dt, CENTRE_BAS)
            self.cible[1] = limiter(self.cible[1] + self.vitesse_tete[1] * dt, CENTRE_HAUT)
            self.tourelle.viser(*self.cible)

    def aligner(self):
        """Le corps tourne vers le regard ; la tête tourne en sens inverse (regard fixe)."""
        depart, angle, tete_depart = self.alignement
        tourne = math.atan2(math.sin(self.lacet - depart), math.cos(self.lacet - depart))
        reste = angle - tourne
        self.cible[0] = limiter(CENTRE_BAS + math.degrees(reste), CENTRE_BAS)
        self.tourelle.viser(*self.cible)
        if abs(reste) < math.radians(1.5):
            self.alignement = None
            self.cible[0] = float(CENTRE_BAS)
            self.tourelle.viser(*self.cible)
            self.stop()
            self.dire('Corps aligné sur la tête.')
            return
        t = Twist()
        t.angular.z = max(-ROTATION_ALIGNER, min(ROTATION_ALIGNER, 2.5 * reste))
        self.pub_vitesse.publish(t)
        self.arrete = False

    def stop(self):
        self.pub_vitesse.publish(Twist())
        self.arrete = True

    # ----- Publication -----
    def publier_angles(self):
        bas, haut = self.tourelle.position(time.perf_counter())
        self.pub_angles.publish(Vector3(x=float(bas - CENTRE_BAS), y=float(haut - CENTRE_HAUT), z=0.0))

    def publier_etat(self):
        self.pub_etat.publish(String(data=self.mode))

    def dire(self, texte, niveau='info'):
        self.get_logger().info(texte)
        self.pub_journal.publish(String(data=json.dumps({'niveau': niveau, 'texte': texte})))

    def diffuser(self):
        numero = 0
        while rclpy.ok():
            numero, _, image = self.camera.suivante(numero)
            if image is not None:
                self.flux.publier(image)

    def fermer(self):
        self.stop()
        self.flux.fermer()
        self.camera.fermer()
        self.tourelle.fermer()


def main():
    rclpy.init()
    banc = Banc()
    try:
        rclpy.spin(banc)
    except KeyboardInterrupt:
        pass
    finally:
        banc.fermer()
        banc.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
