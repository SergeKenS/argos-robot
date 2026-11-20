// panneaux/cartographie.js : rôle Cartographie (dessiner une nouvelle carte).
// Commandes envoyées au répartiteur : cartographier, enregistrer <nom>, abandonner.
// Pendant la cartographie, on promène le robot : les vitesses partent sur
// /cmd_vel_nav, là où Nav2 publie les siennes, pour passer par le lisseur de
// vitesse puis par collision_monitor (arrêt devant un obstacle vu par le LiDAR).

import { Carte } from "../outils/carte.js";
import { Croix } from "../outils/croix.js";

const FORMAT_NOM = /^[a-z0-9_]+$/;
// Vitesse = maximum du lisseur de vitesse de Nav2 (nav2_params.yaml), pour couvrir
// vite une grande surface. Rotation modérée : SLAM dessine plus net en tournant lentement.
const VITESSE_AVANCE = 0.5;    // m/s
const VITESSE_ROTATION = 0.8;  // rad/s
const PERIODE_ENVOI = 100;     // ms : on répète l'ordre tant qu'une touche est tenue

let robot, croix, topicVitesse = null, minuterie = null;
const $ = (id) => document.getElementById(id);

function initialiser(r) {
  robot = r;
  new Carte($("carto-canvas"), robot, { message: $("carto-message-carte"), avecPoints: false });

  croix = new Croix($("carto-croix"),
    { KeyW: "haut", KeyS: "bas", KeyA: "gauche", KeyD: "droite",
      ArrowUp: "haut", ArrowDown: "bas", ArrowLeft: "gauche", ArrowRight: "droite" },
    piloter);

  robot.ecouter("etat", majEtat);
  robot.ecouter("connexion", () => {
    topicVitesse = robot.topic("/cmd_vel_nav", "geometry_msgs/msg/Twist");
    topicVitesse.advertise();
  });
  robot.ecouter("deconnexion", () => { arreter(); topicVitesse = null; });

  $("btn-cartographier").addEventListener("click", () => {
    if (confirm("Commencer une nouvelle carte ? Le robot s'arrête ; promenez-le ensuite. "
        + "La base sera placée là où il se trouve maintenant.")) {
      robot.envoyer("cartographier");
    }
  });
  $("nom-carte").addEventListener("input", (e) => {
    e.target.value = e.target.value.toLowerCase().replace(/[\s-]/g, "_");
    // Un nom déjà pris écraserait une carte existante.
    const nom = e.target.value;
    $("btn-enregistrer").disabled = !FORMAT_NOM.test(nom) || robot.cartes.includes(nom);
  });
  $("btn-enregistrer").addEventListener("click", () => {
    robot.envoyer(`enregistrer ${$("nom-carte").value}`);
    $("nom-carte").value = "";
    $("btn-enregistrer").disabled = true;
  });
  $("btn-abandonner").addEventListener("click", () => {
    if (confirm("Abandonner la nouvelle carte ? Elle sera perdue.")) robot.envoyer("abandonner");
  });

  // Clavier : seulement si ce panneau est visible et qu'on ne tape pas un nom.
  document.addEventListener("keydown", (e) => {
    if (e.repeat || !clavierPermis()) return;
    if (croix.touche(e.code, true)) e.preventDefault();
  });
  document.addEventListener("keyup", (e) => { croix.touche(e.code, false); });
  window.addEventListener("blur", () => croix.toutRelacher());   // fenêtre quittée : on arrête
  majEtat();
}

function enCartographie() { return robot.etat.mode === "CARTOGRAPHIE"; }

function clavierPermis() {
  const saisie = ["INPUT", "SELECT", "TEXTAREA"].includes(document.activeElement.tagName);
  return robot.visible === "cartographie" && enCartographie() && !saisie;
}

function majEtat() {
  const actif = enCartographie();
  $("carto-avant").hidden = actif;
  $("carto-pendant").hidden = !actif;
  croix.actif = actif;
  if (!actif) croix.toutRelacher();
}

// Direction de la croix : y = avancer (+) ou reculer (-), x = droite (+) ou gauche (-).
function piloter({ x, y }) {
  clearInterval(minuterie);
  minuterie = null;
  const avance = y * VITESSE_AVANCE, rotation = -x * VITESSE_ROTATION;
  publier(avance, rotation);
  // Tant qu'on tient, on répète. Si la page se fige ou perd le réseau, les envois
  // cessent et le lisseur de vitesse de Nav2 arrête le robot seul au bout de 1 s.
  if (avance !== 0 || rotation !== 0) minuterie = setInterval(() => publier(avance, rotation), PERIODE_ENVOI);
}

function publier(avance, rotation) {
  if (!topicVitesse) return;
  topicVitesse.publish({ linear: { x: avance, y: 0, z: 0 }, angular: { x: 0, y: 0, z: rotation } });
}

function arreter() {
  croix.toutRelacher();
  clearInterval(minuterie);
  minuterie = null;
}

export default {
  role: "cartographie",
  titre: "Cartographie",
  modes: ["CARTOGRAPHIE"],
  initialiser,
  cacher: arreter,
};
