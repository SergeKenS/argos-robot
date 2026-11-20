// panneaux/manuel.js : rôle Manuel (conduire le corps et la tête, à vue).
// PC : croix de boutons et clavier (W A S D = corps, I J K L = tête, R = recentrer,
//      C = aligner le corps sur la tête, Espace = STOP, Maj = vitesse maximale).
// Écran tactile : deux joysticks.
// Les ordres ne partent que si le robot est en mode MANUEL et ce panneau visible.

import { Croix } from "../outils/croix.js";
import { Joystick } from "../outils/joystick.js";

// ----- Liaison avec le robot --------------------------------------------------
// Noms PROVISOIRES : le rôle Manuel, le nœud tourelle et le nœud caméra n'existent
// pas encore côté robot. Ils seront confirmés en les écrivant.
const COMMANDE_ENTRER = "manuel";              // commande au répartiteur
const COMMANDE_RECENTRER = "recentrer";        // la tête revient droit devant
const COMMANDE_ALIGNER = "aligner";            // le corps pivote vers le regard de la tête
const TOPIC_CORPS = "/argos/manuel/corps";     // geometry_msgs/Twist : avance m/s, rotation rad/s
const TOPIC_TETE = "/argos/manuel/tete";       // geometry_msgs/Vector3 : x = °/s vers la gauche, y = °/s vers le haut
const TOPIC_ANGLES = "/argos/tete/angles";     // geometry_msgs/Vector3 : x = ° à gauche du corps, y = ° vers le haut
const PORT_VIDEO = 8081;                       // flux MJPEG (même serveur que essais/tourelle/flux_video.py)

// ----- Vitesses ---------------------------------------------------------------
const AVANCE = 0.20, AVANCE_MAX = 0.30;        // m/s (0,3 = plafond des roues dans base.yaml)
const ROTATION = 0.8, ROTATION_MAX = 1.5;      // rad/s
const TETE = 60;                               // °/s
const PERIODE_ENVOI = 100;                     // ms : ordres répétés tant qu'on tient
// Débattement de la tourelle autour de son centre (AMPLITUDE dans suivi_fluide.ino).
const LACET_MAX = 45, TANGAGE_MAX = 45;        // degrés

let robot, croixCorps, croixTete, joyCorps, joyTete;
let corps = { x: 0, y: 0 }, tete = { x: 0, y: 0 };
let rapide = false, minuterie = null;
let topicCorps = null, topicTete = null;
let surveillanceVideo = null, relanceVideo = null;
const $ = (id) => document.getElementById(id);
const virgule = (v, n) => v.toFixed(n).replace(".", ",");

function initialiser(r) {
  robot = r;
  construireGraduations();

  const touchesCorps = { KeyW: "haut", KeyS: "bas", KeyA: "gauche", KeyD: "droite" };
  const touchesTete = { KeyI: "haut", KeyK: "bas", KeyJ: "gauche", KeyL: "droite" };
  croixCorps = new Croix($("croix-corps"), touchesCorps, (d) => { corps = d; ordres(); });
  croixTete = new Croix($("croix-tete"), touchesTete, (d) => { tete = d; ordres(); });
  joyCorps = new Joystick($("joystick-corps"), (d) => { corps = d; ordres(); });
  joyTete = new Joystick($("joystick-tete"), (d) => { tete = d; ordres(); });
  nommerTouches(touchesCorps, $("croix-corps"));
  nommerTouches(touchesTete, $("croix-tete"));

  $("btn-prendre").addEventListener("click", () => robot.envoyer(COMMANDE_ENTRER));
  $("btn-stop").addEventListener("click", stop);
  $("btn-recentrer").addEventListener("click", () => robot.envoyer(COMMANDE_RECENTRER));
  $("btn-aligner").addEventListener("click", () => robot.envoyer(COMMANDE_ALIGNER));
  $("btn-menu").addEventListener("click", () => document.body.classList.add("menu-ouvert"));

  robot.ecouter("connexion", () => {
    topicCorps = robot.topic(TOPIC_CORPS, "geometry_msgs/msg/Twist");
    topicCorps.advertise();
    topicTete = robot.topic(TOPIC_TETE, "geometry_msgs/msg/Vector3");
    topicTete.advertise();
    robot.topic(TOPIC_ANGLES, "geometry_msgs/msg/Vector3", { throttle_rate: 50 }).subscribe(afficherTete);
    majVoile();
  });
  robot.ecouter("deconnexion", () => { stop(); topicCorps = topicTete = null; majVoile(); });
  robot.ecouter("etat", () => { if (!commandesPermises()) stop(); majVoile(); });
  robot.ecouter("vitesse", afficherVitesse);

  // ----- Clavier -----
  document.addEventListener("keydown", (e) => {
    if (!clavierPermis()) return;
    if (e.code === "ShiftLeft" || e.code === "ShiftRight") { rapide = true; ordres(); return; }
    if (e.repeat) { if (croixCorps.touches[e.code] || croixTete.touches[e.code] || e.code === "Space") e.preventDefault(); return; }
    if (e.code === "Space") { e.preventDefault(); stop(); return; }
    if (e.code === "KeyR") { robot.envoyer(COMMANDE_RECENTRER); return; }
    if (e.code === "KeyC") { robot.envoyer(COMMANDE_ALIGNER); return; }
    if (croixCorps.touche(e.code, true) || croixTete.touche(e.code, true)) e.preventDefault();
  });
  document.addEventListener("keyup", (e) => {
    if (e.code === "ShiftLeft" || e.code === "ShiftRight") { rapide = false; ordres(); return; }
    croixCorps.touche(e.code, false);
    croixTete.touche(e.code, false);
  });
  window.addEventListener("blur", stop);   // fenêtre quittée : tout s'arrête

  majVoile();
}

// ===== Permissions ===============================================================
function commandesPermises() {
  return robot.connecte && robot.etat.mode === "MANUEL" && robot.visible === "manuel";
}
function clavierPermis() {
  const saisie = ["INPUT", "SELECT", "TEXTAREA"].includes(document.activeElement.tagName);
  return commandesPermises() && !saisie;
}

// Voile posé sur l'image tant que le robot n'est pas en mode Manuel.
function majVoile() {
  const enManuel = robot.connecte && robot.etat.mode === "MANUEL";
  $("voile-manuel").hidden = enManuel;
  $("btn-prendre").hidden = !robot.connecte;
  for (const id of ["btn-recentrer", "btn-aligner"]) $(id).disabled = !enManuel;
  for (const c of [croixCorps, croixTete, joyCorps, joyTete]) c.actif = enManuel;
  if (enManuel) return;
  const mode = robot.etat.mode;
  if (!robot.connecte) {
    $("voile-titre").textContent = "Robot hors ligne";
    $("voile-texte").textContent = "La page se reconnecte toute seule dès que le robot répond.";
  } else if (!mode || mode === "VEILLE") {
    $("voile-titre").textContent = "Prendre les commandes";
    $("voile-texte").textContent = "Le robot est au repos. Il obéira ensuite à vos touches.";
  } else {
    $("voile-titre").textContent = "Prendre les commandes";
    $("voile-texte").textContent = `Le robot est en ${mode}. Prendre les commandes arrête cette tâche.`;
  }
}

// ===== Envoi des ordres ==========================================================
// Appelée à chaque changement : envoie tout de suite, puis répète tant qu'on tient.
// Côté robot, un chien de garde arrête tout si les ordres cessent (page fermée, réseau perdu).
function ordres() {
  envoyer();
  const actif = corps.x || corps.y || tete.x || tete.y;
  if (actif && !minuterie) minuterie = setInterval(envoyer, PERIODE_ENVOI);
  if (!actif && minuterie) { clearInterval(minuterie); minuterie = null; }
}

function envoyer() {
  if (!commandesPermises() || !topicCorps) return;
  // Joystick : direction proportionnelle jusqu'au maximum. Clavier : vitesse
  // normale, maximale avec Maj.
  const avance = corps.y * (rapide || corpsAuJoystick() ? AVANCE_MAX : AVANCE);
  const rotation = -corps.x * (rapide || corpsAuJoystick() ? ROTATION_MAX : ROTATION);
  topicCorps.publish({ linear: { x: avance, y: 0, z: 0 }, angular: { x: 0, y: 0, z: rotation } });
  topicTete.publish({ x: -tete.x * TETE, y: tete.y * TETE, z: 0 });
}
function corpsAuJoystick() { return joyCorps.doigt !== null; }

function stop() {
  croixCorps.toutRelacher();
  croixTete.toutRelacher();
  if (joyCorps.doigt !== null) joyCorps.relacher();
  if (joyTete.doigt !== null) joyTete.relacher();
  corps = { x: 0, y: 0 };
  tete = { x: 0, y: 0 };
  ordres();   // envoie les zéros une fois et arrête la répétition
}

// ===== Instruments ===============================================================
function construireGraduations() {
  // 19 traits de -45° à +45° (un tous les 5°), un grand tous les 15°.
  const traits = [];
  for (let i = 0; i <= 18; i++) {
    const trait = document.createElement("span");
    if (i % 3 === 0) trait.className = "grande";
    traits.push(trait);
  }
  $("graduations").replaceChildren(...traits);
}

// Angles de la tête par rapport au corps (degrés) : x = gauche (+), y = haut (+).
function afficherTete({ x: lacetTete, y: tangage }) {
  const borne = (v, max) => Math.max(-max, Math.min(max, v));
  const cote = (v, positif, negatif) => (Math.abs(v) < 0.5 ? "0°" : `${Math.round(Math.abs(v))}° ${v > 0 ? positif : negatif}`);
  // Règle : la gauche est à gauche (angle positif = curseur vers la gauche).
  $("curseur-tete").style.left = `${50 - (borne(lacetTete, LACET_MAX) / LACET_MAX) * 50}%`;
  $("texte-tete").textContent = cote(lacetTete, "G", "D");
  $("repere-inclinaison").style.top = `${50 - (borne(tangage, TANGAGE_MAX) / TANGAGE_MAX) * 50}%`;
  $("texte-inclinaison").textContent = cote(tangage, "H", "B");
  $("tete-lacet").textContent = cote(lacetTete, "gauche", "droite");
  $("tete-tangage").textContent = cote(tangage, "haut", "bas");
  // Vue de dessus : tourner à gauche = sens inverse des aiguilles d'une montre.
  $("fleche-tete").style.transform = `rotate(${-lacetTete}deg)`;
}

function afficherVitesse({ avance, rotation }) {
  const degres = (rotation * 180) / Math.PI;
  $("niveau-vitesse").style.height = `${Math.min(Math.abs(avance) / AVANCE_MAX, 1) * 100}%`;
  $("niveau-vitesse").classList.toggle("recul", avance < -0.005);
  $("jauge-vitesse").textContent = virgule(avance, 2);
  $("vitesse-stop").textContent = `${virgule(avance, 2)} m/s · ${Math.round(degres)} °/s`;
  $("roues-avance").textContent = `${virgule(avance, 2)} m/s`;
  $("roues-rotation").textContent = `${Math.round(degres)} °/s`;
}

// Sur un clavier AZERTY, W A S D sont à la place de Z Q S D : on affiche la vraie
// lettre si le navigateur sait la donner (Chrome, Edge). Sinon, on garde W A S D.
async function nommerTouches(touches, croix) {
  if (!navigator.keyboard || !navigator.keyboard.getLayoutMap) return;
  try {
    const carte = await navigator.keyboard.getLayoutMap();
    for (const [code, sens] of Object.entries(touches)) {
      const lettre = carte.get(code);
      const bouton = croix.querySelector(`[data-sens=${sens}]`);
      if (lettre && bouton) bouton.textContent = lettre.toUpperCase();
    }
  } catch { /* refusé par le navigateur : on garde les lettres d'origine */ }
}

// ===== Image de la caméra ========================================================
// Le flux ne tourne que si le panneau est visible : il coûte du calcul au Pi.
function demarrerVideo() {
  const image = $("video");
  clearTimeout(relanceVideo);
  image.onerror = () => {
    image.hidden = true;
    $("message-video").hidden = false;
    $("message-video").textContent = "IMAGE INDISPONIBLE · NOUVEL ESSAI DANS 3 S";
    $("en-direct").textContent = "SANS IMAGE";
    $("en-direct").classList.add("coupe");
    relanceVideo = setTimeout(demarrerVideo, 3000);
  };
  // Le paramètre t évite qu'un ancien flux en cache soit réutilisé.
  image.src = `http://${location.hostname || "localhost"}:${PORT_VIDEO}/flux?t=${Date.now()}`;
  // Un flux MJPEG ne signale pas chaque image : on regarde si une image est arrivée.
  clearInterval(surveillanceVideo);
  surveillanceVideo = setInterval(() => {
    if (image.naturalWidth > 0 && image.hidden) {
      image.hidden = false;
      $("message-video").hidden = true;
      $("en-direct").textContent = "EN DIRECT · CAMÉRA";
      $("en-direct").classList.remove("coupe");
    }
  }, 500);
}

function arreterVideo() {
  clearTimeout(relanceVideo);
  clearInterval(surveillanceVideo);
  const image = $("video");
  image.onerror = null;
  image.removeAttribute("src");   // ferme la connexion au flux
  image.hidden = true;
  $("message-video").hidden = false;
  $("message-video").textContent = "IMAGE DE LA CAMÉRA";
}

export default {
  role: "manuel",
  titre: "Manuel",
  modes: ["MANUEL"],
  initialiser,
  montrer() { demarrerVideo(); majVoile(); },
  cacher() { stop(); arreterVideo(); },
};
