// app.js : le cœur de la page.
//   - connexion au robot par rosbridge (reconnexion automatique) ;
//   - données partagées par tous les panneaux : état, points, carte, position ;
//   - onglets (un par rôle du robot) et panneau visible ;
//   - journal (10 messages affichés, 200 gardés en mémoire).
// Chaque panneau (dossier panneaux/) ne connaît que l'objet « robot » ci-dessous.

import service from "./panneaux/service.js";
import cartographie from "./panneaux/cartographie.js";
import manuel from "./panneaux/manuel.js";

// Ajouter un rôle à la page = un fichier dans panneaux/ + une ligne ici + sa section dans index.html.
const PANNEAUX = [service, cartographie, manuel];
// Rôles annoncés mais pas encore écrits (onglet grisé).
const BIENTOT = [{ role: "suivi", titre: "Suivi" }];

// rosbridge tourne sur la même machine que le serveur de la page, port 9090.
const URL_ROSBRIDGE = `ws://${location.hostname || "localhost"}:9090`;
const JOURNAL_AFFICHES = 10;    // messages visibles sans « Voir plus »
const JOURNAL_MAX = 200;        // au-delà, les plus anciens sont oubliés

const $ = (id) => document.getElementById(id);

// ===== Petits calculs sur les poses 2D {x, y, t} ===============================
// Angle de rotation autour de z (« lacet ») à partir d'un quaternion.
export function lacet(q) {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}
// Enchaîner deux déplacements, et défaire un déplacement.
function composer(a, b) {
  const c = Math.cos(a.t), s = Math.sin(a.t);
  return { x: a.x + c * b.x - s * b.y, y: a.y + s * b.x + c * b.y, t: a.t + b.t };
}
function inverser(p) {
  const c = Math.cos(p.t), s = Math.sin(p.t);
  return { x: -(c * p.x + s * p.y), y: -(-s * p.x + c * p.y), t: -p.t };
}

// ===== L'objet « robot » : tout ce que les panneaux peuvent utiliser ==========
const robot = {
  ros: null,                  // connexion rosbridge (null si hors ligne)
  connecte: false,
  etat: { mode: null, cible: null, texte: "" },   // dernier /argos/etat
  points: {},                 // { nom: {x, y, theta} }
  patrouille: [],             // [nom, ...]
  carteNom: null,             // nom de la carte chargée (ex. « arene »)
  cartes: [],                 // cartes disponibles
  carte: null,                // dernier message /map
  pose: null,                 // position du robot sur la carte {x, y, t}
  vitesse: null,              // {avance en m/s, rotation en rad/s} d'après /odom
  visible: null,              // rôle du panneau affiché

  // Abonnement aux événements : connexion, deconnexion, etat, points, carte, pose, vitesse.
  ecouteurs: {},
  ecouter(evenement, fonction) { (this.ecouteurs[evenement] ||= []).push(fonction); },
  emettre(evenement, ...valeurs) {
    for (const fonction of this.ecouteurs[evenement] || []) {
      try { fonction(...valeurs); } catch (erreur) { console.error(erreur); }
    }
  },

  // Crée un topic ou un service ROS sur la connexion actuelle.
  topic(name, messageType, options = {}) {
    return new ROSLIB.Topic({ ros: this.ros, name, messageType, ...options });
  },
  service(name, serviceType) {
    return new ROSLIB.Service({ ros: this.ros, name, serviceType });
  },

  // Envoie une commande texte au répartiteur (ex. « appel chambre_101 »).
  envoyer(texte) {
    if (!this.connecte || !topicCommande) return;
    topicCommande.publish({ data: texte });
    this.journal(`→ ${texte}`);
  },

  journal(texte, niveau = "info") { ajouterAuJournal(texte, niveau); },
};

let topicCommande = null;
let carteVersOdom = null;     // écart carte → odométrie (voir recevoirPoseCarte)
let odomActuelle = null;

// ===== Connexion ===============================================================
function connecter() {
  const ros = new ROSLIB.Ros({ url: URL_ROSBRIDGE });

  ros.on("connection", () => {
    robot.ros = ros;
    robot.connecte = true;
    document.body.classList.remove("deconnecte");
    $("liaison").textContent = $("tiroir-liaison").textContent = "LIAISON OK";
    robot.journal("Connecté au robot.");
    abonner();
    robot.emettre("connexion");
  });
  ros.on("close", () => {
    const etaitConnecte = robot.connecte;
    robot.connecte = false;
    robot.ros = null;
    topicCommande = null;
    document.body.classList.add("deconnecte");
    $("liaison").textContent = $("tiroir-liaison").textContent = "HORS LIGNE";
    if (etaitConnecte) {
      robot.journal("Liaison perdue, nouvel essai toutes les 3 s.", "avertissement");
      robot.emettre("deconnexion");
    }
    setTimeout(connecter, 3000);
  });
  ros.on("error", () => {});   // « close » suit toujours une erreur
}

function abonner() {
  topicCommande = robot.topic("/argos/commande", "std_msgs/msg/String");
  topicCommande.advertise();

  robot.topic("/argos/etat", "std_msgs/msg/String").subscribe((m) => recevoirEtat(m.data));
  robot.topic("/argos/points", "std_msgs/msg/String").subscribe((m) => recevoirPoints(JSON.parse(m.data)));
  // Messages du répartiteur (point enregistré, commande refusée, échec...).
  robot.topic("/argos/journal", "std_msgs/msg/String").subscribe((m) => {
    const { niveau, texte } = JSON.parse(m.data);
    robot.journal(`Robot : ${texte}`, niveau);
  });
  robot.topic("/map", "nav_msgs/msg/OccupancyGrid").subscribe((m) => {
    robot.carte = m;
    robot.emettre("carte");
  });
  // Position : AMCL en navigation, SLAM Toolbox pendant la cartographie (même format).
  const typePose = "geometry_msgs/msg/PoseWithCovarianceStamped";
  robot.topic("/amcl_pose", typePose).subscribe(recevoirPoseCarte);
  robot.topic("/pose", typePose).subscribe(recevoirPoseCarte);
  // L'odométrie arrive très souvent : limitée à 10 messages/s (throttle_rate en ms).
  robot.topic("/odom", "nav_msgs/msg/Odometry", { throttle_rate: 100 }).subscribe(recevoirOdom);
}

// ===== Réception des données partagées =========================================
function recevoirEtat(texte) {
  const [mode, ...reste] = texte.split(" ");
  const cible = reste.join(" ") || null;
  const ancien = robot.etat;
  if (mode !== ancien.mode || cible !== ancien.cible) robot.journal(`État : ${texte}`);
  // Début de cartographie : la position connue appartient à l'ancienne carte.
  if (mode === "CARTOGRAPHIE" && ancien.mode !== "CARTOGRAPHIE") oublierPosition();
  robot.etat = { mode, cible, texte };

  // Étiquette de l'en-tête : le rôle actif, d'un seul mot.
  const panneau = panneauDuMode(mode);
  const etiquette = $("etiquette-etat");
  etiquette.textContent = mode;
  etiquette.dataset.role = panneau ? panneau.role : "";
  if (mode === "ECHEC") etiquette.dataset.echec = ""; else delete etiquette.dataset.echec;
  for (const bouton of boutonsDeRole()) {
    if (panneau && bouton.dataset.role === panneau.role) bouton.dataset.actif = "";
    else delete bouton.dataset.actif;
  }
  // Même étiquette dans le menu.
  const copie = $("tiroir-etiquette");
  copie.textContent = etiquette.textContent;
  copie.dataset.role = etiquette.dataset.role;
  if ("echec" in etiquette.dataset) copie.dataset.echec = ""; else delete copie.dataset.echec;
  // Le robot change de rôle : la page montre le panneau qui correspond.
  if (panneau && mode !== ancien.mode) montrer(panneau.role);
  robot.emettre("etat", robot.etat);
}

function recevoirPoints(donnees) {
  robot.points = donnees.points || {};
  robot.patrouille = donnees.patrouille || [];
  robot.cartes = donnees.cartes || [];
  // Nouvelle carte chargée : l'ancienne position du robot n'a plus de sens.
  if (robot.carteNom && donnees.carte !== robot.carteNom) {
    oublierPosition();
    robot.journal(`Carte « ${donnees.carte} » chargée.`);
  }
  robot.carteNom = donnees.carte;
  robot.emettre("points");
}

// AMCL ne publie sa position qu'après un déplacement notable (~25 cm) : le robot
// avancerait par sauts. On retient l'écart carte → odométrie à chaque position
// reçue, puis on suit l'odométrie (fluide) entre deux mises à jour.
function recevoirPoseCarte(message) {
  const p = message.pose.pose;
  robot.pose = { x: p.position.x, y: p.position.y, t: lacet(p.orientation) };
  carteVersOdom = odomActuelle ? composer(robot.pose, inverser(odomActuelle)) : null;
  robot.emettre("pose");
}

function recevoirOdom(message) {
  const p = message.pose.pose;
  odomActuelle = { x: p.position.x, y: p.position.y, t: lacet(p.orientation) };
  robot.vitesse = { avance: message.twist.twist.linear.x, rotation: message.twist.twist.angular.z };
  robot.emettre("vitesse", robot.vitesse);
  if (carteVersOdom) {
    robot.pose = composer(carteVersOdom, odomActuelle);
    robot.emettre("pose");
  }
}

function oublierPosition() {
  robot.pose = null;
  carteVersOdom = null;
  robot.emettre("pose");
}

// ===== Onglets et panneaux =====================================================
function panneauDuMode(mode) {
  return PANNEAUX.find((p) => p.modes.includes(mode)) || null;
}

// Les onglets (en haut) et le menu (à gauche) sont faits à partir de la même liste.
function construireOnglets() {
  for (const nav of [$("onglets"), $("tiroir-roles")]) {
    for (const p of PANNEAUX) {
      const bouton = document.createElement("button");
      bouton.type = "button";
      bouton.textContent = p.titre;
      bouton.dataset.role = p.role;
      bouton.setAttribute("aria-pressed", "false");
      bouton.addEventListener("click", () => { montrer(p.role); fermerMenu(); });
      nav.append(bouton);
    }
    for (const b of BIENTOT) {
      const bouton = document.createElement("button");
      bouton.type = "button";
      bouton.textContent = `${b.titre} · bientôt`;
      bouton.dataset.bientot = "";
      bouton.disabled = true;
      nav.append(bouton);
    }
  }
}

function boutonsDeRole() {
  return [...$("onglets").children, ...$("tiroir-roles").children];
}

// ===== Menu qui glisse depuis la gauche =========================================
function ouvrirMenu() {
  document.body.classList.add("tiroir-ouvert");
  $("tiroir").setAttribute("aria-hidden", "false");
}
function fermerMenu() {
  document.body.classList.remove("tiroir-ouvert");
  $("tiroir").setAttribute("aria-hidden", "true");
}

function montrer(role) {
  if (robot.visible === role) return;
  const ancien = PANNEAUX.find((p) => p.role === robot.visible);
  if (ancien && ancien.cacher) ancien.cacher();
  robot.visible = role;
  for (const section of document.querySelectorAll("[data-panneau]")) {
    section.hidden = section.dataset.panneau !== role;
  }
  for (const bouton of boutonsDeRole()) {
    bouton.setAttribute("aria-pressed", String(bouton.dataset.role === role));
  }
  // Le journal suit le panneau visible : il va en bas de sa colonne.
  const colonne = document.querySelector(`[data-panneau="${role}"] .colonne`);
  if (colonne) colonne.append($("bloc-journal"));
  document.body.classList.toggle("manuel-visible", role === "manuel");
  try { localStorage.setItem("argos-panneau", role); } catch { /* stockage bloqué : sans importance */ }
  const nouveau = PANNEAUX.find((p) => p.role === role);
  if (nouveau && nouveau.montrer) nouveau.montrer();
}

// ===== Journal =================================================================
const messages = [];          // du plus récent au plus ancien : {heure, texte, niveau}
let journalComplet = false;   // « Voir plus » actif

function ajouterAuJournal(texte, niveau = "info") {
  if (niveau === true) niveau = "erreur";
  const d = new Date(), deux = (n) => String(n).padStart(2, "0");
  const heure = `${deux(d.getHours())}:${deux(d.getMinutes())}:${deux(d.getSeconds())}`;
  messages.unshift({ heure, texte, niveau });
  if (messages.length > JOURNAL_MAX) messages.length = JOURNAL_MAX;
  afficherJournal();
}

function afficherJournal() {
  const liste = $("journal");
  if (!liste) return;
  const visibles = journalComplet ? messages : messages.slice(0, JOURNAL_AFFICHES);
  liste.replaceChildren(...visibles.map(({ heure, texte, niveau }) => {
    const li = document.createElement("li");
    const t = document.createElement("time");
    t.textContent = heure;
    const s = document.createElement("span");
    s.textContent = texte;
    if (niveau !== "info") s.className = niveau;
    li.append(t, s);
    return li;
  }));
  liste.classList.toggle("tout", journalComplet);
  const caches = messages.length - JOURNAL_AFFICHES;
  const bouton = $("btn-journal");
  bouton.hidden = caches <= 0;
  bouton.textContent = journalComplet ? "Voir moins" : `Voir plus (${caches})`;
}

// ===== Démarrage ===============================================================
function demarrer() {
  // Le journal vient d'un modèle : une seule copie, déplacée de panneau en panneau.
  document.body.append($("modele-journal").content.cloneNode(true));
  $("btn-journal").addEventListener("click", () => { journalComplet = !journalComplet; afficherJournal(); });

  $("btn-veille").addEventListener("click", () => robot.envoyer("veille"));
  $("btn-veille-tiroir").addEventListener("click", () => { robot.envoyer("veille"); fermerMenu(); });
  // Menu : ouvert par le bouton ☰ (panneau Manuel), fermé par la croix, à côté, ou Échap.
  document.addEventListener("ouvrir-menu", ouvrirMenu);
  $("btn-menu-entete").addEventListener("click", ouvrirMenu);
  $("btn-fermer-tiroir").addEventListener("click", fermerMenu);
  $("fond-tiroir").addEventListener("click", fermerMenu);
  document.addEventListener("keydown", (e) => { if (e.key === "Escape") fermerMenu(); });

  construireOnglets();
  for (const p of PANNEAUX) {
    p.initialiser(robot, document.querySelector(`[data-panneau="${p.role}"]`));
  }

  // Panneau demandé dans l'adresse (ex. http://argos-pi.local:8080/#manuel),
  // sinon le dernier ouvert, sinon Service.
  let premier = location.hash.slice(1);
  if (!PANNEAUX.some((p) => p.role === premier)) {
    try { premier = localStorage.getItem("argos-panneau"); } catch { /* idem */ }
  }
  if (!PANNEAUX.some((p) => p.role === premier)) premier = "service";
  montrer(premier);

  robot.journal("Connexion au robot...");
  connecter();
}

demarrer();
