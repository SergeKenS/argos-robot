// panneaux/service.js : rôle Service (appel vers un point, patrouille, retour à la base).
// Aussi : placer le robot sur la carte, gérer les points, changer de carte, réglages.
// Commandes envoyées au répartiteur : appel <point>, patrouille, base, veille,
// ajouter, supprimer, definir_patrouille, carte <nom>, redemarrer_navigation.

import { Carte } from "../outils/carte.js";

const FORMAT_NOM = /^[a-z0-9_]+$/;   // même règle que le répartiteur
const NOEUDS_COSTMAP = ["/global_costmap/global_costmap", "/local_costmap/local_costmap"];
const PARAM_MARGE = "inflation_layer.inflation_radius";
const RAYON_ROBOT = 0.22;            // mètres : la marge ne descend pas en dessous

let robot, carte;
const $ = (id) => document.getElementById(id);
const metres = (v) => `${Number(v).toFixed(2).replace(".", ",")} m`;

function initialiser(r) {
  robot = r;
  carte = new Carte($("service-canvas"), robot, {
    message: $("service-message-carte"),
    surSelection: (p) => {
      $("service-info-selection").textContent = `Emplacement choisi : x ${p.x} · y ${p.y}`;
      majBoutonsAjout();
    },
  });

  // Les boutons des points ne sont refaits qu'au changement d'état ou de points,
  // jamais à chaque position (10 fois/s) : un bouton refait pendant un clic le perdrait.
  robot.ecouter("etat", () => { majEnCours(); afficherPoints(); });
  robot.ecouter("pose", () => { majEnCours(); majBoutonsAjout(); });
  robot.ecouter("points", () => { afficherPoints(); majEnCours(); afficherCartes(); });
  robot.ecouter("connexion", lireMarge);

  // ----- Carte : placer le robot, changer de carte -----
  $("btn-placer").addEventListener("click", basculerPlacement);
  $("btn-charger-carte").addEventListener("click", () => {
    const nom = $("choix-carte").value;
    if (!nom || nom === robot.carteNom) return;
    if (confirm(`Charger la carte « ${nom} » ? Le robot s'arrête et devra être replacé.`)) {
      carte.oublierSelection();
      robot.envoyer(`carte ${nom}`);
    }
  });

  // ----- Mission -----
  $("btn-annuler").addEventListener("click", () => robot.envoyer("veille"));
  $("btn-patrouille").addEventListener("click", () => robot.envoyer("patrouille"));
  $("btn-base").addEventListener("click", () => robot.envoyer("base"));

  // ----- Ajout d'un point -----
  $("nom-point").addEventListener("input", (e) => {
    e.target.value = e.target.value.toLowerCase().replace(/[\s-]/g, "_");
    majBoutonsAjout();
  });
  $("btn-ajout-robot").addEventListener("click", () => {
    robot.envoyer(`ajouter ${nomValide()}`);
    $("nom-point").value = ""; majBoutonsAjout();
  });
  $("btn-ajout-clic").addEventListener("click", () => {
    robot.envoyer(`ajouter ${nomValide()} ${carte.selection.x} ${carte.selection.y}`);
    $("nom-point").value = ""; majBoutonsAjout();
  });

  // ----- Réglages -----
  $("curseur-marge").addEventListener("input", (e) => { $("valeur-marge").textContent = metres(e.target.value); });
  $("btn-marge").addEventListener("click", appliquerMarge);
  $("btn-redemarrer").addEventListener("click", () => robot.envoyer("redemarrer_navigation"));

  afficherPoints();
  majEnCours();
}

// ===== Placement du robot ======================================================
function basculerPlacement() {
  if (carte.placement) {
    carte.annulerPlacement();
    finPlacement();
    return;
  }
  carte.commencerPlacement((x, y, t) => { publierPositionInitiale(x, y, t); finPlacement(); });
  $("btn-placer").textContent = "Annuler le placement";
  $("service-info-selection").textContent = "Appuyez sur la position du robot, glissez vers où il regarde, relâchez.";
}
function finPlacement() {
  $("btn-placer").textContent = "Placer le robot";
  $("service-info-selection").textContent = "Touchez la carte pour choisir un emplacement.";
}

function publierPositionInitiale(x, y, t) {
  if (!robot.connecte) return;
  const covariance = new Array(36).fill(0);
  // Incertitude de départ, mêmes valeurs que RViz : 0,25 m² en x et y, ~0,07 rad² en angle.
  covariance[0] = 0.25; covariance[7] = 0.25; covariance[35] = 0.06853891945200942;
  robot.topic("/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped").publish({
    // AMCL n'accepte une position que dans le repère de la carte (« map »).
    header: { frame_id: "map", stamp: { sec: 0, nanosec: 0 } },
    pose: {
      pose: { position: { x, y, z: 0 }, orientation: { x: 0, y: 0, z: Math.sin(t / 2), w: Math.cos(t / 2) } },
      covariance,
    },
  });
  robot.journal(`Robot placé : x ${x.toFixed(2)} · y ${y.toFixed(2)} · θ ${t.toFixed(2)}`);
}

// ===== Mission en cours ========================================================
function majEnCours() {
  const { mode, cible } = robot.etat;
  const bloc = $("bloc-en-cours");
  const actif = ["APPEL", "PATROUILLE", "ECHEC"].includes(mode);
  bloc.hidden = !actif;
  if (!actif) return;
  const nom = (cible || "").replace(/_/g, " ");
  bloc.classList.toggle("echec", mode === "ECHEC");
  $("en-cours-titre").textContent = mode === "APPEL" ? `Appel ${nom}`
    : mode === "PATROUILLE" ? `Patrouille · ${nom}` : `Échec · ${nom}`;
  // Distance à vol d'oiseau jusqu'au point visé (le trajet réel est plus long).
  const point = robot.points[cible];
  let surtitre = mode === "ECHEC" ? "EN ATTENTE D'UNE COMMANDE" : "EN ROUTE";
  if (point && robot.pose && mode !== "ECHEC") {
    const d = Math.hypot(point.x - robot.pose.x, point.y - robot.pose.y);
    surtitre += ` · à ${d.toFixed(1).replace(".", ",")} m`;
  }
  $("en-cours-surtitre").textContent = surtitre;
  $("btn-annuler").textContent = mode === "ECHEC" ? "Veille" : "Annuler";
}

// ===== Points ==================================================================
function afficherPoints() {
  const noms = Object.keys(robot.points);

  // Boutons d'appel (la base a son propre bouton).
  const appels = noms.filter((n) => n !== "base").map((nom) => {
    const bouton = document.createElement("button");
    bouton.type = "button";
    bouton.textContent = nom.replace(/_/g, " ");
    if (nom === robot.etat.cible && robot.etat.mode === "APPEL") bouton.dataset.cible = "";
    bouton.addEventListener("click", () => robot.envoyer(`appel ${nom}`));
    return bouton;
  });
  if (appels.length === 0) {
    const vide = document.createElement("p");
    vide.className = "aide";
    vide.textContent = "Aucun point. Ajoutez-en dans « Points » ci-dessous.";
    appels.push(vide);
  }
  $("points-appel").replaceChildren(...appels);

  // Liste détaillée : patrouille et suppression.
  const lignes = noms.map((nom) => {
    const p = robot.points[nom];
    const li = document.createElement("li");
    const infos = document.createElement("div");
    infos.className = "infos";
    const titre = document.createElement("div");
    titre.className = "nom"; titre.textContent = nom;
    const coord = document.createElement("div");
    coord.className = "coord"; coord.textContent = `x ${p.x} · y ${p.y} · θ ${p.theta}`;
    infos.append(titre, coord);

    // Case « patrouille » : ajoute le point à la fin de la boucle, ou l'en retire.
    const label = document.createElement("label");
    const caseP = document.createElement("input");
    caseP.type = "checkbox"; caseP.checked = robot.patrouille.includes(nom);
    caseP.addEventListener("change", () => {
      const nouvelle = caseP.checked ? [...robot.patrouille, nom] : robot.patrouille.filter((n) => n !== nom);
      robot.envoyer(`definir_patrouille ${nouvelle.join(" ")}`.trim());
    });
    label.append(caseP, "Ronde");

    const supprimer = document.createElement("button");
    supprimer.type = "button";
    supprimer.className = "icone petit discret";
    supprimer.setAttribute("aria-label", `Supprimer le point ${nom}`);
    supprimer.textContent = "✕";
    supprimer.addEventListener("click", () => {
      if (confirm(`Supprimer le point « ${nom} » ?`)) robot.envoyer(`supprimer ${nom}`);
    });

    li.append(infos, label, supprimer);
    return li;
  });
  $("liste-points").replaceChildren(...lignes);

  $("ordre-patrouille").textContent = robot.patrouille.length ? robot.patrouille.join(" > ") : "";
  $("btn-patrouille").disabled = robot.patrouille.length === 0;
  $("btn-base").disabled = !("base" in robot.points);
}

// Nom du nouveau point : vérifié ici avant l'envoi (le répartiteur revérifie).
function nomValide() {
  const nom = $("nom-point").value.trim();
  const ok = FORMAT_NOM.test(nom);
  $("aide-nom").classList.toggle("erreur", nom !== "" && !ok);
  return ok ? nom : null;
}
function majBoutonsAjout() {
  const nom = nomValide();
  $("btn-ajout-robot").disabled = !nom || !robot.pose;
  $("btn-ajout-clic").disabled = !nom || !carte.selection;
}

// ===== Cartes disponibles ======================================================
function afficherCartes() {
  const choix = $("choix-carte");
  choix.replaceChildren(...robot.cartes.map((nom) => new Option(nom, nom)));
  choix.value = robot.carteNom;
  $("service-titre-carte").textContent = robot.carteNom ? `CARTE : ${robot.carteNom}` : "CARTE";
}

// ===== Marge de sécurité (paramètre de Nav2 modifié en direct) ==================
// Lecture par le service standard « get_parameters » de chaque nœud ROS 2.
function lireMarge() {
  robot.service(`${NOEUDS_COSTMAP[0]}/get_parameters`, "rcl_interfaces/srv/GetParameters")
    .callService({ names: [PARAM_MARGE] }, (reponse) => {
      const v = reponse.values && reponse.values[0] && reponse.values[0].double_value;
      if (v) { $("curseur-marge").value = v; $("valeur-marge").textContent = metres(v); }
    }, () => { /* Nav2 absent (robot sans LiDAR) : le réglage garde sa valeur */ });
}

// Applique la valeur aux deux cartes de coûts (globale et locale).
function appliquerMarge() {
  const v = Math.max(RAYON_ROBOT, parseFloat($("curseur-marge").value));
  for (const noeud of NOEUDS_COSTMAP) {
    robot.service(`${noeud}/set_parameters`, "rcl_interfaces/srv/SetParameters").callService(
      // type 3 = nombre décimal (rcl_interfaces/msg/ParameterType.PARAMETER_DOUBLE)
      { parameters: [{ name: PARAM_MARGE, value: { type: 3, double_value: v } }] },
      (reponse) => {
        const r = reponse.results && reponse.results[0];
        if (r && r.successful) robot.journal(`Marge ${metres(v)} appliquée (${noeud.split("/")[1]}).`);
        else robot.journal(`Marge refusée (${noeud}) : ${r ? r.reason : "?"}`, "erreur");
      },
      (erreur) => robot.journal(`Marge : échec (${erreur}).`, "erreur"));
  }
}

export default {
  role: "service",
  titre: "Service",
  modes: ["APPEL", "PATROUILLE", "ECHEC"],   // états de /argos/etat qui appartiennent à ce rôle
  initialiser,
  cacher() { if (carte.placement) { carte.annulerPlacement(); finPlacement(); } },
};
