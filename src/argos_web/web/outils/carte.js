// outils/carte.js : dessine la carte du bâtiment, les points et le robot.
// Utilisée par les panneaux Service et Cartographie (une instance par canvas).
// Fonctions en option : choisir un emplacement d'un clic, placer le robot
// (appuyer là où il est, glisser vers là où il regarde, relâcher).

const RAYON_ROBOT = 0.22;   // mètres (robot_radius dans nav2_params.yaml)

function couleur(nom) {
  return getComputedStyle(document.documentElement).getPropertyValue(nom).trim();
}
function rgb(hex) {
  const n = parseInt(hex.slice(1), 16);
  return [(n >> 16) & 255, (n >> 8) & 255, n & 255];
}

export class Carte {
  // canvas : l'élément <canvas> ; message : texte affiché tant que la carte n'est pas arrivée.
  // avecPoints : dessiner les points nommés ; surSelection(point) : appelée après un clic.
  constructor(canvas, robot, { message = null, avecPoints = true, surSelection = null } = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.robot = robot;
    this.message = message;
    this.avecPoints = avecPoints;
    this.surSelection = surSelection;
    this.selection = null;       // emplacement choisi {x, y} en mètres
    this.placement = null;       // placement en cours {depart, fin, fini}
    this.image = null;           // carte déjà convertie en image
    this.source = null;          // message /map d'où vient l'image
    this.ignorerClic = false;
    this.prevu = false;

    for (const evenement of ["carte", "pose", "points", "etat"]) {
      robot.ecouter(evenement, () => this.dessiner());
    }
    // Le canvas suit la taille de son cadre (et la densité de pixels de l'écran).
    new ResizeObserver(() => {
      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.round(canvas.clientWidth * dpr);
      canvas.height = Math.round(canvas.clientHeight * dpr);
      this.dessiner();
    }).observe(canvas);

    canvas.addEventListener("click", (e) => this.cliquer(e));
    canvas.addEventListener("pointerdown", (e) => this.appuyer(e));
    canvas.addEventListener("pointermove", (e) => this.glisser(e));
    canvas.addEventListener("pointerup", () => this.relacher());
  }

  // ----- Placement du robot (équivalent de « 2D Pose Estimate » de RViz) -----
  // fini(x, y, t) est appelée au relâchement ; t = angle en radians.
  commencerPlacement(fini) {
    this.placement = { depart: null, fin: null, fini };
    this.canvas.style.touchAction = "none";   // pas de défilement pendant le geste
    this.dessiner();
  }
  annulerPlacement() {
    this.placement = null;
    this.canvas.style.touchAction = "";
    this.dessiner();
  }

  oublierSelection() { this.selection = null; this.dessiner(); }

  // ----- Conversions mètres <-> pixels -----
  vue() {
    const { width: l, height: h, resolution: r, origin } = this.robot.carte.info;
    const echelle = Math.min(this.canvas.width / l, this.canvas.height / h);   // pixels par cellule
    return {
      echelle, r, h, ox: origin.position.x, oy: origin.position.y,
      dx: (this.canvas.width - l * echelle) / 2, dy: (this.canvas.height - h * echelle) / 2,
    };
  }
  versEcran(v, x, y) {
    return [v.dx + ((x - v.ox) / v.r) * v.echelle, v.dy + (v.h - (y - v.oy) / v.r) * v.echelle];
  }
  // Position du pointeur sur la carte, en mètres (null hors de la carte).
  pointeur(e) {
    if (!this.robot.carte) return null;
    const dpr = window.devicePixelRatio || 1;
    const rect = this.canvas.getBoundingClientRect();
    const v = this.vue();
    const px = (e.clientX - rect.left) * dpr, py = (e.clientY - rect.top) * dpr;
    const x = v.ox + ((px - v.dx) / v.echelle) * v.r;
    const y = v.oy + (v.h - (py - v.dy) / v.echelle) * v.r;
    const i = this.robot.carte.info;
    if (x < v.ox || x > v.ox + i.width * i.resolution || y < v.oy || y > v.oy + i.height * i.resolution) return null;
    return { x, y };
  }

  // ----- Gestes -----
  cliquer(e) {
    if (this.ignorerClic) { this.ignorerClic = false; return; }
    if (!this.surSelection || this.placement) return;
    const p = this.pointeur(e);
    if (!p) return;
    this.selection = { x: Math.round(p.x * 100) / 100, y: Math.round(p.y * 100) / 100 };
    this.surSelection(this.selection);
    this.dessiner();
  }
  appuyer(e) {
    if (!this.placement) return;
    const p = this.pointeur(e);
    if (!p) return;
    this.placement.depart = p;
    this.canvas.setPointerCapture(e.pointerId);   // suivre le geste même hors du canvas
    this.dessiner();
  }
  glisser(e) {
    if (!this.placement || !this.placement.depart) return;
    this.placement.fin = this.pointeur(e) || this.placement.fin;
    this.dessiner();
  }
  relacher() {
    if (!this.placement || !this.placement.depart) return;
    const { depart, fin, fini } = this.placement;
    // Orientation = direction du glissé. Glissé trop court : on garde l'orientation actuelle.
    const glisse = fin && Math.hypot(fin.x - depart.x, fin.y - depart.y) > 0.1;
    const pose = this.robot.pose;
    const t = glisse ? Math.atan2(fin.y - depart.y, fin.x - depart.x) : (pose ? pose.t : 0);
    this.ignorerClic = true;   // le « click » qui suit ne doit pas devenir une sélection
    this.annulerPlacement();
    fini(depart.x, depart.y, t);
  }

  // ----- Dessin -----
  // Regroupe les demandes : au plus une image par rafraîchissement d'écran.
  dessiner() {
    if (this.prevu) return;
    this.prevu = true;
    requestAnimationFrame(() => { this.prevu = false; this.dessinerMaintenant(); });
  }

  // Une cellule = un pixel ; dans le message, la ligne 0 est en BAS de la carte.
  construireImage() {
    const carte = this.robot.carte;
    const { width: l, height: h } = carte.info;
    const image = document.createElement("canvas");
    image.width = l; image.height = h;
    const ic = image.getContext("2d");
    const pixels = ic.createImageData(l, h);
    const libre = rgb(couleur("--carte-libre")), occupe = rgb(couleur("--carte-occupe"));
    const inconnu = rgb(couleur("--carte-inconnu"));
    for (let ligne = 0; ligne < h; ligne++) {
      for (let col = 0; col < l; col++) {
        const valeur = carte.data[ligne * l + col];   // -1 inconnu, 0 libre ... 100 occupé
        const c = valeur < 0 ? inconnu : valeur >= 50 ? occupe : libre;
        const i = ((h - 1 - ligne) * l + col) * 4;
        pixels.data[i] = c[0]; pixels.data[i + 1] = c[1]; pixels.data[i + 2] = c[2]; pixels.data[i + 3] = 255;
      }
    }
    ic.putImageData(pixels, 0, 0);
    return image;
  }

  dessinerMaintenant() {
    const { ctx, canvas, robot } = this;
    if (canvas.width === 0) return;   // panneau caché
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    if (this.message) this.message.hidden = Boolean(robot.carte);
    if (!robot.carte) return;
    if (this.source !== robot.carte) { this.image = this.construireImage(); this.source = robot.carte; }
    const v = this.vue();
    const dpr = window.devicePixelRatio || 1;

    ctx.imageSmoothingEnabled = false;   // garder des cellules nettes
    ctx.drawImage(this.image, v.dx, v.dy, robot.carte.info.width * v.echelle, robot.carte.info.height * v.echelle);

    // Points nommés. En cartographie, ils appartiennent à l'ancienne carte : cachés.
    if (this.avecPoints && robot.etat.mode !== "CARTOGRAPHIE") {
      ctx.font = `600 ${13 * dpr}px Rajdhani, sans-serif`;
      for (const [nom, p] of Object.entries(robot.points)) {
        const [x, y] = this.versEcran(v, p.x, p.y);
        const ordre = robot.patrouille.indexOf(nom);
        const cible = nom === robot.etat.cible;
        const taille = (cible ? 9 : 7) * dpr;
        ctx.beginPath();
        // La base est un carré, les autres points des disques.
        if (nom === "base") ctx.rect(x - taille, y - taille, 2 * taille, 2 * taille);
        else ctx.arc(x, y, taille, 0, 2 * Math.PI);
        ctx.fillStyle = nom === "base" ? couleur("--service") : ordre >= 0 ? couleur("--ok") : couleur("--carte-point");
        ctx.fill();
        if (cible) { ctx.lineWidth = 3 * dpr; ctx.strokeStyle = couleur("--orange"); ctx.stroke(); }
        ctx.fillStyle = couleur("--texte");
        ctx.fillText(ordre >= 0 ? `${ordre + 1}. ${nom}` : nom, x + 11 * dpr, y + 4 * dpr);
      }
    }

    // Emplacement choisi : une croix.
    if (this.selection) {
      const [x, y] = this.versEcran(v, this.selection.x, this.selection.y);
      const t = 7 * dpr;
      ctx.strokeStyle = couleur("--carte-selection"); ctx.lineWidth = 2.5 * dpr;
      ctx.beginPath();
      ctx.moveTo(x - t, y - t); ctx.lineTo(x + t, y + t);
      ctx.moveTo(x + t, y - t); ctx.lineTo(x - t, y + t);
      ctx.stroke();
    }

    // Robot : un disque à sa vraie taille, avec un trait dans le sens de la marche.
    if (robot.pose) {
      const [x, y] = this.versEcran(v, robot.pose.x, robot.pose.y);
      const rayon = Math.max((RAYON_ROBOT / v.r) * v.echelle, 6 * dpr);
      ctx.beginPath();
      ctx.arc(x, y, rayon, 0, 2 * Math.PI);
      ctx.fillStyle = couleur("--orange"); ctx.globalAlpha = 0.85; ctx.fill(); ctx.globalAlpha = 1;
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(x + Math.cos(robot.pose.t) * rayon * 1.6, y - Math.sin(robot.pose.t) * rayon * 1.6);
      ctx.strokeStyle = couleur("--orange"); ctx.lineWidth = 3 * dpr; ctx.stroke();
    }

    // Placement en cours : flèche pointillée du point pressé vers le pointeur.
    if (this.placement && this.placement.depart) {
      const [x1, y1] = this.versEcran(v, this.placement.depart.x, this.placement.depart.y);
      const fin = this.placement.fin || this.placement.depart;
      const [x2, y2] = this.versEcran(v, fin.x, fin.y);
      ctx.setLineDash([6 * dpr, 4 * dpr]);
      ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2);
      ctx.strokeStyle = couleur("--orange"); ctx.lineWidth = 3 * dpr; ctx.stroke();
      ctx.setLineDash([]);
      ctx.beginPath(); ctx.arc(x1, y1, 6 * dpr, 0, 2 * Math.PI);
      ctx.fillStyle = couleur("--orange"); ctx.fill();
    }
  }
}
