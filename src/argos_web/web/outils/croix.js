// outils/croix.js : une croix de quatre boutons (haut, bas, gauche, droite),
// commandée à la souris, au doigt ou au clavier. Plusieurs directions peuvent
// être tenues à la fois (avancer en tournant). La croix donne une direction
// {x, y} entre -1 et 1 : x positif = droite, y positif = haut.

export class Croix {
  // element : conteneur des boutons marqués data-sens="haut|bas|gauche|droite".
  // touches : { code de touche : sens }, ex. { KeyW: "haut" } (code = position
  //           physique de la touche, la même sur un clavier QWERTY ou AZERTY).
  // surChangement(direction) : appelée à chaque changement.
  constructor(element, touches, surChangement) {
    this.element = element;
    this.touches = touches;
    this.surChangement = surChangement;
    this.tenus = new Set();   // sens tenus en ce moment
    this.actif = true;

    for (const bouton of element.querySelectorAll("[data-sens]")) {
      const sens = bouton.dataset.sens;
      bouton.addEventListener("pointerdown", (e) => {
        if (!this.actif) return;
        bouton.setPointerCapture(e.pointerId);
        this.appuyer(sens);
      });
      for (const fin of ["pointerup", "pointercancel", "lostpointercapture"]) {
        bouton.addEventListener(fin, () => this.relacher(sens));
      }
      // Pas de menu contextuel sur un appui long (téléphone).
      bouton.addEventListener("contextmenu", (e) => e.preventDefault());
    }
  }

  // Touche du clavier ; renvoie true si elle appartient à cette croix.
  touche(code, enfoncee) {
    const sens = this.touches[code];
    if (!sens) return false;
    if (enfoncee) this.appuyer(sens); else this.relacher(sens);
    return true;
  }

  appuyer(sens) {
    if (!this.actif || this.tenus.has(sens)) return;
    this.tenus.add(sens);
    this.majBoutons();
    this.surChangement(this.direction());
  }

  relacher(sens) {
    if (!this.tenus.delete(sens)) return;
    this.majBoutons();
    this.surChangement(this.direction());
  }

  toutRelacher() {
    if (this.tenus.size === 0) return;
    this.tenus.clear();
    this.majBoutons();
    this.surChangement(this.direction());
  }

  direction() {
    const t = this.tenus;
    return {
      x: (t.has("droite") ? 1 : 0) - (t.has("gauche") ? 1 : 0),
      y: (t.has("haut") ? 1 : 0) - (t.has("bas") ? 1 : 0),
    };
  }

  majBoutons() {
    for (const bouton of this.element.querySelectorAll("[data-sens]")) {
      bouton.classList.toggle("enfonce", this.tenus.has(bouton.dataset.sens));
    }
  }
}
