// outils/joystick.js : joystick tactile. On pose le doigt n'importe où dans le
// cercle et on glisse : la manette suit, la direction est proportionnelle à
// l'écart au centre. Au relâchement, la manette revient au centre (direction nulle).
// Direction {x, y} entre -1 et 1 : x positif = droite, y positif = haut.

export class Joystick {
  // element : le cercle, qui contient un élément .manette.
  // surChangement(direction) : appelée à chaque mouvement.
  constructor(element, surChangement) {
    this.element = element;
    this.manette = element.querySelector(".manette");
    this.surChangement = surChangement;
    this.doigt = null;        // identifiant du pointeur qui tient la manette
    this.actif = true;

    element.addEventListener("pointerdown", (e) => {
      if (!this.actif || this.doigt !== null) return;
      this.doigt = e.pointerId;
      element.setPointerCapture(e.pointerId);
      this.suivre(e);
    });
    element.addEventListener("pointermove", (e) => {
      if (e.pointerId === this.doigt) this.suivre(e);
    });
    for (const fin of ["pointerup", "pointercancel", "lostpointercapture"]) {
      element.addEventListener(fin, (e) => { if (e.pointerId === this.doigt) this.relacher(); });
    }
    element.addEventListener("contextmenu", (e) => e.preventDefault());
  }

  suivre(e) {
    const rect = this.element.getBoundingClientRect();
    // Course maximale : du centre au bord, moins la moitié de la manette.
    const course = (rect.width - this.manette.offsetWidth) / 2;
    let dx = e.clientX - (rect.left + rect.width / 2);
    let dy = e.clientY - (rect.top + rect.height / 2);
    const distance = Math.hypot(dx, dy);
    if (distance > course) { dx *= course / distance; dy *= course / distance; }
    this.manette.style.transform = `translate(${dx}px, ${dy}px)`;
    // Petite zone morte au centre : un doigt posé sans bouger ne fait rien.
    const x = dx / course, y = -dy / course;
    const zoneMorte = 0.12;
    this.surChangement({
      x: Math.abs(x) < zoneMorte ? 0 : x,
      y: Math.abs(y) < zoneMorte ? 0 : y,
    });
  }

  relacher() {
    this.doigt = null;
    this.manette.style.transform = "";
    this.surChangement({ x: 0, y: 0 });
  }
}
