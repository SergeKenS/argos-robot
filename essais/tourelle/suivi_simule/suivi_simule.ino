// suivi_simule.ino
// Essai 2 de la tourelle : un objet virtuel se déplace et la tourelle le suit,
// comme elle suivra plus tard une vraie balle vue par la caméra.
// L'objet alterne deux phases :
//   - mouvement doux (8 s) : il décrit une boucle en forme de 8 ;
//   - sauts (4 s) : il change brusquement de place chaque seconde.
// Outils > Traceur série (115200 bauds) affiche l'objet et la tourelle.
//
// Branchement : le même que balayage.ino
//   fils bruns                              -> GND
//   fils rouges                             -> VIN (5 V venant de l'USB)
//   orange du servo du bas  (gauche/droite) -> D18
//   orange du servo du haut (haut/bas)      -> D19

#include <ESP32Servo.h>

// Broches de signal des servos
const int BROCHE_BAS = 18;
const int BROCHE_HAUT = 19;

// Centres mesurés sur la monture (caméra droit devant)
const int CENTRE_BAS = 100;
const int CENTRE_HAUT = 30;

// Écart maximal autorisé de part et d'autre du centre
const int AMPLITUDE = 45;

// Rythme de la boucle : celui du servo, qui lit sa consigne toutes les 20 ms
const int PERIODE_MS = 20;

// Réactivité : part de l'écart parcourue à chaque période (entre 0 et 1).
// Plus grand = plus réactif, mais avec une vraie caméra, risque de dépasser
// la balle et d'osciller.
const float GAIN = 0.3;

// Durée des deux phases de l'objet virtuel, en millisecondes
const unsigned long DUREE_DOUX_MS = 8000;
const unsigned long DUREE_SAUTS_MS = 4000;
const unsigned long INTERVALLE_SAUT_MS = 1000;

Servo servoBas;
Servo servoHaut;

// Positions de l'objet virtuel et de la tourelle, en degrés
float objetBas = CENTRE_BAS;
float objetHaut = CENTRE_HAUT;
float tourelleBas = CENTRE_BAS;
float tourelleHaut = CENTRE_HAUT;

// Moment du dernier saut de l'objet
unsigned long dernierSaut = 0;

// Garde un angle dans la plage autorisée autour du centre et dans 0° à 180°
float limiter(float angle, int centre) {
  float mini = max(centre - AMPLITUDE, 0);
  float maxi = min(centre + AMPLITUDE, 180);
  return constrain(angle, mini, maxi);
}

// Fait avancer l'objet virtuel selon la phase en cours
void deplacerObjet(unsigned long maintenant) {
  // Temps écoulé depuis le début du cycle (mouvement doux puis sauts)
  unsigned long t = maintenant % (DUREE_DOUX_MS + DUREE_SAUTS_MS);

  if (t < DUREE_DOUX_MS) {
    // Mouvement doux : un 8, obtenu avec deux sinus de périodes 4 s et 2 s
    float secondes = t / 1000.0;
    objetBas = CENTRE_BAS + 40 * sin(2 * PI * secondes / 4.0);
    objetHaut = CENTRE_HAUT + 25 * sin(2 * PI * secondes / 2.0);
  } else if (maintenant - dernierSaut >= INTERVALLE_SAUT_MS) {
    // Sauts : nouvelle position au hasard
    objetBas = random(CENTRE_BAS - AMPLITUDE, CENTRE_BAS + AMPLITUDE + 1);
    objetHaut = random(CENTRE_HAUT - AMPLITUDE, CENTRE_HAUT + AMPLITUDE + 1);
    dernierSaut = maintenant;
  }

  objetBas = limiter(objetBas, CENTRE_BAS);
  objetHaut = limiter(objetHaut, CENTRE_HAUT);
}

void setup() {
  // Messages lisibles dans le moniteur ou le traceur série (115200 bauds)
  Serial.begin(115200);

  // Réserve les minuteries PWM de l'ESP32 (comme l'exemple de la bibliothèque)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Un servo SG90 attend une impulsion toutes les 20 ms, soit 50 Hz
  servoBas.setPeriodHertz(50);
  servoHaut.setPeriodHertz(50);

  // Largeur d'impulsion pour 0° et 180°, en microsecondes (valeurs usuelles du SG90)
  servoBas.attach(BROCHE_BAS, 500, 2400);
  servoHaut.attach(BROCHE_HAUT, 500, 2400);

  // Départ : la tourelle au centre
  servoBas.write(CENTRE_BAS);
  servoHaut.write(CENTRE_HAUT);
  delay(1000);
}

void loop() {
  deplacerObjet(millis());

  // La tourelle parcourt une partie de l'écart qui la sépare de l'objet :
  // vite quand il est loin, doucement quand elle arrive dessus
  tourelleBas += GAIN * (objetBas - tourelleBas);
  tourelleHaut += GAIN * (objetHaut - tourelleHaut);

  servoBas.write((int)round(tourelleBas));
  servoHaut.write((int)round(tourelleHaut));

  // Une ligne par période pour le traceur série (nom:valeur, séparés par des virgules)
  Serial.printf("objetBas:%.0f,tourelleBas:%.0f,objetHaut:%.0f,tourelleHaut:%.0f\n",
                objetBas, tourelleBas, objetHaut, tourelleHaut);

  delay(PERIODE_MS);
}
