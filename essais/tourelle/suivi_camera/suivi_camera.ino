// suivi_camera.ino
// Essai 3 de la tourelle, côté ESP32 : reçoit des angles par l'USB et les
// applique aux servos. C'est le PC (suivi_balle.py) qui voit la balle et
// calcule les angles ; l'ESP32 ne fait qu'exécuter.
//
// Format reçu : une ligne "bas,haut" en degrés, par exemple "100,30".
// Test sans Python : moniteur série à 115200 bauds, taper "120,40" puis Entrée.
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

// Écart maximal autorisé de part et d'autre du centre, pour protéger la monture
const int AMPLITUDE = 45;

Servo servoBas;
Servo servoHaut;

// Garde un angle dans la plage autorisée autour du centre et dans 0° à 180°
int limiter(int angle, int centre) {
  int mini = max(centre - AMPLITUDE, 0);
  int maxi = min(centre + AMPLITUDE, 180);
  return constrain(angle, mini, maxi);
}

void setup() {
  Serial.begin(115200);
  // Attend au plus 50 ms la fin d'une ligne en cours de réception
  Serial.setTimeout(50);

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
  Serial.println("Prêt : envoyer bas,haut (ex. 100,30)");
}

void loop() {
  if (Serial.available() == 0) {
    return;
  }

  // Lit une ligne complète, puis en extrait les deux angles
  String ligne = Serial.readStringUntil('\n');
  int bas, haut;
  if (sscanf(ligne.c_str(), "%d,%d", &bas, &haut) != 2) {
    Serial.println("Format attendu : bas,haut (ex. 100,30)");
    return;
  }

  bas = limiter(bas, CENTRE_BAS);
  haut = limiter(haut, CENTRE_HAUT);
  servoBas.write(bas);
  servoHaut.write(haut);

  // Renvoie les angles réellement appliqués (après limites)
  Serial.printf("Appliqué : %d,%d\n", bas, haut);
}
