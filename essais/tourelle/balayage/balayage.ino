// balayage.ino
// Essai 1 de la tourelle : fait tourner les deux servos SG90, l'un après
// l'autre, de 45° à 135° et retour, lentement.
// But : vérifier le branchement et l'alimentation.
//
// Branchement :
//   fils bruns                              -> GND
//   fils rouges                             -> VIN (5 V venant de l'USB)
//   orange du servo du bas  (gauche/droite) -> D18
//   orange du servo du haut (haut/bas)      -> D19

#include <ESP32Servo.h>

// Broches de signal des servos
const int BROCHE_BAS = 18;
const int BROCHE_HAUT = 19;

// Centre de chaque servo : l'angle où la caméra regarde droit devant.
// À ajuster selon le montage (90 si les bras ont été fixés à 90°).
const int CENTRE_BAS = 100;
const int CENTRE_HAUT = 30;

// Balayage de part et d'autre du centre, pour ne pas forcer contre la monture
const int AMPLITUDE = 45;

// Pause entre deux pas de 1°, en millisecondes (plus grand = plus lent)
const int PAUSE_MS = 0;

Servo servoBas;
Servo servoHaut;

// Fait aller un servo de son centre au maximum, puis au minimum, puis au centre
void balayer(Servo &servo, const char *nom, int centre) {
  // Bornes du balayage, limitées à la course du servo (0° à 180°)
  int angleMin = constrain(centre - AMPLITUDE, 0, 180);
  int angleMax = constrain(centre + AMPLITUDE, 0, 180);

  Serial.print("Balayage du servo ");
  Serial.print(nom);
  Serial.print(" de ");
  Serial.print(angleMin);
  Serial.print(" à ");
  Serial.println(angleMax);

  for (int angle = centre; angle <= angleMax; angle++) {
    servo.write(angle);
    delay(PAUSE_MS);
  }
  for (int angle = angleMax; angle >= angleMin; angle--) {
    servo.write(angle);
    delay(PAUSE_MS);
  }
  for (int angle = angleMin; angle <= centre; angle++) {
    servo.write(angle);
    delay(PAUSE_MS);
  }
}

void setup() {
  // Messages lisibles dans le moniteur série (115200 bauds)
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

  // Départ : les deux servos au centre, 3 s pour vérifier que la caméra
  // regarde droit devant
  servoBas.write(CENTRE_BAS);
  servoHaut.write(CENTRE_HAUT);
  Serial.println("Servos au centre");
  delay(3000);
}

void loop() {
  balayer(servoBas, "du bas", CENTRE_BAS);
  delay(50);
  balayer(servoHaut, "du haut", CENTRE_HAUT);
  delay(50);
}
