// mesure_servo.ino
// Mesure : combien de temps dure servo.write() sur l'ESP32 ?
// But : vérifier d'où vient la limite d'environ 25 ordres par seconde de
// suivi_camera.ino. Hypothèse (lue dans le pilote PWM de l'ESP32) : write()
// attend que la consigne précédente soit prise en compte par le signal 50 Hz.
//
// Aucun échange série pendant la mesure : si write() est lent ici, le port
// série n'y est pour rien.
// La tourelle bouge à peine : 2° autour du centre.
// Résultats sur le port série (115200 bauds). Envoyer un caractère pour
// refaire la mesure.
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

// Nombre d'ordres par essai
const int REPETITIONS = 50;

Servo servoBas;
Servo servoHaut;

// Envoie REPETITIONS ordres et affiche la durée d'un ordre (moyenne, min, max).
// deuxServos : écrit aussi le servo du haut, comme le fait suivi_camera.ino.
// pauseMs : attente entre deux ordres (0 = le plus vite possible).
void mesurer(const char *nom, bool deuxServos, int pauseMs) {
  unsigned long total = 0, mini = 0xFFFFFFFF, maxi = 0;
  for (int i = 0; i < REPETITIONS; i++) {
    int decalage = (i % 2) * 2;  // alterne centre et centre + 2°
    unsigned long debut = micros();
    servoBas.write(CENTRE_BAS + decalage);
    if (deuxServos) {
      servoHaut.write(CENTRE_HAUT + decalage);
    }
    unsigned long duree = micros() - debut;
    total += duree;
    mini = min(mini, duree);
    maxi = max(maxi, duree);
    delay(pauseMs);
  }
  Serial.printf("%s : moyenne %lu us, min %lu us, max %lu us\n",
                nom, total / REPETITIONS, mini, maxi);
  // Laisse la dernière consigne être prise en compte avant l'essai suivant
  delay(200);
}

void toutMesurer() {
  Serial.println("Debut des mesures");
  mesurer("1 servo, sans pause", false, 0);
  mesurer("2 servos, sans pause", true, 0);
  mesurer("2 servos, pause 20 ms", true, 20);
  mesurer("2 servos, pause 40 ms", true, 40);
  mesurer("2 servos, pause 60 ms", true, 60);
  Serial.println("Fin des mesures");
}

void setup() {
  Serial.begin(115200);

  // Même préparation des servos que suivi_camera.ino
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoBas.setPeriodHertz(50);
  servoHaut.setPeriodHertz(50);
  servoBas.attach(BROCHE_BAS, 500, 2400);
  servoHaut.attach(BROCHE_HAUT, 500, 2400);

  // Départ au centre, puis 1 s pour que la tourelle soit immobile
  servoBas.write(CENTRE_BAS);
  servoHaut.write(CENTRE_HAUT);
  delay(1000);

  toutMesurer();
}

void loop() {
  // Un caractère reçu : on refait la mesure
  if (Serial.available() > 0) {
    while (Serial.available() > 0) {
      Serial.read();
    }
    toutMesurer();
  }
}
