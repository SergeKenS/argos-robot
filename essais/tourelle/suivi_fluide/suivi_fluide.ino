// suivi_fluide.ino
// Essai 4 de la tourelle, côté ESP32 : mouvement fluide, ordres sans file
// d'attente. Le PC envoie la cible ; l'ESP32 y amène la tourelle en douceur,
// 50 fois par seconde.
//
// Ce qui change par rapport à suivi_camera.ino (essai 3) :
//   1. Signal des servos produit par le MCPWM (circuit de l'ESP32 prévu pour
//      les moteurs) au lieu de la bibliothèque ESP32Servo. Mesuré avec
//      mesure_servo.ino : ESP32Servo bloque jusqu'à 40 ms par ordre, d'où la
//      limite de 25 ordres par seconde. Ici, changer la consigne est immédiat.
//      Précision : 1 µs d'impulsion, soit 0,1° (ESP32Servo : 1,85°).
//   2. Lecture série sans attente : chaque ordre reçu remplace le précédent.
//   3. Boucle à 50 Hz, calée sur les impulsions du servo : la tourelle rejoint
//      la cible avec une vitesse et une accélération limitées.
//   4. Renvoie sa position au PC pendant qu'elle bouge.
//   5. Rattrape le jeu des servos (mesuré : 2,5° en bas, 2,2° en haut) : quand
//      un servo change de sens, il ne bouge pas tant que la consigne n'a pas
//      reculé de la largeur de son jeu. L'ESP32 ajoute donc ce recul lui-même.
//
// Reçu (une ligne par ordre) :
//   "bas,haut"  cible en degrés, décimales permises (ex. "100.5,30")
//   "V,300"     vitesse maximale, en degrés par seconde
//   "A,2000"    accélération maximale, en degrés par seconde carrée
//   "G,15"      gain d'approche (plus grand = arrivée plus vive)
//   "J,2.4,2.1" jeu du servo du bas et du haut, en degrés (0 = pas de rattrapage)
// Envoyé : "P,temps_ms,bas,haut" à chaque changement de position.
// Test sans Python : moniteur série à 115200 bauds, taper "120,40" puis Entrée.
//
// Branchement : le même que balayage.ino
//   fils bruns                              -> GND
//   fils rouges                             -> VIN (5 V venant de l'USB)
//   orange du servo du bas  (gauche/droite) -> D18
//   orange du servo du haut (haut/bas)      -> D19

#include "driver/mcpwm_prelude.h"

// Broches de signal des servos
const int BROCHE_BAS = 18;
const int BROCHE_HAUT = 19;

// Centres mesurés sur la monture (caméra droit devant)
// CENTRE_BAS : 100 avant la fixation sur le châssis, 105 depuis (2026-11-14)
const float CENTRE_BAS = 105;
const float CENTRE_HAUT = 30;

// Écart maximal autorisé de part et d'autre du centre, pour protéger la monture
const float AMPLITUDE = 45;

// Signal d'un servo SG90 : une impulsion toutes les 20 ms (50 Hz), dont la
// largeur donne l'angle : 500 µs pour 0°, 2400 µs pour 180°
const uint32_t PERIODE_US = 20000;
const float IMPULSION_0_US = 500;
const float IMPULSION_180_US = 2400;

// La boucle calcule la nouvelle position 2 ms avant le départ de chaque
// impulsion : le servo la reçoit presque aussitôt
const uint32_t AVANCE_US = 2000;
// Durée d'un tour de boucle, en secondes
const float DT = PERIODE_US / 1e6;

// Réglages du mouvement (modifiables par le port série)
float vitesseMax = 300;        // degrés par seconde
float accelerationMax = 2000;  // degrés par seconde carrée
float gain = 15;               // vitesse d'approche par degré d'écart (1/s)

// Jeu de chaque servo, en degrés. Mesuré : 2,5 en bas, 2,2 en haut. Réglé un
// peu en dessous : trop de rattrapage ferait dépasser la cible.
const float JEU_BAS = 2.4;
const float JEU_HAUT = 2.1;
// Recul minimal (degrés) pour considérer que le mouvement a changé de sens :
// un tremblement plus petit ne déclenche pas de rattrapage
const float SEUIL_SENS = 0.1;

// Un axe de la tourelle : son signal et son mouvement
struct Axe {
  mcpwm_cmpr_handle_t comparateur;  // règle la largeur de l'impulsion
  uint32_t impulsion;               // largeur en cours, en microsecondes
  float centre;                     // degrés
  float cible;                      // où le PC veut aller (degrés)
  float position;                   // où la tourelle doit viser maintenant (degrés)
  float vitesse;                    // degrés par seconde
  float jeu;                        // jeu du servo (degrés)
  int sens;                         // sens du dernier déplacement : +1 ou -1
  float extreme;                    // position la plus avancée dans ce sens
};

Axe bas;
Axe haut;

// Minuterie commune aux deux servos, et heure du prochain tour de boucle (µs)
mcpwm_timer_handle_t minuterie;
unsigned long prochainTour;

// Ligne en cours de réception sur le port série
char ligne[32];
unsigned int longueur = 0;

// Garde un angle dans la plage autorisée autour du centre et dans 0° à 180°
float limiter(float angle, float centre) {
  float mini = max(centre - AMPLITUDE, 0.0f);
  float maxi = min(centre + AMPLITUDE, 180.0f);
  return constrain(angle, mini, maxi);
}

// Largeur d'impulsion (µs) qui correspond à un angle (degrés)
uint32_t versImpulsion(float angle) {
  angle = constrain(angle, 0.0f, 180.0f);
  return lroundf(IMPULSION_0_US + angle * (IMPULSION_180_US - IMPULSION_0_US) / 180);
}

// Angle à donner au servo pour que la tourelle vise la position voulue :
// décalé d'un demi-jeu dans le sens du mouvement
float consigneServo(Axe &axe) {
  return axe.position + axe.sens * axe.jeu / 2;
}

// Prépare le signal d'un servo sur une broche, au centre
void preparerAxe(Axe &axe, int broche, float centre, float jeu) {
  axe.centre = centre;
  axe.cible = centre;
  axe.position = centre;
  axe.vitesse = 0;
  axe.jeu = jeu;
  axe.sens = 1;
  axe.extreme = centre;
  axe.impulsion = versImpulsion(consigneServo(axe));

  // Opérateur : relie la minuterie à une sortie
  mcpwm_operator_config_t configOperateur = {};
  configOperateur.group_id = 0;
  mcpwm_oper_handle_t operateur;
  ESP_ERROR_CHECK(mcpwm_new_operator(&configOperateur, &operateur));
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operateur, minuterie));

  // Comparateur : l'instant où l'impulsion se termine. Une nouvelle valeur
  // n'est prise en compte qu'au début de la période suivante : jamais
  // d'impulsion coupée en deux.
  mcpwm_comparator_config_t configComparateur = {};
  configComparateur.flags.update_cmp_on_tez = true;
  ESP_ERROR_CHECK(mcpwm_new_comparator(operateur, &configComparateur, &axe.comparateur));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(axe.comparateur, axe.impulsion));

  // Générateur : la broche passe à 1 au début de la période, puis à 0 quand
  // la minuterie atteint le comparateur
  mcpwm_generator_config_t configGenerateur = {};
  configGenerateur.gen_gpio_num = broche;
  mcpwm_gen_handle_t generateur;
  ESP_ERROR_CHECK(mcpwm_new_generator(operateur, &configGenerateur, &generateur));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
    generateur,
    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
    generateur,
    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, axe.comparateur, MCPWM_GEN_ACTION_LOW)));
}

// Avance un axe d'un tour de boucle (20 ms) vers sa cible.
// Renvoie true si la consigne du servo a changé.
bool avancer(Axe &axe) {
  float ecart = axe.cible - axe.position;
  float distance = fabsf(ecart);

  if (distance < 0.02 && fabsf(axe.vitesse) < 1) {
    // Arrivé : on se pose exactement sur la cible
    axe.position = axe.cible;
    axe.vitesse = 0;
  } else {
    // Vitesse voulue. Près de la cible : proportionnelle à l'écart (arrivée
    // en douceur). Plus loin : la plus grande vitesse d'où l'on peut encore
    // freiner à temps. Jamais plus que vitesseMax.
    float zoneDouce = accelerationMax / (gain * gain);
    float voulue;
    if (distance <= zoneDouce) {
      voulue = gain * distance;
    } else {
      voulue = sqrtf(2 * accelerationMax * (distance - zoneDouce / 2));
    }
    voulue = min(voulue, vitesseMax);
    if (ecart < 0) {
      voulue = -voulue;
    }

    // La vitesse ne change pas plus vite que l'accélération permise
    float pas = accelerationMax * DT;
    axe.vitesse += constrain(voulue - axe.vitesse, -pas, pas);
    axe.position += axe.vitesse * DT;
  }

  // Sens du mouvement, pour le rattrapage du jeu : il ne change que si la
  // position recule d'au moins SEUIL_SENS depuis son point le plus avancé
  if ((axe.position - axe.extreme) * axe.sens > 0) {
    axe.extreme = axe.position;
  } else if ((axe.extreme - axe.position) * axe.sens > SEUIL_SENS) {
    axe.sens = -axe.sens;
    axe.extreme = axe.position;
  }

  uint32_t impulsion = versImpulsion(consigneServo(axe));
  if (impulsion == axe.impulsion) {
    return false;
  }
  axe.impulsion = impulsion;
  mcpwm_comparator_set_compare_value(axe.comparateur, impulsion);
  return true;
}

// Applique une ligne reçue : une cible "bas,haut" ou un réglage "lettre,valeur"
void traiterLigne() {
  float a, b;
  char lettre;
  if (sscanf(ligne, "%f,%f", &a, &b) == 2) {
    if (isfinite(a) && isfinite(b)) {
      bas.cible = limiter(a, bas.centre);
      haut.cible = limiter(b, haut.centre);
    }
  } else {
    int lus = sscanf(ligne, "%c,%f,%f", &lettre, &a, &b);
    if (lus < 2 || !isfinite(a)) {
      return;
    }
    if (lettre == 'V') {
      vitesseMax = constrain(a, 10.0f, 600.0f);
    } else if (lettre == 'A') {
      accelerationMax = constrain(a, 100.0f, 20000.0f);
    } else if (lettre == 'G') {
      gain = constrain(a, 1.0f, 40.0f);
    } else if (lettre == 'J' && lus == 3 && isfinite(b)) {
      bas.jeu = constrain(a, 0.0f, 5.0f);
      haut.jeu = constrain(b, 0.0f, 5.0f);
    }
    Serial.printf("Réglages : V %.0f, A %.0f, G %.1f, J %.2f %.2f\n",
                  vitesseMax, accelerationMax, gain, bas.jeu, haut.jeu);
  }
}

// Lit ce qui est arrivé sur le port série, sans jamais attendre. S'il y a
// plusieurs ordres en attente, ils sont tous lus et seul le dernier reste.
void lireSerie() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      ligne[longueur] = '\0';
      traiterLigne();
      longueur = 0;
    } else if (longueur < sizeof(ligne) - 1) {
      ligne[longueur] = c;
      longueur++;
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Minuterie : compte les microsecondes de 0 à 20 000, puis recommence
  mcpwm_timer_config_t configMinuterie = {};
  configMinuterie.group_id = 0;
  configMinuterie.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  configMinuterie.resolution_hz = 1000000;
  configMinuterie.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
  configMinuterie.period_ticks = PERIODE_US;
  ESP_ERROR_CHECK(mcpwm_new_timer(&configMinuterie, &minuterie));

  preparerAxe(bas, BROCHE_BAS, CENTRE_BAS, JEU_BAS);
  preparerAxe(haut, BROCHE_HAUT, CENTRE_HAUT, JEU_HAUT);

  // Départ du signal. Les impulsions partent ensuite toutes les 20 ms : la
  // boucle se cale dessus, AVANCE_US avant chacune.
  ESP_ERROR_CHECK(mcpwm_timer_enable(minuterie));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(minuterie, MCPWM_TIMER_START_NO_STOP));
  prochainTour = micros() + PERIODE_US - AVANCE_US;

  Serial.println("Prêt : envoyer bas,haut (ex. 100,30)");
}

void loop() {
  lireSerie();

  // Un tour toutes les 20 ms, juste avant l'impulsion suivante
  if ((long)(micros() - prochainTour) < 0) {
    return;
  }
  prochainTour += PERIODE_US;

  bool basBouge = avancer(bas);
  bool hautBouge = avancer(haut);
  if (basBouge || hautBouge) {
    Serial.printf("P,%lu,%.2f,%.2f\n", millis(), bas.position, haut.position);
  }
}
