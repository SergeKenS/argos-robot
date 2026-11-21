# Essai de l'application, mode Manuel (2026-11-20)

But : conduire le corps et la tête depuis la page (PC, téléphone), avant d'écrire
le vrai rôle Manuel côté robot.

## Lancer (sur le Pi)

```
cd ~/Documents/Projet/argos_ws && source install/setup.bash
ros2 launch argos_web argos_web.launch.py &
ros2 run argos_base pont_moteurs --ros-args --params-file install/argos_base/share/argos_base/config/base.yaml &
python3 essais/appli/banc_manuel.py
```
Page : http://192.168.2.78:8080/#manuel (l'adresse IP plutôt que `argos-pi.local`,
voir problème 4). Le répartiteur ne doit pas tourner en même temps.

`banc_manuel.py` est **provisoire** : il joue le futur rôle Manuel, le nœud tourelle
et le nœud caméra. Il sert de référence pour les écrire, puis sera supprimé.

## Résultats

| Essai | Mesure |
|---|---|
| Avancer 1 s (clavier W) | 22 cm |
| Pivoter 0,7 s (clavier A) | 27° |
| Aligner, tête à 45° | corps tourné de 45° en 1,1 à 1,7 s, regard fixe, hauteur de la tête inchangée |
| Départ « plein avant » avec la rampe | 0,22 m/s à 0,5 s, 0,36 m/s à 1,1 s, dépassement d'environ 15 % au relâché |
| Touche lâchée (roue libre) | de 0,45 m/s à l'arrêt en 0,84 s, sur 20 cm |
| STOP (freinage) | de 0,40 m/s à l'arrêt en 0,47 s, sur 11 cm |
| Bouton opposé tenu | freine jusqu'à 0 en 0,5 s, puis repart dans l'autre sens |

## Problèmes rencontrés

1. **L'image de la caméra n'apparaissait pas dans la page.**
   - Symptôme : « SANS IMAGE », alors que le flux s'affichait sur sa propre page (port 8081).
   - Diagnostic : messages de Chrome lus par le protocole DevTools : réponse sans
     en-tête CORS pour `/flux?t=...`, alors que `curl /flux` en avait un.
   - Cause : le serveur vidéo ne reconnaissait que `/flux` exactement ; avec un
     paramètre, il renvoyait sa page HTML au lieu des images.
   - Solution : `flux_video.py` ignore le paramètre et envoie l'en-tête CORS.
   - Fausse piste : la règle « ORB » de Chrome, soupçonnée d'abord.
2. **Pic de vitesse au démarrage (0,35 m/s pour 0,28 demandé).**
   - Cause : le joystick passe de 0 au maximum d'un coup, et le régulateur de la
     carte (réglé nerveux, problème 7 des moteurs) dépasse sur un ordre brutal.
   - Solution : rampe d'accélération dans `pont_moteurs` (0,4 m/s²), pour toutes
     les sources d'ordres. Vitesse maximale portée à 0,70 m/s (limite mesurée).
3. **Recentrer ne faisait rien, et Aligner semblait recentrer la tête.**
   - Cause : oubli dans le banc. La consigne passait au centre, mais l'ordre ne
     partait vers la tourelle qu'au mouvement suivant (donc pendant Aligner).
   - Leçon : le vrai rôle aura un test automatique pour chaque bouton.
4. **`argos-pi.local` met 11 s à répondre sous Windows**, à chaque connexion.
   - Contournement : utiliser l'adresse IP. Solution durable : adresse fixe
     donnée par la box au Pi.

## Comportement retenu (à reprendre dans le vrai rôle)

- Touche ou joystick tenu : le robot accélère peu à peu jusqu'à 0,70 m/s.
- Lâché : la page n'envoie plus rien et le robot passe en roue libre au bout de 0,3 s.
- STOP ou sens opposé : la page envoie « vitesse 0 » ou l'autre sens, et le robot freine.
- Recentrer : la tête revient droit devant, le corps ne bouge pas.
- Aligner : le corps pivote vers le regard, la tête tourne en sens inverse (regard
  fixe) et finit au centre en largeur ; sa hauteur ne change pas.
