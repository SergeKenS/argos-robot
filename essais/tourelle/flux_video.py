"""Diffuse l'image de la caméra en direct dans un navigateur (flux MJPEG).

Un seul programme peut ouvrir la caméra : c'est donc le programme de suivi qui
partage ses images, déjà annotées. Il appelle flux.publier(image) à chaque image.

Regarder : http://argos-pi.local:8081 (PC ou téléphone, sur le même réseau).

MJPEG = une suite d'images JPEG envoyées l'une après l'autre dans la même
réponse HTTP. Tous les navigateurs l'affichent sans rien installer.
Solution d'essai : sur le vrai robot, l'image passera par ROS 2 (voir ETAPES.md).
"""
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

PAGE = b"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Camera Argos</title></head>
<body style="margin:0;background:#111;display:flex;justify-content:center">
<img src="/flux" style="max-width:100%;height:auto"></body></html>"""


class FluxVideo:
    """Petit serveur web qui envoie la dernière image publiée à chaque spectateur."""

    def __init__(self, port=8081, images_par_seconde=15, qualite=70):
        """Démarre le serveur dans un fil d'exécution à part."""
        self.intervalle = 1 / images_par_seconde
        self.qualite = qualite
        self.jpeg = None             # dernière image, déjà compressée
        self.numero = 0              # augmente à chaque nouvelle image
        self.derniere = 0.0          # heure de la dernière compression
        self.condition = threading.Condition()
        self.actif = True

        flux = self

        class Requete(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == '/flux':
                    flux.envoyer(self)
                else:
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.end_headers()
                    self.wfile.write(PAGE)

            def log_message(self, *arguments):
                pass  # pas une ligne dans le terminal à chaque requête

        self.serveur = ThreadingHTTPServer(('', port), Requete)
        self.serveur.daemon_threads = True
        threading.Thread(target=self.serveur.serve_forever, daemon=True).start()
        print(f'Image en direct : http://<adresse du robot>:{port}')

    def publier(self, image):
        """Garde l'image pour les spectateurs. Limité à images_par_seconde :
        compresser chaque image coûterait du temps au suivi pour rien."""
        maintenant = time.perf_counter()
        if maintenant - self.derniere < self.intervalle:
            return
        self.derniere = maintenant
        ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, self.qualite])
        if ok:
            with self.condition:
                self.jpeg, self.numero = jpeg.tobytes(), self.numero + 1
                self.condition.notify_all()

    def envoyer(self, requete):
        """Envoie les images à un spectateur, jusqu'à ce qu'il ferme la page."""
        requete.send_response(200)
        requete.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=image')
        requete.send_header('Cache-Control', 'no-cache')
        requete.end_headers()
        vu = 0
        try:
            while self.actif:
                with self.condition:
                    # Attendre une image plus récente que la dernière envoyée
                    self.condition.wait_for(lambda: self.numero != vu or not self.actif, timeout=1)
                    if self.numero == vu:
                        continue
                    jpeg, vu = self.jpeg, self.numero
                requete.wfile.write(b'--image\r\nContent-Type: image/jpeg\r\n'
                                    b'Content-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n')
                requete.wfile.write(jpeg + b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            pass  # le spectateur a fermé la page

    def fermer(self):
        """Arrête le serveur et libère les spectateurs."""
        self.actif = False
        with self.condition:
            self.condition.notify_all()
        self.serveur.shutdown()
        self.serveur.server_close()
