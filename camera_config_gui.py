"""
Configuration Caméra — GUI
--------------------------
Interface PyQt6 pour configurer et capturer des images depuis le STM32N6
via UART.

Nouveautés :
  - Connexion automatique (polling) sur la carte ST (VID USB 0x0483)
  - Moniteur série : tous les printf du STM32 sont affichés dans le journal
  - Fenêtre toujours carrée (rapport 1:1) mais redimensionnable
  - Enchaînement imposé : Capturer → Tester → Envoyer
  - Envoi de la date/heure courante au µC (commande 'T') avant la config
  - Thème clair, interface en français

Dépendances : PyQt6, numpy, Pillow, pyserial
Usage       : python camera_config_gui.py
"""

import sys
import time
import queue
import struct
import serial
import serial.tools.list_ports
import numpy as np
from io import BytesIO
from datetime import datetime

from PIL import Image

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QLineEdit, QPushButton,
    QFrame, QGroupBox, QSizePolicy, QTextEdit, QStackedWidget,
    QGraphicsView, QGraphicsScene, QGraphicsRectItem, QGraphicsPixmapItem,
    QGraphicsLineItem, QGraphicsSimpleTextItem, QGraphicsItem,
    QButtonGroup, QCheckBox,
)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer, QRectF
from PyQt6.QtGui import QFont, QIntValidator, QPixmap, QPen, QBrush, QColor, QImage, QPainter

# =============================================================================
#  Constantes
# =============================================================================

MAGIC          = 0x12345678
UART_BAUDRATE  = 10_000_000
ST_VID         = 0x0483      # VID USB STMicroelectronics (ST-Link VCP)
POLL_MS        = 1500        # période de polling des ports série
BG             = "#f4f6fa"   # fond clair
FG             = "#2a3442"
MAX_DOWNSIZE   = 7.99   # downsize_ratio max (jamais 8 exactement)
HANDLE_PX      = 9      # taille visuelle d'une poignée de redimensionnement (px écran)
MIN_CROP_PX    = 4      # taille mini d'un côté de zone de crop (px image)
AXIS_TICK_STEP        = 100  # pas de graduation, en pixels image
# Marges réservées à la graduation/au titre, en pixels ÉCRAN visés (comme le
# texte lui-même via ItemIgnoresTransformations) -- converties en unités de
# scène (~ pixels image) selon le zoom courant dans set_image()/_layout_axes().
# En pixels image fixes, elles deviendraient négligeables une fois l'image
# (2592x1944) réduite à l'affichage, et la photo finirait par occuper
# quasiment tout le viewport au lieu de laisser une marge lisible.
AXIS_LEFT_MARGIN_PX   = 42
AXIS_TOP_MARGIN_PX    = 26
AXIS_BOTTOM_MARGIN_PX = 20

# =============================================================================
#  Calcul decimation pipe 2
# =============================================================================

def compute_pipe2_params(block_size):
    """
    Cherche le plus petit decimation_ratio dans {1,2,4,8} tel que
    downsize = block_size / decimation <= 8.
    => plus petit dec => plus grand downsize <= 8 (maximise downsize).
    Retourne (decimation_ratio, downsize_ratio) ou (None, None) si impossible.
    """
    for dec in [1, 2, 4, 8]:
        downsize = block_size / dec
        if downsize <= MAX_DOWNSIZE:
            return dec, downsize
    return None, None

# =============================================================================
#  Widget d'affichage (placeholder avant la première capture)
# =============================================================================
#  Au-delà de la première capture, InteractiveCropView est la SEULE vue
#  utilisée, config appliquée ou non -- les rectangles de crop sont juste
#  ajoutés/retirés de sa scène. La graduation/le titre sont donc toujours
#  rendus par exactement le même code, dans les deux cas : aucun risque de
#  divergence de style/position entre "appliqué" et "retiré".

def make_placeholder_label():
    label = QLabel("Aucune image capturée")
    label.setAlignment(Qt.AlignmentFlag.AlignCenter)
    label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
    label.setMinimumSize(400, 300)
    label.setStyleSheet(
        f"background-color: {BG}; border: 1px solid #b8c4d4;"
        "color: #8a96a8; font-size: 13px; font-family: 'Segoe UI';"
    )
    return label

# =============================================================================
#  Zone de crop interactive (redimensionnable à la souris, façon Word)
# =============================================================================

class HandleRectItem(QGraphicsRectItem):
    """Rectangle de crop déplaçable/redimensionnable à la souris : coins et
    côtés agissent comme des poignées de redimensionnement, l'intérieur
    déplace tout le rectangle. Les coordonnées du rect() sont directement en
    pixels image (l'item n'a ni rotation ni décalage de position — seule la
    QGraphicsView applique un facteur d'échelle à l'affichage)."""

    _CURSORS = {
        'nw': Qt.CursorShape.SizeFDiagCursor, 'se': Qt.CursorShape.SizeFDiagCursor,
        'ne': Qt.CursorShape.SizeBDiagCursor, 'sw': Qt.CursorShape.SizeBDiagCursor,
        'n':  Qt.CursorShape.SizeVerCursor,   's':  Qt.CursorShape.SizeVerCursor,
        'e':  Qt.CursorShape.SizeHorCursor,   'w':  Qt.CursorShape.SizeHorCursor,
        'move': Qt.CursorShape.OpenHandCursor,
    }

    def __init__(self, rect, color, img_w, img_h, on_change):
        super().__init__(rect)
        self._img_w      = img_w
        self._img_h      = img_h
        self.on_change   = on_change     # callback(top, bottom, left, right)
        self._mode       = None
        self._drag_start = None
        self._rect_start = None
        self._active     = True

        pen = QPen(QColor(color)); pen.setWidth(2); pen.setCosmetic(True)
        self.setPen(pen)
        self.setBrush(QBrush(QColor(color)))
        self.setOpacity(0.30)
        self.setAcceptHoverEvents(True)
        self.setCursor(Qt.CursorShape.OpenHandCursor)

    def set_active(self, active):
        """Seule la zone active répond à la souris — évite qu'une zone
        cachée sous l'autre (chevauchement) ne vole les clics destinés à
        celle du dessous.
        No-op si l'état ne change pas : set_rect() appelle ceci à chaque
        mise à jour (y compris celles déclenchées par le glisser-déposer
        lui-même, via le sync champs<->rect) -- sans ce garde-fou, un
        déplacement en cours se voyait couper après le premier mouvement
        (_mode remis à None en plein glisser)."""
        if self._active == active:
            return
        self._active = active
        self._mode = None
        self.setCursor(Qt.CursorShape.OpenHandCursor if active else Qt.CursorShape.ArrowCursor)

    # ── détection de poignée ────────────────────────────────────────────────
    def _handle_size(self):
        views = self.scene().views() if self.scene() else []
        scale = views[0].transform().m11() if views else 1.0
        if scale <= 0:
            scale = 1.0
        return max(HANDLE_PX / scale, MIN_CROP_PX)

    def _zone_at(self, pos):
        r = self.rect()
        h = self._handle_size()
        near_top    = abs(pos.y() - r.top())    <= h
        near_bottom = abs(pos.y() - r.bottom()) <= h
        near_left   = abs(pos.x() - r.left())   <= h
        near_right  = abs(pos.x() - r.right())  <= h
        if near_top and near_left:     return 'nw'
        if near_top and near_right:    return 'ne'
        if near_bottom and near_left:  return 'sw'
        if near_bottom and near_right: return 'se'
        if near_top:    return 'n'
        if near_bottom: return 's'
        if near_left:   return 'w'
        if near_right:  return 'e'
        if r.contains(pos):
            return 'move'
        return None

    # ── interaction souris ───────────────────────────────────────────────────
    def hoverMoveEvent(self, event):
        if not self._active:
            return
        zone = self._zone_at(event.pos())
        self.setCursor(self._CURSORS.get(zone, Qt.CursorShape.ArrowCursor))
        super().hoverMoveEvent(event)

    def mousePressEvent(self, event):
        if not self._active:
            # Laisse l'évènement descendre à la zone du dessous (active).
            event.ignore()
            return
        self._mode       = self._zone_at(event.pos())
        self._drag_start = event.pos()
        self._rect_start = QRectF(self.rect())
        if self._mode == 'move':
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
        event.accept()

    def mouseMoveEvent(self, event):
        if self._mode is None:
            return
        d = event.pos() - self._drag_start
        r = QRectF(self._rect_start)

        if self._mode == 'move':
            r.translate(d.x(), d.y())
            r = self._clamp_move(r)
        else:
            # Chaque bord déplacé est borné par rapport au bord OPPOSÉ (qui,
            # lui, reste fixe) et par les limites de l'image — un simple
            # clamp global sur le rect final ferait bouger le bord fixe au
            # lieu de simplement arrêter le bord tiré.
            if 'n' in self._mode:
                top = r.top() + d.y()
                r.setTop(max(0, min(top, r.bottom() - MIN_CROP_PX)))
            if 's' in self._mode:
                bottom = r.bottom() + d.y()
                r.setBottom(min(self._img_h, max(bottom, r.top() + MIN_CROP_PX)))
            if 'w' in self._mode:
                left = r.left() + d.x()
                r.setLeft(max(0, min(left, r.right() - MIN_CROP_PX)))
            if 'e' in self._mode:
                right = r.right() + d.x()
                r.setRight(min(self._img_w, max(right, r.left() + MIN_CROP_PX)))

        self.setRect(r)
        if self.on_change:
            self.on_change(int(round(r.top())), int(round(r.bottom())),
                            int(round(r.left())), int(round(r.right())))

    def mouseReleaseEvent(self, event):
        self._mode = None
        self.setCursor(Qt.CursorShape.OpenHandCursor)

    def _clamp_move(self, r):
        """Déplacement pur : la taille ne change pas, seule la position est
        bornée pour rester dans l'image."""
        w, h = r.width(), r.height()
        left = min(max(r.left(), 0), max(self._img_w - w, 0))
        top  = min(max(r.top(),  0), max(self._img_h - h, 0))
        return QRectF(left, top, w, h)


class InteractiveCropView(QGraphicsView):
    """Affiche l'image capturée et superpose les zones pipe1/pipe2
    redimensionnables (HandleRectItem)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setStyleSheet(f"background-color: {BG}; border: 1px solid #b8c4d4;")
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setMinimumSize(400, 300)
        self._pix_item  = None
        self._rect_items = {}
        self._axis_items = []
        self._img_w = self._img_h = 0
        self._left_margin = self._top_margin = self._bottom_margin = 0
        self._active_key = 'p1'
        # Aperçu "résolution réduite" (pixellisation réelle de la zone,
        # simulant decimation+downsize) -- voir update_pixel_preview().
        self._img_np = None
        self._pixelate_enabled = False
        self._pixel_items = {}
        self._pending_block_sizes = {}

    def set_image(self, img_np):
        h, w = img_np.shape[:2]
        self._img_np = np.ascontiguousarray(img_np)
        qimg = QImage(self._img_np.data, w, h, w * 3, QImage.Format.Format_RGB888).copy()
        pix = QPixmap.fromImage(qimg)
        if self._pix_item is None:
            self._pix_item = QGraphicsPixmapItem(pix)
            self._scene.addItem(self._pix_item)
        else:
            self._pix_item.setPixmap(pix)
        self._img_w, self._img_h = w, h
        self._layout_axes()
        self._fit()

    def _layout_axes(self):
        """(Re)calcule les marges (en unités de scène) pour qu'elles
        occupent la taille écran visée (AXIS_*_MARGIN_PX), puis redessine
        la graduation/le titre avec ces marges. Appelé à chaque nouvelle
        image et à chaque redimensionnement de la vue -- le zoom fitInView
        change avec la taille du widget, donc la conversion écran->scène
        aussi."""
        if self._img_w <= 0:
            return
        approx_scale = self.viewport().width() / self._img_w
        if approx_scale <= 0:
            approx_scale = 1.0
        self._left_margin   = AXIS_LEFT_MARGIN_PX   / approx_scale
        self._top_margin    = AXIS_TOP_MARGIN_PX    / approx_scale
        self._bottom_margin = AXIS_BOTTOM_MARGIN_PX / approx_scale
        self._scene.setSceneRect(
            -self._left_margin, -self._top_margin,
            self._img_w + self._left_margin,
            self._img_h + self._top_margin + self._bottom_margin)
        self._draw_axes(self._img_w, self._img_h)

    def _draw_axes(self, w, h):
        """Graduation en pixels + titre -- toujours affichés, config
        appliquée ou non, par exactement ce même code (donc jamais de
        divergence de style/position entre les deux), à taille d'écran
        constante (ItemIgnoresTransformations). Dessinés en dehors de
        l'image (marges négatives/en bas) donc sans jamais recouvrir la
        photo ni les zones de crop."""
        for it in self._axis_items:
            self._scene.removeItem(it)
        self._axis_items = []

        tick_pen = QPen(QColor('#b8c4d4')); tick_pen.setWidth(1); tick_pen.setCosmetic(True)
        label_font = QFont("Segoe UI", 7)
        label_brush = QBrush(QColor('#5a6a80'))

        def add_line(x1, y1, x2, y2):
            line = QGraphicsLineItem(x1, y1, x2, y2)
            line.setPen(tick_pen)
            self._scene.addItem(line)
            self._axis_items.append(line)

        def add_text(x, y, text, font=label_font, brush=label_brush):
            item = QGraphicsSimpleTextItem(text)
            item.setFont(font)
            item.setBrush(brush)
            # Sans ce flag, la taille du texte suit le zoom fitInView de la
            # vue (image 2592px compressée dans le widget) et devient
            # illisible -- avec, il garde toujours sa taille écran réelle.
            item.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations, True)
            item.setPos(x, y)
            self._scene.addItem(item)
            self._axis_items.append(item)

        x = 0
        while x < w:
            add_line(x, h, x, h + 5)
            add_text(x - 10, h + 6, str(x))
            x += AXIS_TICK_STEP

        y = 0
        while y < h:
            add_line(-5, y, 0, y)
            add_text(-self._left_margin + 4, y - 6, str(y))
            y += AXIS_TICK_STEP

        title_font  = QFont("Segoe UI", 9)
        title_brush = QBrush(QColor(FG))
        title = QGraphicsSimpleTextItem("Capture — graduation en pixels")
        title.setFont(title_font)
        title.setBrush(title_brush)
        title.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations, True)
        title.setPos(w / 2 - title.boundingRect().width() / 2, -self._top_margin + 4)
        self._scene.addItem(title)
        self._axis_items.append(title)

    def set_rect(self, key, top, bottom, left, right, color, on_change):
        """Crée ou repositionne la zone `key` ('p1'/'p2') sur les coordonnées
        données — toujours utilisées comme coordonnées de base (comme le
        faisait l'ancien aperçu statique)."""
        rect = QRectF(left, top, right - left, bottom - top)
        item = self._rect_items.get(key)
        if item is None:
            item = HandleRectItem(rect, color, self._img_w, self._img_h, on_change)
            self._scene.addItem(item)
            self._rect_items[key] = item
        else:
            item._img_w, item._img_h = self._img_w, self._img_h
            item.on_change = on_change
            item.setRect(rect)
        # zValue > 1 (au-dessus de l'aperçu pixellisé, zValue 1, lui-même
        # au-dessus de l'image de base, zValue 0 implicite) : le remplissage
        # translucide du rectangle reste visible par-dessus la pixellisation.
        item.setZValue(3 if key == self._active_key else 2)
        item.set_active(key == self._active_key)

    def set_active_pipe(self, key):
        """Seule la zone `key` reste modifiable à la souris — évite qu'une
        zone chevauchante ne bloque l'accès aux poignées de l'autre."""
        self._active_key = key
        for k, item in self._rect_items.items():
            item.setZValue(3 if k == key else 2)
            item.set_active(k == key)

    def clear_rects(self):
        for item in self._rect_items.values():
            self._scene.removeItem(item)
        self._rect_items = {}
        for item in self._pixel_items.values():
            self._scene.removeItem(item)
        self._pixel_items = {}
        self._pending_block_sizes = {}

    # ── Aperçu "résolution réduite" ─────────────────────────────────────────

    def set_pixelate_enabled(self, enabled):
        """Bascule l'aperçu pixellisé pour toutes les zones déjà posées,
        en réutilisant le dernier facteur connu pour chacune."""
        self._pixelate_enabled = enabled
        if not enabled:
            for item in self._pixel_items.values():
                self._scene.removeItem(item)
            self._pixel_items = {}
            return
        for key, block_size in self._pending_block_sizes.items():
            if key in self._rect_items and block_size:
                self._render_pixel_preview(key, block_size)

    def update_pixel_preview(self, key, block_size):
        """Appelé à chaque changement pertinent (zone déplacée/redimensionnée,
        taille de bloc modifiée) -- ne redessine que si l'aperçu est activé,
        mais mémorise toujours le facteur pour un futur set_pixelate_enabled(True)."""
        self._pending_block_sizes[key] = block_size
        if self._pixelate_enabled:
            self._render_pixel_preview(key, block_size)

    def _render_pixel_preview(self, key, block_size):
        item = self._rect_items.get(key)
        if item is None or self._img_np is None or not block_size or block_size < 1:
            return

        r = item.rect()
        left = max(0, min(int(r.left()), self._img_w - 1))
        top  = max(0, min(int(r.top()),  self._img_h - 1))
        w = max(1, min(int(r.width()),  self._img_w - left))
        h = max(1, min(int(r.height()), self._img_h - top))

        # Downscale (moyennage, comme le downsize matériel) puis upscale au
        # plus proche voisin (bloc plein, comme l'affichage d'un pixel de
        # sortie) : simule fidèlement la perte de détail, pas une imitation.
        sub = self._img_np[top:top + h, left:left + w]
        small_w = max(1, round(w / block_size))
        small_h = max(1, round(h / block_size))
        blocky = (Image.fromarray(sub)
                  .resize((small_w, small_h), Image.BOX)
                  .resize((w, h), Image.NEAREST))
        arr = np.ascontiguousarray(np.array(blocky, dtype=np.uint8))
        qimg = QImage(arr.data, w, h, w * 3, QImage.Format.Format_RGB888).copy()

        pix_item = self._pixel_items.get(key)
        if pix_item is None:
            pix_item = QGraphicsPixmapItem(QPixmap.fromImage(qimg))
            pix_item.setZValue(1)
            self._scene.addItem(pix_item)
            self._pixel_items[key] = pix_item
        else:
            pix_item.setPixmap(QPixmap.fromImage(qimg))
        pix_item.setPos(left, top)

    def _fit(self):
        if self._pix_item is not None:
            self.fitInView(self._scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    def resizeEvent(self, event):
        # Le facteur d'échelle change avec la taille du widget -- sans ce
        # recalcul, les marges de graduation (dimensionnées pour une taille
        # écran constante) redeviennent fausses après un redimensionnement.
        self._layout_axes()
        self._fit()
        super().resizeEvent(event)

# =============================================================================
#  Thread série unique : SEUL propriétaire du port COM
# =============================================================================
#  Le port est ouvert une seule fois (à la connexion) et lu en continu : tous
#  les printf du STM sont journalisés sans interruption.  Les boutons n'ouvrent
#  jamais le port ; ils déposent une commande (capture / config) dans une file
#  que ce thread exécute, puis il reprend la lecture.  Plus aucune fermeture/
#  réouverture => plus aucun printf perdu.

class SerialWorker(QThread):
    line_received  = pyqtSignal(str)           # ligne printf du STM (journal)
    image_received = pyqtSignal(object, str)   # numpy img, description (capture 'S' à la demande)
    movement_snapshot_received = pyqtSignal(object, str)  # idem, mais poussé
                                                # sans demande par RECORD_MODE_INIT
                                                # (mouvement détecté côté µC)
    capture_error  = pyqtSignal(str)
    config_result  = pyqtSignal(bool, str)     # success, message
    status         = pyqtSignal(str)
    port_opened    = pyqtSignal(bool)          # True = port ouvert

    def __init__(self, port):
        super().__init__()
        self.port     = port
        self._cmd_q   = queue.Queue()
        self._running = True

    # ── API appelée depuis le thread GUI ────────────────────────────────────
    def request_capture(self):
        self._cmd_q.put(('capture', None))

    def request_config(self, data_bytes):
        self._cmd_q.put(('config', data_bytes))

    def stop(self):
        self._running = False
        self.wait(2500)

    # ── Boucle du thread ────────────────────────────────────────────────────
    def run(self):
        try:
            ser = serial.Serial(self.port, UART_BAUDRATE, timeout=0.2)
        except serial.SerialException as e:
            self.line_received.emit(f"[monitor] cannot open port: {e}")
            self.port_opened.emit(False)
            return
        self.port_opened.emit(True)

        line         = bytearray()   # ligne printf en cours de reconstruction
        awaiting_ack = False         # attente de l'ack config
        saw_fail     = False         # au moins un 'F' reçu pendant l'attente
        ack_deadline = 0.0
        pending      = bytearray()   # octets déjà lus mais pas encore traités
                                      # (reste d'un chunk après un 0xAA non sollicité)

        while self._running:
            # 1) Commande en attente ?
            try:
                cmd, arg = self._cmd_q.get_nowait()
            except queue.Empty:
                cmd = None

            if cmd == 'capture':
                line = bytearray()
                self._do_capture(ser)
                continue
            elif cmd == 'config':
                try:
                    now = datetime.now()
                    tpayload = bytes([now.year - 2000, now.month, now.day,
                                      now.hour, now.minute, now.second])
                    ser.reset_input_buffer()
                    # 'T' : règle la RTC (avant 'V', pendant SEND_YUV_FRAME)
                    ser.write(b'T' + tpayload); ser.flush()
                    time.sleep(0.2)
                    # 'V' : passe en réception de config, puis la structure
                    ser.write(b'V'); ser.flush()
                    time.sleep(0.3)
                    ser.write(arg); ser.flush()
                except Exception as e:
                    self.config_result.emit(False, f"serial error: {e}")
                    continue
                line = bytearray()
                awaiting_ack = True
                saw_fail     = False
                ack_deadline = time.time() + 5.0
                # on retombe dans la lecture normale (journal + détection ack)

            # 2) Lecture continue : journalise les printf, détecte l'ack
            try:
                data = pending + ser.read(256)
                pending = bytearray()
            except Exception:
                break

            i = 0
            n = len(data)
            while i < n:
                byte = data[i]
                i += 1
                # ack config = octet isolé 'V'/'F' (hors d'une ligne de texte).
                # Le µC peut envoyer un ou plusieurs 'F' (config pas encore
                # prête) AVANT le 'V' final : seul 'V' valide, 'F' = on attend.
                if awaiting_ack and len(line) == 0 and byte in (0x56, 0x46):
                    if byte == 0x56:             # 'V' : succès définitif
                        awaiting_ack = False
                    else:                        # 'F' : pas encore, on continue
                        saw_fail = True
                    continue
                # Snapshot non sollicité (RECORD_MODE_INIT, mouvement détecté) :
                # même protocole 0xAA que la capture 'S', mais pas précédé
                # d'une demande de notre part. Le reste du chunk déjà lu est
                # transmis en "pending" (les octets qui suivent le 0xAA font
                # déjà partie de la taille/du JPEG, il ne faut pas les perdre).
                if len(line) == 0 and byte == 0xAA:
                    pending = self._read_and_emit_snapshot(ser, bytearray(data[i:]), unsolicited=True)
                    break
                if byte == 0x0A:                 # \n : fin de ligne
                    text = line.decode('ascii', errors='replace').rstrip('\r')
                    line = bytearray()
                    if text:
                        self.line_received.emit(text)
                elif byte != 0x0D:               # ignore \r
                    line.append(byte)

            if awaiting_ack and time.time() > ack_deadline:
                awaiting_ack = False
                if saw_fail:
                    self.config_result.emit(
                        False, "⚠  config rejected by the microcontroller (f).")
                else:
                    self.config_result.emit(
                        False, "⚠  no response from the microcontroller (timeout).")

        try:
            ser.close()
        except Exception:
            pass
        self.port_opened.emit(False)

    # ── Lecture d'un bloc de n octets, en piochant d'abord dans `leftover` ──
    # (octets déjà lus dans un chunk précédent mais pas encore consommés)
    # avant de compléter par ser.read(). Retourne (bytes_lus, leftover_restant).
    @staticmethod
    def _read_exact(ser, count, leftover):
        buf = bytearray()
        if leftover:
            take = leftover[:count]
            buf.extend(take)
            del leftover[:len(take)]
        remaining = count - len(buf)
        if remaining > 0:
            buf.extend(ser.read(remaining))
        return bytes(buf), leftover

    # ── Lit taille + JPEG + exposition/gain après un sync 0xAA déjà consommé,
    # décode et émet image_received. Utilisé à la fois par la capture 'S' à la
    # demande et par un snapshot non sollicité (mouvement détecté côté µC).
    # `leftover` : octets du chunk courant déjà lus après le 0xAA (peut être
    # vide). Retourne les octets en trop non consommés (normalement vide),
    # à réinjecter dans la boucle principale au lieu d'être perdus.
    # `unsolicited` : True pour un snapshot poussé par RECORD_MODE_INIT (émet
    # movement_snapshot_received, pas image_received -- l'éditeur de crop de
    # la config ne doit pas être perturbé par une image reçue sans demande).
    def _read_and_emit_snapshot(self, ser, leftover, unsolicited=False):
        old_timeout = ser.timeout
        signal = self.movement_snapshot_received if unsolicited else self.image_received
        try:
            ser.timeout = 60
            size_bytes, leftover = self._read_exact(ser, 4, leftover)
            if len(size_bytes) != 4:
                self.capture_error.emit("failed to read the jpeg size.")
                return leftover
            jpeg_size = int.from_bytes(size_bytes, 'little')
            if jpeg_size > 10_000_000:
                self.capture_error.emit(f"invalid size: {jpeg_size} bytes.")
                return leftover

            ser.timeout = 30
            jpeg_data, leftover = self._read_exact(ser, jpeg_size, leftover)
            if len(jpeg_data) != jpeg_size:
                self.capture_error.emit("incomplete jpeg data.")
                return leftover

            exposure_us = 0
            gain_raw    = 0
            b, leftover = self._read_exact(ser, 4, leftover)
            if len(b) == 4:
                exposure_us = int.from_bytes(b, 'little')
            b, leftover = self._read_exact(ser, 4, leftover)
            if len(b) == 4:
                gain_raw = int.from_bytes(b, 'little')

            gain_db     = gain_raw / 1000.0
            gain_linear = 10 ** (gain_db / 20)
            iso_approx  = int(100 * gain_linear)

            if jpeg_data[:2] != b'\xff\xd8':
                self.capture_error.emit("corrupted jpeg")
                return leftover

            img_pil = Image.open(BytesIO(jpeg_data))
            img_np  = np.array(img_pil.convert("RGB"), dtype=np.uint8)
            desc = (f"exposure = {exposure_us} µs | gain = {gain_db:.1f} db "
                    f"(≈ ISO {iso_approx})")
            signal.emit(img_np, desc)

        except Exception as e:
            import traceback; traceback.print_exc()
            self.capture_error.emit(f"error: {e}")
        finally:
            ser.timeout = old_timeout
        return leftover

    # ── Capture binaire (protocole 'S') ─────────────────────────────────────
    def _do_capture(self, ser):
        ser.reset_input_buffer()
        ser.write(b'S'); ser.flush()

        # Sync 0xAA (capture + encodage JPEG : jusqu'à ~10 s).
        # Avant le 0xAA, le µC peut émettre des printf (ex.
        # "[FSM] frame captured: X KB") : on les journalise au lieu de les
        # jeter.
        ser.timeout = 2
        deadline = time.time() + 10.0
        got_sync = False
        pre = bytearray()
        while time.time() < deadline:
            b = ser.read(1)
            if not b:
                continue
            if b == b'\xaa':
                got_sync = True
                break
            c = b[0]
            if c == 0x0A:
                text = pre.decode('ascii', errors='replace').rstrip('\r')
                pre = bytearray()
                if text:
                    self.line_received.emit(text)
            elif c != 0x0D:
                pre.append(c)
        if not got_sync:
            self.capture_error.emit(
                "timeout: no sync received (is the mcu in config mode?).")
            ser.timeout = 0.2
            return

        self._read_and_emit_snapshot(ser, bytearray())
        ser.timeout = 0.2             # rétablit le timeout de lecture continue

# =============================================================================
#  Style (thème clair)
# =============================================================================

STYLE = """
QMainWindow, QWidget {
    background-color: #f4f6fa;
    color: #2a3442;
    font-family: 'Segoe UI', 'Arial', sans-serif;
}
QGroupBox {
    border: 1px solid #c9d2e0;
    border-radius: 4px;
    margin-top: 10px;
    padding-top: 6px;
    font-size: 10px;
    color: #4a5a78;
    letter-spacing: 1px;
    background-color: #fbfcfe;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 8px;
    padding: 0 4px;
}
QLineEdit {
    background-color: #ffffff;
    border: 1px solid #c9d2e0;
    border-radius: 3px;
    color: #2a3442;
    padding: 4px 8px;
    font-size: 11px;
}
QLineEdit:focus { border-color: #4a7ad4; }
QLineEdit:disabled { color: #aab2c0; background-color: #f0f2f6; border-color: #dde2ea; }
QLineEdit:read-only { color: #aab2c0; background-color: #f0f2f6; border-color: #dde2ea; }
QComboBox {
    background-color: #ffffff;
    border: 1px solid #c9d2e0;
    border-radius: 3px;
    color: #2a3442;
    padding: 4px 8px;
    font-size: 11px;
    min-height: 26px;
}
QComboBox:focus { border-color: #4a7ad4; }
QComboBox::drop-down { border: none; }
QComboBox QAbstractItemView {
    background-color: #ffffff;
    color: #2a3442;
    selection-background-color: #d8e4f8;
    border: 1px solid #c9d2e0;
}
QLabel { color: #4a5668; font-size: 10px; }
QPushButton {
    background-color: #ffffff;
    border: 1px solid #c9d2e0;
    border-radius: 3px;
    color: #4a5668;
    padding: 6px 14px;
    font-size: 10px;
    letter-spacing: 1px;
    min-height: 28px;
}
QPushButton:hover { border-color: #4a7ad4; color: #2a3442; background-color: #eef3fc; }
QPushButton:pressed { background-color: #dde7f8; }
QPushButton:disabled { color: #b0b8c4; border-color: #dde2ea; background-color: #f0f2f6; }

/* Boutons actifs : remplis d'une couleur pastel, texte blanc.
 * Désactivés : gris (règle :disabled ci-dessous). */
QPushButton#btn-capture:enabled { background-color: #7cc47c; border-color: #5a9a5a; color: #ffffff; }
QPushButton#btn-capture:enabled:hover  { background-color: #66b366; }
QPushButton#btn-capture:enabled:pressed { background-color: #559a55; }
QPushButton#btn-capture:disabled { background-color: #f0f2f6; border-color: #dde2ea; color: #b0b8c4; }

QPushButton#btn-try:enabled { background-color: #e0be5e; border-color: #c9a544; color: #ffffff; }
QPushButton#btn-try:enabled:hover  { background-color: #d3ad46; }
QPushButton#btn-try:enabled:pressed { background-color: #bd9a38; }
QPushButton#btn-try:disabled { background-color: #f0f2f6; border-color: #dde2ea; color: #b0b8c4; }

QPushButton#btn-send:enabled { background-color: #7a9ce0; border-color: #4a6ac4; color: #ffffff; font-weight: bold; }
QPushButton#btn-send:enabled:hover  { background-color: #6488d6; }
QPushButton#btn-send:enabled:pressed { background-color: #5578c8; }
QPushButton#btn-send:disabled { background-color: #f0f2f6; border-color: #dde2ea; color: #b0b8c4; }

/* Sélecteur de zone active (édition à la souris) : couleur = zone concernée,
 * pleine quand sélectionnée, pastel sinon -- toujours identifiable. */
QPushButton#btn-pipe1:checked { background-color: #d43a3a; border-color: #b32e2e; color: #ffffff; }
QPushButton#btn-pipe2:checked { background-color: #2a5ad4; border-color: #1f45ad; color: #ffffff; }
QPushButton#btn-pipe1:enabled:!checked { background-color: #f6d6d6; border-color: #e8b0b0; color: #8a3a3a; }
QPushButton#btn-pipe2:enabled:!checked { background-color: #d6e3f7; border-color: #b0c8ec; color: #3a5a8a; }
QPushButton#btn-pipe1:disabled, QPushButton#btn-pipe2:disabled {
    background-color: #f0f2f6; border-color: #dde2ea; color: #b0b8c4;
}

QLabel#dtclock {
    color: #2a5ad4;
    font-family: 'Consolas', 'Courier New', monospace;
    font-size: 13px;
    letter-spacing: 1px;
}
QTextEdit#logbox {
    background-color: #ffffff;
    border: 1px solid #c9d2e0;
    border-radius: 4px;
    color: #2a3442;
    font-family: 'Consolas', 'Courier New', monospace;
    font-size: 10px;
}
QFrame#sep { background-color: #c9d2e0; max-height: 1px; min-height: 1px; }
QFrame#panel { background-color: #fbfcfe; border: 1px solid #c9d2e0; border-radius: 4px; }
QToolTip {
    background-color: #fbfcfe;
    color: #2a3442;
    border: 1px solid #c9d2e0;
    padding: 6px 8px;
    font-size: 11px;
}
"""

# =============================================================================
#  Helpers UI
# =============================================================================

# Infobulle "Taille bloc", partagée par les champs pipe 1 et pipe 2. Texte
# simple uniquement (pas de tableaux/couleurs codées en dur) : le fond et la
# couleur de texte viennent de la règle QToolTip du style global (STYLE
# ci-dessus), comme tout le reste du GUI -- rendu fiable garanti.
BLOCK_SIZE_TOOLTIP_HTML = (
    "<b>Taille bloc</b><br>"
    "Facteur de réduction de la zone : un bloc de N&times;N pixels capteur "
    "devient 1 seul pixel de sortie.<br>"
    "<pre>■ ■ ■ ■\n■ ■ ■ ■   &rarr;   ■\n■ ■ ■ ■\n■ ■ ■ ■</pre>"
    "<i>4&times;4 pixels capteur &rarr; 1 pixel de sortie</i><br><br>"
    "Plus la valeur est grande, plus la zone est réduite (et moins "
    "bruitée) &mdash; mais sa résolution effective diminue d'autant.<br><br>"
    "<b>Décimation</b> : ne garde qu'1 pixel sur N et jette les autres "
    "(aucun moyennage &rarr; le bruit du capteur reste entier).<br>"
    "<b>Downsize</b> : redimensionnement par interpolation, qui moyenne "
    "plusieurs pixels voisins en un seul, <b>limité à &times;8</b> (&rarr; réduit le bruit).<br><br>"
    "Pipe 1 : le \"Taille bloc\" correspond au downsize seulement.<br>"
    "Pipe 2 : le \"Taille bloc\" correspond à <b>décimation &times; downsize</b> ; "
    "la décimation est ajoutée automatiquement en amont (comme facteur de l'image \"downsizée\" pour atteindre la "
    "taille de bloc demandée)."
)


def _lbl(text):
    l = QLabel(text)
    l.setFont(QFont("Segoe UI", 9))
    return l


def _info_icon(tooltip_html, size=15):
    """Petit badge 'ⓘ' discret ; l'infobulle s'ouvre au survol."""
    icon = QLabel("ℹ")  # ℹ
    icon.setFixedSize(size, size)
    icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
    icon.setStyleSheet(
        f"QLabel {{ background-color: #d6e3f7; color: #2a5ad4;"
        f" border-radius: {size // 2}px; font-size: 10px; font-weight: bold; }}"
    )
    icon.setToolTip(tooltip_html)
    icon.setCursor(Qt.CursorShape.WhatsThisCursor)
    return icon


def _lbl_with_info(text, tooltip_html):
    """Label de champ + icône info accolée (remplace un _lbl() simple dans une grille)."""
    w = QWidget()
    h = QHBoxLayout(w)
    h.setContentsMargins(0, 0, 0, 0)
    h.setSpacing(5)
    h.addWidget(_lbl(text))
    h.addWidget(_info_icon(tooltip_html))
    h.addStretch()
    return w


def _int_field(default="0", max_val=99999):
    f = QLineEdit(str(default))
    f.setValidator(QIntValidator(0, max_val))
    f.setMaximumWidth(90)
    return f

# =============================================================================
#  Fenêtre principale
# =============================================================================

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Configuration Caméra — STM32N6")
        self.setStyleSheet(STYLE)

        # État du flux de travail
        self._last_image = None    # numpy array
        self._worker     = None    # SerialWorker (unique propriétaire du port)
        self._auto_port  = None    # port de la carte ST détectée

        self._ready    = False   # µC prêt (message "wait for send yuv frame")
        self._captured = False   # une capture a réussi
        self._tested   = False   # config actuellement « appliquée » (aperçu interactif affiché)
        self._sent     = False   # config envoyée (fin de session)
        self._busy     = False   # capture/envoi en cours
        self._active_pipe = 'p1' # zone modifiable à la souris (persiste entre les applications)

        self._build_ui()
        self._connect_signals()
        self._update_dec_label()  # calcule décimation/downsize pour la taille bloc par défaut
        self._reset_display()     # état initial : aucune image -> tous les champs verrouillés
        self._update_buttons()

        self.resize(1200, 850)

        # ── Horloge (date/heure envoyée au ST) ───────────────────────────────
        self._dt_timer = QTimer(self)
        self._dt_timer.timeout.connect(self._update_dt_label)
        self._dt_timer.start(1000)
        self._update_dt_label()

        # ── Connexion automatique : polling des ports série ──────────────────
        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self._poll_ports)
        self._poll_timer.start(POLL_MS)
        self._poll_ports()

    # ── Construction UI ───────────────────────────────────────────────────────

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        # ── Panneau gauche ────────────────────────────────────────────────────
        left = QFrame(); left.setObjectName("panel"); left.setFixedWidth(275)
        ll = QVBoxLayout(left)
        ll.setContentsMargins(12, 12, 12, 12)
        ll.setSpacing(10)

        title = QLabel("DIAS\nConfiguration caméra")
        title.setFont(QFont("Segoe UI", 17, QFont.Weight.Bold))
        title.setStyleSheet("color: #2a5ad4; letter-spacing: 3px;")
        ll.addWidget(title)

        s0 = QFrame(); s0.setObjectName("sep"); ll.addWidget(s0)

        # ── Connexion (automatique) ───────────────────────────────────────────
        pg = QGroupBox("Connexion carte ST (auto)")
        pl = QVBoxLayout(pg); pl.setSpacing(5)
        self.port_info = QLabel("Recherche de la carte…")
        self.port_info.setStyleSheet("color: #96702a; font-size: 10px;")
        pl.addWidget(self.port_info)
        ll.addWidget(pg)

        # ── Date / heure envoyée au ST ────────────────────────────────────────
        dtg = QGroupBox("Date/heure (envoyée au ST)")
        dtl = QVBoxLayout(dtg); dtl.setSpacing(5)
        self.dt_label = QLabel("------ --:--:--")
        self.dt_label.setObjectName("dtclock")
        self.dt_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        dtl.addWidget(self.dt_label)
        ll.addWidget(dtg)

        # ── Bouton Capture ────────────────────────────────────────────────────
        self.btn_capture = QPushButton("Capturer une image")
        self.btn_capture.setObjectName("btn-capture")
        self.btn_capture.setMinimumHeight(34)
        ll.addWidget(self.btn_capture)

        s1 = QFrame(); s1.setObjectName("sep"); ll.addWidget(s1)

        # ── Appliquer/retirer + zone active + déverrouillage manuel ──────────
        apply_group = QGroupBox("Aperçu et édition de la config")
        agl = QVBoxLayout(apply_group)
        agl.setSpacing(8)

        # Appliquer / retirer la config (bascule l'aperçu interactif)
        self.btn_apply = QPushButton("Appliquer la config")
        self.btn_apply.setObjectName("btn-try")
        self.btn_apply.setMinimumHeight(32)
        agl.addWidget(self.btn_apply)

        # Sélecteur de zone active : quand les zones pipe1/pipe2 se
        # chevauchent, celle du dessous ne peut pas être attrapée à la
        # souris — on choisit ici laquelle des deux répond au glisser.
        # Toujours affiché, mais grisé tant qu'aucune zone n'est appliquée.
        self.active_bar = QWidget()
        abl = QHBoxLayout(self.active_bar)
        abl.setContentsMargins(0, 0, 0, 0); abl.setSpacing(6)
        abl.addWidget(_lbl("Zone à modifier :"))
        self.btn_active_p1 = QPushButton("Pipe 1"); self.btn_active_p1.setObjectName("btn-pipe1")
        self.btn_active_p2 = QPushButton("Pipe 2"); self.btn_active_p2.setObjectName("btn-pipe2")
        for b in (self.btn_active_p1, self.btn_active_p2):
            b.setCheckable(True); b.setMinimumHeight(24)
        self.btn_active_p1.setChecked(True)
        self._active_group = QButtonGroup(self)
        self._active_group.setExclusive(True)
        self._active_group.addButton(self.btn_active_p1)
        self._active_group.addButton(self.btn_active_p2)
        abl.addStretch()
        abl.addWidget(self.btn_active_p1)
        abl.addWidget(self.btn_active_p2)
        agl.addWidget(self.active_bar)

        # Déverrouillage manuel des champs de limites
        self.chk_unlock_fields = QCheckBox("Modifier les limites manuellement")
        agl.addWidget(self.chk_unlock_fields)

        # Aperçu de la résolution réduite (pixellisation réelle des zones,
        # simulant decimation+downsize) -- décoché par défaut, config déjà
        # appliquée ou non.
        self.chk_pixel_preview = QCheckBox("Aperçu résolution réduite")
        agl.addWidget(self.chk_pixel_preview)

        ll.addWidget(apply_group)

        # ── Pipe 1 ────────────────────────────────────────────────────────────
        g1 = QGroupBox("Second plan : zone rouge (pipe 1)")
        g1l = QGridLayout(g1); g1l.setSpacing(6)
        g1l.addWidget(_lbl("Limite haute  (Y px)"), 0, 0)
        self.p1_top = _int_field(500);    g1l.addWidget(self.p1_top, 0, 1)
        g1l.addWidget(_lbl("Limite basse  (Y px)"), 1, 0)
        self.p1_bot = _int_field(800); g1l.addWidget(self.p1_bot, 1, 1)
        g1l.addWidget(_lbl("Limite gauche (X px)"), 2, 0)
        self.p1_left = _int_field(0);   g1l.addWidget(self.p1_left, 2, 1)
        g1l.addWidget(_lbl("Limite droite (X px)"), 3, 0)
        self.p1_right = _int_field(2592); g1l.addWidget(self.p1_right, 3, 1)
        g1l.addWidget(_lbl_with_info("Taille bloc   (px, ≤8)", BLOCK_SIZE_TOOLTIP_HTML), 4, 0)
        self.p1_bs = _int_field(5, 8);  g1l.addWidget(self.p1_bs, 4, 1)
        ll.addWidget(g1)

        # ── Pipe 2 ────────────────────────────────────────────────────────────
        g2 = QGroupBox("Premier plan : zone bleue (pipe 2)")
        g2l = QGridLayout(g2); g2l.setSpacing(6)
        g2l.addWidget(_lbl("Limite haute  (Y px)"), 0, 0)
        self.p2_top = _int_field(800);    g2l.addWidget(self.p2_top, 0, 1)
        g2l.addWidget(_lbl("Limite basse  (Y px)"), 1, 0)
        self.p2_bot = _int_field(1900); g2l.addWidget(self.p2_bot, 1, 1)
        g2l.addWidget(_lbl("Limite gauche (X px)"), 2, 0)
        self.p2_left = _int_field(0);   g2l.addWidget(self.p2_left, 2, 1)
        g2l.addWidget(_lbl("Limite droite (X px)"), 3, 0)
        self.p2_right = _int_field(2592); g2l.addWidget(self.p2_right, 3, 1)
        g2l.addWidget(_lbl_with_info("Taille bloc   (px)", BLOCK_SIZE_TOOLTIP_HTML), 4, 0)
        self.p2_bs = _int_field(35);    g2l.addWidget(self.p2_bs, 4, 1)

        # Info decimation (calculée automatiquement, champ readonly comme les
        # autres champs d'information) — spécifique au pipe 2 : lui seul
        # décime (pipe 1 downsize sans décimation, downsize_ratio_pipe1
        # découle directement de sa taille de bloc, sans recherche de
        # combinaison).
        self.p2_dec_label = QLineEdit("décimation = — | downsize = —")
        self.p2_dec_label.setReadOnly(True)
        g2l.addWidget(self.p2_dec_label, 5, 0, 1, 2)
        ll.addWidget(g2)

        s2 = QFrame(); s2.setObjectName("sep"); ll.addWidget(s2)

        # ── Bouton Envoyer ─────────────────────────────────────────────────────
        self.btn_send = QPushButton("Envoyer la configuration")
        self.btn_send.setObjectName("btn-send")
        self.btn_send.setMinimumHeight(34)
        ll.addWidget(self.btn_send)

        ll.addStretch()

        # ── Côté droit : affichage + journal ──────────────────────────────────
        right = QWidget()
        rl = QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.setSpacing(8)

        self.placeholder = make_placeholder_label()
        self.crop_view   = InteractiveCropView()
        self.display_stack = QStackedWidget()
        self.display_stack.addWidget(self.placeholder)
        self.display_stack.addWidget(self.crop_view)
        rl.addWidget(self.display_stack, stretch=1)

        log_group = QGroupBox("Journal")
        lgl = QVBoxLayout(log_group)
        lgl.setContentsMargins(8, 12, 8, 8)
        self.logbox = QTextEdit()
        self.logbox.setObjectName("logbox")
        self.logbox.setReadOnly(True)
        self.logbox.setMinimumHeight(160)
        self.logbox.setMaximumHeight(240)
        lgl.addWidget(self.logbox)
        rl.addWidget(log_group)

        root.addWidget(left)
        root.addWidget(right, stretch=1)

        self._log("application started")
        self._log("looking for dev. board (VID 0x0483)…")

    # ── Journal ───────────────────────────────────────────────────────────────

    def _log(self, msg):
        self.logbox.append(f"<span style='color:#2a5ad4'>APP DIAS &raquo;</span>&nbsp;&nbsp;{msg}")
        sb = self.logbox.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _log_stm(self, text):
        """Ligne brute reçue du STM32 (printf)."""
        self.logbox.append(f"<span style='color:#2f7a2f'>STM &raquo;</span> {text}")
        sb = self.logbox.verticalScrollBar()
        sb.setValue(sb.maximum())
        # Le µC signale qu'il attend une capture -> (ré)active "Capturer"
        if "(capturer une image)" in text:
            self._on_ready()
        if "RESTART OF THE CONFIG PROCEDURE" in text:
            self._on_config_warmup()

    def _reset_display(self):
        """Repart sur le placeholder (aucune image) et reverrouille les 8
        champs de limites — nouvelle session de capture, la position figée
        par une application précédente n'a plus de sens tant qu'on n'a pas
        réappliqué sur la nouvelle image."""
        self.display_stack.setCurrentWidget(self.placeholder)
        self.btn_apply.setText("Appliquer la config")
        self.crop_view.clear_rects()
        self._last_image = None
        self.chk_unlock_fields.setChecked(False)
        for f in (self.p1_top, self.p1_bot, self.p1_left, self.p1_right,
                  self.p2_top, self.p2_bot, self.p2_left, self.p2_right):
            f.setReadOnly(True)

    def _on_ready(self):
        """Reçu à chaque fois que le µC entre en attente de capture ('wait for
        send yuv frame') : on repart d'un flux propre et on (ré)active
        "Capturer".  Le µC n'émet ce message qu'une fois par entrée dans cet
        état (jamais entre une capture et son envoi), donc ce reset ne clobbe
        pas une capture en cours."""
        self._ready    = True
        self._busy     = False
        self._captured = False
        self._tested   = False
        self._sent     = False
        self._reset_display()
        self._update_buttons()

    def _on_config_warmup(self):
        """Le µC (re)démarre un warmup de config : retour à l'état initial du
        lancement, les 3 boutons sont désactivés jusqu'au prochain
        'wait for send yuv frame'."""
        self._ready    = False
        self._busy     = False
        self._captured = False
        self._tested   = False
        self._sent     = False
        self._reset_display()
        self._update_buttons()

    # ── Horloge date/heure ─────────────────────────────────────────────────────

    def _update_dt_label(self):
        self.dt_label.setText(datetime.now().strftime("%Y-%m-%d  %H:%M:%S"))

    # ── Worker série (unique propriétaire du port) ─────────────────────────────

    def _start_worker(self):
        if self._worker is not None or not self._auto_port:
            return
        self._worker = SerialWorker(self._auto_port)
        self._worker.line_received.connect(self._log_stm)
        self._worker.image_received.connect(self._on_image_received)
        self._worker.movement_snapshot_received.connect(self._on_movement_snapshot)
        self._worker.capture_error.connect(self._on_error)
        self._worker.config_result.connect(self._on_send_result)
        self._worker.status.connect(self._log)
        self._worker.start()

    def _stop_worker(self):
        if self._worker is not None:
            self._worker.stop()
            self._worker = None

    # ── Signaux ───────────────────────────────────────────────────────────────

    def _connect_signals(self):
        self.btn_capture.clicked.connect(self._do_capture)
        self.btn_apply.clicked.connect(self._do_apply_toggle)
        self.btn_send.clicked.connect(self._do_send)
        self.p2_bs.textChanged.connect(self._update_dec_label)
        self.btn_active_p1.clicked.connect(lambda: self._set_active_pipe('p1'))
        self.btn_active_p2.clicked.connect(lambda: self._set_active_pipe('p2'))
        self.chk_unlock_fields.toggled.connect(self._on_unlock_toggled)
        self.chk_pixel_preview.toggled.connect(self._on_pixel_preview_toggled)
        for f in (self.p1_top, self.p1_bot, self.p1_left, self.p1_right):
            f.textChanged.connect(lambda _, k='p1': self._sync_rect_from_fields(k))
        for f in (self.p2_top, self.p2_bot, self.p2_left, self.p2_right):
            f.textChanged.connect(lambda _, k='p2': self._sync_rect_from_fields(k))
        self.p1_bs.textChanged.connect(lambda: self._update_pixel_preview('p1'))
        self.p2_bs.textChanged.connect(lambda: self._update_pixel_preview('p2'))

    # ── Connexion automatique (polling VID 0x0483) ────────────────────────────

    def _poll_ports(self):
        st_port = None
        for p in serial.tools.list_ports.comports():
            if p.vid == ST_VID:
                st_port = p
                break

        if st_port is not None:
            if self._auto_port != st_port.device:
                self._auto_port = st_port.device
                desc = st_port.description or "carte ST"
                self.port_info.setText(f"✔ Connectée : {st_port.device}")
                self.port_info.setStyleSheet("color: #2f7a2f; font-size: 10px;")
                self._log(f"dev. board detected ({desc}).")
                # Nouvelle connexion : flux propre.  "Capturer" reste désactivé
                # jusqu'à réception de "wait for send yuv frame".
                self._ready    = False
                self._captured = False
                self._tested   = False
                self._sent     = False
                self._reset_display()
                self._update_buttons()
                self._start_worker()
        else:
            if self._auto_port is not None:
                self._log("dev. board disconnected")
                self._stop_worker()
                self._ready = False
            self._auto_port = None
            self.port_info.setText("Searching for dev. board… (VID 0x0483)")
            self.port_info.setStyleSheet("color: #96702a; font-size: 10px;")
            self._update_buttons()

    # ── Info decimation pipe 2 ────────────────────────────────────────────────

    def _update_dec_label(self):
        try:
            bs = int(self.p2_bs.text())
        except ValueError:
            self.p2_dec_label.setStyleSheet("")
            self.p2_dec_label.setText("décimation = — | downsize = —")
            return
        if bs <= 0:
            self.p2_dec_label.setStyleSheet("")
            self.p2_dec_label.setText("décimation = — | downsize = —")
            return
        dec, ds = compute_pipe2_params(bs)
        if dec is None:
            self.p2_dec_label.setText("⚠ taille de bloc invalide pour ce pipe")
            self.p2_dec_label.setStyleSheet("color: #d43a3a;")
        else:
            self.p2_dec_label.setText(f"décimation = {dec} | downsize = {ds:.2f}")
            self.p2_dec_label.setStyleSheet("")

    # ── Capture ───────────────────────────────────────────────────────────────

    def _do_capture(self):
        self._busy = True
        self._update_buttons()
        self._worker.request_capture()

    def _on_image_received(self, img_np, desc):
        self._last_image = img_np
        self._captured = True
        self._tested   = False        # nouvelle capture => il faut réappliquer
        self.btn_apply.setText("Appliquer la config")
        self.crop_view.clear_rects()
        self.crop_view.set_image(img_np)
        self.display_stack.setCurrentWidget(self.crop_view)
        self._busy = False
        self._update_buttons()

    def _on_movement_snapshot(self, img_np, desc):
        """Snapshot poussé par le µC dès qu'un mouvement déclenche un
        enregistrement (RECORD_MODE_INIT) -- affichage seul, ne touche pas à
        l'état d'édition de la config (rectangles de crop, bouton Appliquer)
        contrairement à _on_image_received (capture 'S' manuelle)."""
        self.crop_view.set_image(img_np)
        self.display_stack.setCurrentWidget(self.crop_view)

    # ── Appliquer / retirer la config (local) ─────────────────────────────────

    def _on_pipe1_rect_changed(self, top, bottom, left, right):
        self.p1_top.setText(str(top));   self.p1_bot.setText(str(bottom))
        self.p1_left.setText(str(left)); self.p1_right.setText(str(right))
        self._update_pixel_preview('p1')

    def _on_pipe2_rect_changed(self, top, bottom, left, right):
        self.p2_top.setText(str(top));   self.p2_bot.setText(str(bottom))
        self.p2_left.setText(str(left)); self.p2_right.setText(str(right))
        self._update_pixel_preview('p2')

    # ── Aperçu résolution réduite ────────────────────────────────────────────

    def _get_block_size(self, key):
        """Facteur de réduction total de la zone (= « taille bloc » pour les
        deux pipes : pour pipe 2, decimation x downsize == taille bloc par
        construction, voir compute_pipe2_params). None si le champ est
        invalide -- l'appelant doit alors s'abstenir de dessiner l'aperçu."""
        try:
            if key == 'p1':
                bs = int(self.p1_bs.text())
                if bs < 1 or bs > 8:
                    return None
                return min(float(bs), MAX_DOWNSIZE)
            bs = int(self.p2_bs.text())
            if bs < 1:
                return None
            dec, _ds = compute_pipe2_params(bs)
            return float(bs) if dec is not None else None
        except ValueError:
            return None

    def _update_pixel_preview(self, key):
        block_size = self._get_block_size(key)
        if block_size is not None:
            self.crop_view.update_pixel_preview(key, block_size)

    def _on_pixel_preview_toggled(self, checked):
        self.crop_view.set_pixelate_enabled(checked)
        if checked:
            self._update_pixel_preview('p1')
            self._update_pixel_preview('p2')

    def _sync_rect_from_fields(self, key):
        """Répercute en direct une saisie manuelle (champs déverrouillés)
        sur le rectangle correspondant, sans attendre un « Retirer » suivi
        d'un « Appliquer » -- silencieusement ignoré si les 4 valeurs ne
        forment pas encore un rectangle valide (en cours de frappe). Aussi
        déclenché (sans effet, values déjà identiques) par les setText() du
        glisser-déposer lui-même — inoffensif, pas de boucle puisque
        set_rect() ne réémet pas on_change."""
        if not self._tested:
            return
        if key == 'p1':
            fields = (self.p1_top, self.p1_bot, self.p1_left, self.p1_right)
            color, callback = '#d43a3a', self._on_pipe1_rect_changed
        else:
            fields = (self.p2_top, self.p2_bot, self.p2_left, self.p2_right)
            color, callback = '#2a5ad4', self._on_pipe2_rect_changed
        try:
            top, bottom, left, right = (int(f.text()) for f in fields)
        except ValueError:
            return
        if bottom <= top or right <= left:
            return
        self.crop_view.set_rect(key, top, bottom, left, right, color, callback)
        self._update_pixel_preview(key)

    def _on_unlock_toggled(self, checked):
        """La case « Modifier les limites manuellement » est la seule
        autorité sur l'édition au clavier des 8 champs — indépendante de
        l'application/retrait de la config ou du glisser-déposer (qui, lui,
        continue de fonctionner même readonly puisqu'il écrit par code)."""
        for f in (self.p1_top, self.p1_bot, self.p1_left, self.p1_right,
                  self.p2_top, self.p2_bot, self.p2_left, self.p2_right):
            f.setReadOnly(not checked)

    def _do_apply_toggle(self):
        """Ajoute/retire la surimpression interactive des zones pipe1/pipe2,
        aux coordonnées actuellement dans les champs (toujours la base
        utilisée) — ajustables directement à la souris (coins/côtés), comme
        un rectangle Word. L'image et sa graduation restent affichées par
        InteractiveCropView dans les deux cas (déjà en place depuis la
        capture) : aucun changement de vue, donc aucune divergence de style
        possible entre config appliquée et retirée."""
        if not self._tested:
            cfg = self._read_config()
            if cfg is None: return

            self.crop_view.set_rect(
                'p1',
                cfg['crop_v_start_pipe1'],
                cfg['crop_v_start_pipe1'] + cfg['crop_v_size_pipe1'],
                cfg['crop_h_start_pipe1'],
                cfg['crop_h_start_pipe1'] + cfg['crop_h_size_pipe1'],
                '#d43a3a', self._on_pipe1_rect_changed)
            self.crop_view.set_rect(
                'p2',
                cfg['crop_v_start_pipe2'],
                cfg['crop_v_start_pipe2'] + cfg['crop_v_size_pipe2'],
                cfg['crop_h_start_pipe2'],
                cfg['crop_h_start_pipe2'] + cfg['crop_h_size_pipe2'],
                '#2a5ad4', self._on_pipe2_rect_changed)
            self.crop_view.set_active_pipe(self._active_pipe)
            self.btn_active_p1.setChecked(self._active_pipe == 'p1')
            self.btn_active_p2.setChecked(self._active_pipe == 'p2')
            self._update_pixel_preview('p1')
            self._update_pixel_preview('p2')

            self._tested = True             # débloque « Envoyer »
            self.btn_apply.setText("Retirer la config")
        else:
            self.crop_view.clear_rects()
            self._tested = False
            self.btn_apply.setText("Appliquer la config")

        self._update_buttons()

    def _set_active_pipe(self, key):
        self._active_pipe = key
        self.crop_view.set_active_pipe(key)

    # ── Envoyer la config ─────────────────────────────────────────────────────

    def _do_send(self):
        cfg = self._read_config()
        if cfg is None: return
        data = struct.pack(
            '<IHHHHHHHHBff',
            MAGIC,
            cfg['crop_v_start_pipe1'], cfg['crop_v_size_pipe1'],
            cfg['crop_h_start_pipe1'], cfg['crop_h_size_pipe1'],
            cfg['crop_v_start_pipe2'], cfg['crop_v_size_pipe2'],
            cfg['crop_h_start_pipe2'], cfg['crop_h_size_pipe2'],
            cfg['decimation_ratio_pipe2'],
            cfg['downsize_ratio_pipe1'],
            cfg['downsize_ratio_pipe2'],
        )
        self._busy = True
        self._update_buttons()
        # L'image disparaît dès l'envoi de la config
        self.display_stack.setCurrentWidget(self.placeholder)
        self.btn_apply.setText("Appliquer la config")
        self.crop_view.clear_rects()
        self._last_image = None
        self._worker.request_config(data)

    def _on_send_result(self, success, msg):
        self._log(msg)
        if success:
            # Config validée : l'image disparaît, les 3 boutons se figent
            self._sent = True
            self.display_stack.setCurrentWidget(self.placeholder)
            self.crop_view.clear_rects()
            self._last_image = None
        self._busy = False
        self._update_buttons()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _update_buttons(self):
        """Applique les règles d'enchaînement Capturer → Appliquer → Envoyer."""
        connected = self._auto_port is not None

        # Taille bloc : verrouillée tant qu'aucune image n'a été capturée --
        # contrairement aux 8 champs de limites, ce paramètre n'a pas
        # d'équivalent « glisser à la souris », donc pas besoin d'attendre
        # « Modifier les limites manuellement » : juste qu'une image existe.
        self.p1_bs.setReadOnly(not self._captured)
        self.p2_bs.setReadOnly(not self._captured)

        if self._sent or self._busy:
            # Session terminée ou opération en cours : tout est figé
            self.btn_capture.setEnabled(False)
            self.btn_apply.setEnabled(False)
            self.btn_send.setEnabled(False)
            self.btn_active_p1.setEnabled(False)
            self.btn_active_p2.setEnabled(False)
            return

        # Capturer : seulement quand le µC a signalé "wait for send yuv frame"
        self.btn_capture.setEnabled(connected and self._ready)
        # Appliquer/retirer : seulement après une capture
        self.btn_apply.setEnabled(connected and self._captured)
        # Envoyer : seulement après une capture ET une application
        self.btn_send.setEnabled(connected and self._captured and self._tested)
        # Zone active (pipe1/pipe2) : n'a de sens que si la config est appliquée
        self.btn_active_p1.setEnabled(self._tested)
        self.btn_active_p2.setEnabled(self._tested)

    def _read_config(self):
        def to_int(field, name):
            try:
                v = int(field.text())
                if v < 0: raise ValueError
                return v
            except ValueError:
                self._log(f"⚠  invalid value: '{name}'.")
                return None

        p1_top   = to_int(self.p1_top,   "pipe 1 — top limit")
        if p1_top   is None: return None
        p1_bot   = to_int(self.p1_bot,   "pipe 1 — bottom limit")
        if p1_bot   is None: return None
        p1_left  = to_int(self.p1_left,  "pipe 1 — left limit")
        if p1_left  is None: return None
        p1_right = to_int(self.p1_right, "pipe 1 — right limit")
        if p1_right is None: return None
        p1_bs    = to_int(self.p1_bs,    "pipe 1 — block size")
        if p1_bs    is None: return None

        p2_top   = to_int(self.p2_top,   "pipe 2 — top limit")
        if p2_top   is None: return None
        p2_bot   = to_int(self.p2_bot,   "pipe 2 — bottom limit")
        if p2_bot   is None: return None
        p2_left  = to_int(self.p2_left,  "pipe 2 — left limit")
        if p2_left  is None: return None
        p2_right = to_int(self.p2_right, "pipe 2 — right limit")
        if p2_right is None: return None
        p2_bs    = to_int(self.p2_bs,    "pipe 2 — block size")
        if p2_bs    is None: return None

        # Validations
        if p1_bot <= p1_top:
            self._log("⚠  pipe 1: bottom limit must be > top limit.")
            return None
        if p1_right <= p1_left:
            self._log("⚠  pipe 1: right limit must be > left limit.")
            return None
        if p2_bot <= p2_top:
            self._log("⚠  pipe 2: bottom limit must be > top limit.")
            return None
        if p2_right <= p2_left:
            self._log("⚠  pipe 2: right limit must be > left limit.")
            return None
        if p1_bs < 1 or p1_bs > 8:
            self._log("⚠  pipe 1: block size must be between 1 and 8.")
            return None
        if p2_bs < 1:
            self._log("⚠  pipe 2: block size must be ≥ 1.")
            return None

        # Calcul decimation pipe 2
        dec2, ds2 = compute_pipe2_params(p2_bs)
        if dec2 is None:
            self._log(
                f"⚠  pipe 2: block size {p2_bs} invalid — no decimation/downsize "
                f"combination possible (downsize must be ≤ 8)."
            )
            return None

        # downsize pipe1 : jamais exactement 8
        ds1 = min(float(p1_bs), MAX_DOWNSIZE)

        return dict(
            crop_v_start_pipe1   = p1_top,
            crop_v_size_pipe1    = p1_bot - p1_top,
            crop_h_start_pipe1   = p1_left,
            crop_h_size_pipe1    = p1_right - p1_left,
            crop_v_start_pipe2   = p2_top,
            crop_v_size_pipe2    = p2_bot - p2_top,
            crop_h_start_pipe2   = p2_left,
            crop_h_size_pipe2    = p2_right - p2_left,
            decimation_ratio_pipe2 = dec2,
            downsize_ratio_pipe1   = ds1,
            downsize_ratio_pipe2   = ds2,
        )

    def _on_error(self, msg):
        self._log(f"⚠  {msg}")
        self._busy = False
        self._update_buttons()

    def closeEvent(self, event):
        self._poll_timer.stop()
        self._dt_timer.stop()
        self._stop_worker()
        event.accept()

# =============================================================================
#  Lancement
# =============================================================================

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    sys.exit(app.exec())
