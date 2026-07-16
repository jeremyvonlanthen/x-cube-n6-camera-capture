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

Dépendances : PyQt6, matplotlib, numpy, Pillow, pyserial
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

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from PIL import Image

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QLineEdit, QPushButton,
    QFrame, QGroupBox, QSizePolicy, QTextEdit,
)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt6.QtGui import QFont, QIntValidator, QPixmap

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
#  Rendu matplotlib → QPixmap
# =============================================================================

def fig_to_pixmap(fig, dpi=90):
    buf = BytesIO()
    fig.savefig(buf, format='png', dpi=dpi, bbox_inches='tight',
                facecolor=fig.get_facecolor())
    buf.seek(0)
    pix = QPixmap()
    pix.loadFromData(buf.read(), "PNG")
    return pix


def make_raw_pixmap(img, size):
    """Image brute avec graduation."""
    w_px, h_px = size
    dpi = 90
    fig, ax = plt.subplots(figsize=(w_px / dpi, h_px / dpi))
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(BG)
    h, w = img.shape[:2]
    ax.imshow(img, aspect='equal')
    ax.set_xticks(np.arange(0, w, 100))
    ax.set_yticks(np.arange(0, h, 100))
    ax.tick_params(colors='#5a6a80', labelsize=7)
    for sp in ax.spines.values():
        sp.set_edgecolor('#b8c4d4')
    ax.set_title("Capture — graduation en pixels", color=FG, fontsize=9, pad=4)
    fig.tight_layout(pad=0.3)
    pix = fig_to_pixmap(fig, dpi=dpi)
    plt.close(fig)
    return pix


def make_preview_pixmap(img, p1_top, p1_bot, p1_left, p1_right,
                         p2_top, p2_bot, p2_left, p2_right, size):
    """Image avec zones pipe 1 (rouge) et pipe 2 (bleu), crop H et V."""
    w_px, h_px = size
    dpi = 90
    fig, ax = plt.subplots(figsize=(w_px / dpi, h_px / dpi))
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(BG)
    h, w = img.shape[:2]
    ax.imshow(img, aspect='equal')

    # Pipe 1 — rouge : rectangle délimité par les 4 bords
    ax.add_patch(patches.Rectangle(
        (p1_left, p1_top), p1_right - p1_left, p1_bot - p1_top,
        linewidth=1.5, edgecolor='#d43a3a', facecolor='#d43a3a', alpha=0.25))
    ax.text(p1_left + 8, (p1_top + p1_bot) / 2, "Pipe 1", color='#d43a3a', fontsize=8,
            va='center', bbox=dict(facecolor='white', alpha=0.6, pad=1, edgecolor='none'))

    # Pipe 2 — bleu : rectangle délimité par les 4 bords
    ax.add_patch(patches.Rectangle(
        (p2_left, p2_top), p2_right - p2_left, p2_bot - p2_top,
        linewidth=1.5, edgecolor='#2a5ad4', facecolor='#2a5ad4', alpha=0.25))
    ax.text(p2_left + 8, (p2_top + p2_bot) / 2, "Pipe 2", color='#2a5ad4', fontsize=8,
            va='center', bbox=dict(facecolor='white', alpha=0.6, pad=1, edgecolor='none'))

    ax.set_xticks(np.arange(0, w, 100))
    ax.set_yticks(np.arange(0, h, 100))
    ax.tick_params(colors='#5a6a80', labelsize=7)
    for sp in ax.spines.values():
        sp.set_edgecolor('#b8c4d4')
    ax.set_title("Test de la config — graduation en pixels", color=FG, fontsize=9, pad=4)
    fig.tight_layout(pad=0.3)
    pix = fig_to_pixmap(fig, dpi=dpi)
    plt.close(fig)
    return pix

# =============================================================================
#  Widget d'affichage
# =============================================================================

class ImageDisplay(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setMinimumSize(400, 300)
        self._pixmap_full = None
        self._show_placeholder()

    def _show_placeholder(self):
        self.setText("Aucune image capturée")
        self.setStyleSheet(
            f"background-color: {BG}; border: 1px solid #b8c4d4;"
            "color: #8a96a8; font-size: 13px; font-family: 'Segoe UI';"
        )

    def set_pixmap(self, pixmap):
        self._pixmap_full = pixmap
        self.setStyleSheet(f"background-color: {BG}; border: 1px solid #b8c4d4;")
        self.setText("")
        self._rescale()

    def clear_image(self):
        """Efface l'image affichée et remet le placeholder."""
        self._pixmap_full = None
        self.clear()
        self._show_placeholder()

    def _rescale(self):
        if self._pixmap_full and not self._pixmap_full.isNull():
            scaled = self._pixmap_full.scaled(
                self.width(), self.height(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            )
            self.setPixmap(scaled)

    def resizeEvent(self, event):
        self._rescale()
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
    image_received = pyqtSignal(object, str)   # numpy img, description
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
            self.line_received.emit(f"[moniteur] ouverture impossible : {e}")
            self.port_opened.emit(False)
            return
        self.port_opened.emit(True)

        line         = bytearray()   # ligne printf en cours de reconstruction
        awaiting_ack = False         # attente de l'ack config
        saw_fail     = False         # au moins un 'F' reçu pendant l'attente
        ack_deadline = 0.0

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
                    self.config_result.emit(False, f"Erreur série : {e}")
                    continue
                line = bytearray()
                awaiting_ack = True
                saw_fail     = False
                ack_deadline = time.time() + 5.0
                # on retombe dans la lecture normale (journal + détection ack)

            # 2) Lecture continue : journalise les printf, détecte l'ack
            try:
                data = ser.read(256)
            except Exception:
                break

            for byte in data:
                # ack config = octet isolé 'V'/'F' (hors d'une ligne de texte).
                # Le µC peut envoyer un ou plusieurs 'F' (config pas encore
                # prête) AVANT le 'V' final : seul 'V' valide, 'F' = on attend.
                if awaiting_ack and len(line) == 0 and byte in (0x56, 0x46):
                    if byte == 0x56:             # 'V' : succès définitif
                        awaiting_ack = False
                    else:                        # 'F' : pas encore, on continue
                        saw_fail = True
                    continue
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
                        False, "⚠  Config refusée par le microcontrôleur (F).")
                else:
                    self.config_result.emit(
                        False, "⚠  Pas de réponse du microcontrôleur (timeout).")

        try:
            ser.close()
        except Exception:
            pass
        self.port_opened.emit(False)

    # ── Capture binaire (protocole 'S') ─────────────────────────────────────
    def _do_capture(self, ser):
        try:
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
                    "Timeout — pas de sync reçu (le µC est-il en mode config ?).")
                return

            ser.timeout = 60
            size_bytes = ser.read(4)
            if len(size_bytes) != 4:
                self.capture_error.emit("Erreur de lecture de la taille JPEG.")
                return
            jpeg_size = int.from_bytes(size_bytes, 'little')
            if jpeg_size > 10_000_000:
                self.capture_error.emit(f"Taille invalide : {jpeg_size} octets.")
                return

            ser.timeout = 30
            jpeg_data = ser.read(jpeg_size)
            if len(jpeg_data) != jpeg_size:
                self.capture_error.emit("Données JPEG incomplètes.")
                return

            exposure_us = 0
            gain_raw    = 0
            b = ser.read(4)
            if len(b) == 4:
                exposure_us = int.from_bytes(b, 'little')
            b = ser.read(4)
            if len(b) == 4:
                gain_raw = int.from_bytes(b, 'little')

            gain_db     = gain_raw / 1000.0
            gain_linear = 10 ** (gain_db / 20)
            iso_approx  = int(100 * gain_linear)

            if jpeg_data[:2] != b'\xff\xd8':
                self.capture_error.emit("JPEG corrompu.")
                return

            img_pil = Image.open(BytesIO(jpeg_data))
            img_np  = np.array(img_pil.convert("RGB"), dtype=np.uint8)
            desc = (f"exposition = {exposure_us} µs | gain = {gain_db:.1f} dB "
                    f"(≈ ISO {iso_approx})")
            self.image_received.emit(img_np, desc)

        except Exception as e:
            import traceback; traceback.print_exc()
            self.capture_error.emit(f"Erreur : {e}")
        finally:
            ser.timeout = 0.2         # rétablit le timeout de lecture continue

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
QLineEdit:disabled { color: #9aa4b4; background-color: #eceff5; }
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
"""

# =============================================================================
#  Helpers UI
# =============================================================================

def _lbl(text):
    l = QLabel(text)
    l.setFont(QFont("Segoe UI", 9))
    return l


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
        self._tested   = False   # « Tester » pressé depuis la dernière capture
        self._sent     = False   # config envoyée (fin de session)
        self._busy     = False   # capture/envoi en cours

        self._build_ui()
        self._connect_signals()
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

        # ── Pipe 1 ────────────────────────────────────────────────────────────
        g1 = QGroupBox("Pipe 1  —  zone rouge")
        g1l = QGridLayout(g1); g1l.setSpacing(6)
        g1l.addWidget(_lbl("Limite haute  (Y px)"), 0, 0)
        self.p1_top = _int_field(500);    g1l.addWidget(self.p1_top, 0, 1)
        g1l.addWidget(_lbl("Limite basse  (Y px)"), 1, 0)
        self.p1_bot = _int_field(800); g1l.addWidget(self.p1_bot, 1, 1)
        g1l.addWidget(_lbl("Limite gauche (X px)"), 2, 0)
        self.p1_left = _int_field(0);   g1l.addWidget(self.p1_left, 2, 1)
        g1l.addWidget(_lbl("Limite droite (X px)"), 3, 0)
        self.p1_right = _int_field(2592); g1l.addWidget(self.p1_right, 3, 1)
        g1l.addWidget(_lbl("Taille bloc   (px, ≤8)"), 4, 0)
        self.p1_bs = _int_field(5, 8);  g1l.addWidget(self.p1_bs, 4, 1)
        ll.addWidget(g1)

        # ── Pipe 2 ────────────────────────────────────────────────────────────
        g2 = QGroupBox("Pipe 2  —  zone bleue")
        g2l = QGridLayout(g2); g2l.setSpacing(6)
        g2l.addWidget(_lbl("Limite haute  (Y px)"), 0, 0)
        self.p2_top = _int_field(800);    g2l.addWidget(self.p2_top, 0, 1)
        g2l.addWidget(_lbl("Limite basse  (Y px)"), 1, 0)
        self.p2_bot = _int_field(1900); g2l.addWidget(self.p2_bot, 1, 1)
        g2l.addWidget(_lbl("Limite gauche (X px)"), 2, 0)
        self.p2_left = _int_field(0);   g2l.addWidget(self.p2_left, 2, 1)
        g2l.addWidget(_lbl("Limite droite (X px)"), 3, 0)
        self.p2_right = _int_field(2592); g2l.addWidget(self.p2_right, 3, 1)
        g2l.addWidget(_lbl("Taille bloc   (px)"),   4, 0)
        self.p2_bs = _int_field(35);    g2l.addWidget(self.p2_bs, 4, 1)

        # Info decimation (calculée automatiquement, affichage seul)
        self.p2_dec_label = QLabel("décimation=— / downsize=—")
        self.p2_dec_label.setStyleSheet("color: #2a5ad4; font-size: 9px;")
        g2l.addWidget(self.p2_dec_label, 5, 0, 1, 2)
        ll.addWidget(g2)

        s2 = QFrame(); s2.setObjectName("sep"); ll.addWidget(s2)

        # ── Boutons Tester / Envoyer ──────────────────────────────────────────
        self.btn_try  = QPushButton("Tester la configuration")
        self.btn_try.setObjectName("btn-try")
        self.btn_try.setMinimumHeight(32)
        ll.addWidget(self.btn_try)

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

        self.display = ImageDisplay()
        rl.addWidget(self.display, stretch=1)

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
        self._log("<b>NOTE: IF NECESSARY, PRESS <i>USER1</i> BUTTON (SD initialization included)</b>")
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
        if "wait for send yuv frame" in text:
            self._on_ready()
        if "config mode warmup" in text:
            self._on_config_warmup()

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
        self.display.clear_image()
        self._last_image = None
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
        self.display.clear_image()
        self._last_image = None
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
        self.btn_try.clicked.connect(self._do_try)
        self.btn_send.clicked.connect(self._do_send)
        self.p2_bs.textChanged.connect(self._update_dec_label)

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
                self.display.clear_image()
                self._last_image = None
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
            self.p2_dec_label.setText("décimation=— / downsize=—")
            return
        if bs <= 0:
            self.p2_dec_label.setText("décimation=— / downsize=—")
            return
        dec, ds = compute_pipe2_params(bs)
        if dec is None:
            self.p2_dec_label.setText("⚠ taille de bloc invalide pour ce pipe")
            self.p2_dec_label.setStyleSheet("color: #d43a3a; font-size: 9px;")
        else:
            self.p2_dec_label.setText(f"décimation={dec}  downsize={ds:.4f}")
            self.p2_dec_label.setStyleSheet("color: #2a5ad4; font-size: 9px;")

    # ── Capture ───────────────────────────────────────────────────────────────

    def _do_capture(self):
        self._busy = True
        self._update_buttons()
        self._worker.request_capture()

    def _on_image_received(self, img_np, desc):
        self._last_image = img_np
        self._captured = True
        self._tested   = False        # nouvelle capture => il faut re-tester
        size = (self.display.width(), self.display.height())
        pix = make_raw_pixmap(img_np, size)
        self.display.set_pixmap(pix)
        self._busy = False
        self._update_buttons()

    # ── Tester la config (local) ──────────────────────────────────────────────

    def _do_try(self):
        cfg = self._read_config()
        if cfg is None: return
        size = (self.display.width(), self.display.height())
        pix = make_preview_pixmap(
            self._last_image,
            cfg['crop_v_start_pipe1'],
            cfg['crop_v_start_pipe1'] + cfg['crop_v_size_pipe1'],
            cfg['crop_h_start_pipe1'],
            cfg['crop_h_start_pipe1'] + cfg['crop_h_size_pipe1'],
            cfg['crop_v_start_pipe2'],
            cfg['crop_v_start_pipe2'] + cfg['crop_v_size_pipe2'],
            cfg['crop_h_start_pipe2'],
            cfg['crop_h_start_pipe2'] + cfg['crop_h_size_pipe2'],
            size
        )
        self.display.set_pixmap(pix)
        self._tested = True           # débloque « Envoyer »
        self._update_buttons()

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
        self.display.clear_image()
        self._last_image = None
        self._worker.request_config(data)

    def _on_send_result(self, success, msg):
        self._log(msg)
        if success:
            # Config validée : l'image disparaît, les 3 boutons se figent
            self._sent = True
            self.display.clear_image()
            self._last_image = None
        self._busy = False
        self._update_buttons()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _update_buttons(self):
        """Applique les règles d'enchaînement Capturer → Tester → Envoyer."""
        connected = self._auto_port is not None

        if self._sent or self._busy:
            # Session terminée ou opération en cours : tout est figé
            self.btn_capture.setEnabled(False)
            self.btn_try.setEnabled(False)
            self.btn_send.setEnabled(False)
            return

        # Capturer : seulement quand le µC a signalé "wait for send yuv frame"
        self.btn_capture.setEnabled(connected and self._ready)
        # Tester : seulement après une capture
        self.btn_try.setEnabled(connected and self._captured)
        # Envoyer : seulement après une capture ET un test
        self.btn_send.setEnabled(connected and self._captured and self._tested)

    def _read_config(self):
        def to_int(field, name):
            try:
                v = int(field.text())
                if v < 0: raise ValueError
                return v
            except ValueError:
                self._log(f"⚠  valeur invalide : « {name} ».")
                return None

        p1_top   = to_int(self.p1_top,   "Pipe 1 — limite haute")
        if p1_top   is None: return None
        p1_bot   = to_int(self.p1_bot,   "Pipe 1 — limite basse")
        if p1_bot   is None: return None
        p1_left  = to_int(self.p1_left,  "Pipe 1 — limite gauche")
        if p1_left  is None: return None
        p1_right = to_int(self.p1_right, "Pipe 1 — limite droite")
        if p1_right is None: return None
        p1_bs    = to_int(self.p1_bs,    "Pipe 1 — taille bloc")
        if p1_bs    is None: return None

        p2_top   = to_int(self.p2_top,   "Pipe 2 — limite haute")
        if p2_top   is None: return None
        p2_bot   = to_int(self.p2_bot,   "Pipe 2 — limite basse")
        if p2_bot   is None: return None
        p2_left  = to_int(self.p2_left,  "Pipe 2 — limite gauche")
        if p2_left  is None: return None
        p2_right = to_int(self.p2_right, "Pipe 2 — limite droite")
        if p2_right is None: return None
        p2_bs    = to_int(self.p2_bs,    "Pipe 2 — taille bloc")
        if p2_bs    is None: return None

        # Validations
        if p1_bot <= p1_top:
            self._log("⚠  pipe 1 : la limite basse doit être > limite haute.")
            return None
        if p1_right <= p1_left:
            self._log("⚠  pipe 1 : la limite droite doit être > limite gauche.")
            return None
        if p2_bot <= p2_top:
            self._log("⚠  pipe 2 : la limite basse doit être > limite haute.")
            return None
        if p2_right <= p2_left:
            self._log("⚠  pipe 2 : la limite droite doit être > limite gauche.")
            return None
        if p1_bs < 1 or p1_bs > 8:
            self._log("⚠  pipe 1 : la taille de bloc doit être entre 1 et 8.")
            return None
        if p2_bs < 1:
            self._log("⚠  pipe 2 : la taille de bloc doit être ≥ 1.")
            return None

        # Calcul decimation pipe 2
        dec2, ds2 = compute_pipe2_params(p2_bs)
        if dec2 is None:
            self._log(
                f"⚠  pipe 2 : taille de bloc {p2_bs} invalide — aucune combinaison "
                f"décimation/downsize possible (downsize doit être ≤ 8)."
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
