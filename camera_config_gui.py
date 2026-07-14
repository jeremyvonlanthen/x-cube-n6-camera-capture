"""
Configuration Caméra — GUI
--------------------------
Interface PyQt6 pour configurer et capturer des images depuis le STM32N6
via UART.

Nouveautés :
  - Connexion automatique (polling) sur la carte ST (VID USB 0x0483)
  - Bouton « Confirmer » supprimé : « Tester la config » disponible à tout moment
  - « Envoyer la config » envoie d'abord 'V' (sortie du mode capture côté µC)
    puis la structure Config_t
  - Thème clair, interface en français
  - Zone de journal agrandie

Dépendances : PyQt6, matplotlib, numpy, Pillow, pyserial
Usage       : python camera_config_gui.py
"""

import sys
import os
import time
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

from PIL import Image, ImageDraw, ImageFont

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QLineEdit, QPushButton, QFileDialog,
    QFrame, QStatusBar, QGroupBox, QSizePolicy, QComboBox, QTextEdit,
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
IMAGE_W        = 2592
IMAGE_H        = 1944
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
        self.setMinimumSize(600, 350)
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
#  Thread de capture UART
# =============================================================================

class CaptureThread(QThread):
    image_received = pyqtSignal(object, str)   # numpy img, description
    error          = pyqtSignal(str)
    status         = pyqtSignal(str)

    def __init__(self, port, save_folder=None):
        super().__init__()
        self.port        = port
        self.save_folder = save_folder   # None = pas de sauvegarde sur disque

    def run(self):
        try:
            self.status.emit("Connexion UART…")
            cdc = serial.Serial(self.port, UART_BAUDRATE, timeout=2)
            time.sleep(0.5)

            # Vider le buffer
            cdc.reset_input_buffer()
            time.sleep(0.1)
            while cdc.in_waiting:
                cdc.read(cdc.in_waiting)
                time.sleep(0.1)

            self.status.emit("Envoi commande 'S'…")
            cdc.write(b'S')
            cdc.flush()

            # Attendre sync 0xAA (la capture + encodage JPEG côté µC prend
            # un peu de temps : on laisse jusqu'à ~10 s)
            self.status.emit("Attente du sync…")
            cdc.timeout = 2
            deadline = time.time() + 10.0
            got_sync = False
            while time.time() < deadline:
                sync = cdc.read(1)
                if sync == b'\xaa':
                    got_sync = True
                    break
            if not got_sync:
                self.error.emit("Timeout — pas de sync reçu (le µC est-il en mode config ?).")
                cdc.close()
                return

            # Lire taille
            cdc.timeout = 60
            size_bytes = cdc.read(4)
            if len(size_bytes) != 4:
                self.error.emit("Erreur de lecture de la taille JPEG.")
                cdc.close()
                return

            jpeg_size = int.from_bytes(size_bytes, 'little')
            self.status.emit(f"Réception de l'image ({jpeg_size} octets)…")

            if jpeg_size > 10_000_000:
                self.error.emit(f"Taille invalide : {jpeg_size} octets.")
                cdc.close()
                return

            cdc.timeout = 30
            jpeg_data = cdc.read(jpeg_size)
            if len(jpeg_data) != jpeg_size:
                self.error.emit("Données JPEG incomplètes.")
                cdc.close()
                return

            # Lire exposition et gain
            exposure_us = 0
            gain_raw    = 0
            exposure_bytes = cdc.read(4)
            if len(exposure_bytes) == 4:
                exposure_us = int.from_bytes(exposure_bytes, 'little')

            gain_bytes = cdc.read(4)
            if len(gain_bytes) == 4:
                gain_raw = int.from_bytes(gain_bytes, 'little')

            gain_db     = gain_raw / 1000.0
            gain_linear = 10 ** (gain_db / 20)
            iso_approx  = int(100 * gain_linear)

            cdc.close()

            # Vérifier JPEG
            if jpeg_data[:2] != b'\xff\xd8':
                self.error.emit("JPEG corrompu.")
                return

            now = datetime.now()
            img_pil = Image.open(BytesIO(jpeg_data))

            # ── Sauvegarde sur disque désactivée (dossier non requis) ────────
            # if self.save_folder:
            #     os.makedirs(self.save_folder, exist_ok=True)
            #     filename = f"capture_{now.strftime('%Y%m%d_%H%M%S')}.jpg"
            #     filepath = os.path.join(self.save_folder, filename)
            #     with open(filepath, 'wb') as f:
            #         f.write(jpeg_data)
            #     # Ajouter timestamp sur l'image
            #     img_pil = Image.open(filepath)
            #     draw    = ImageDraw.Draw(img_pil)
            #     try:
            #         font = ImageFont.truetype("arial.ttf", 36)
            #     except Exception:
            #         font = ImageFont.load_default()
            #     draw.text((50, 50), now.strftime('%d/%m/%Y\n%Hh%M %Ss'),
            #               fill=(255,), font=font)
            #     img_pil.save(filepath)
            # ─────────────────────────────────────────────────────────────────

            # Convertir en numpy pour affichage
            img_np = np.array(img_pil.convert("RGB"), dtype=np.uint8)

            desc = (f"exposition = {exposure_us} µs | gain = {gain_db:.1f} dB "
                    f"(≈ ISO {iso_approx})")
            self.status.emit(f"Image reçue — {desc}")
            self.image_received.emit(img_np, desc)

        except serial.SerialException as e:
            self.error.emit(f"Erreur série : {e}")
        except Exception as e:
            import traceback; traceback.print_exc()
            self.error.emit(f"Erreur : {e}")


# =============================================================================
#  Thread d'envoi de config UART
# =============================================================================

class SendConfigThread(QThread):
    result = pyqtSignal(bool, str)   # success, message
    status = pyqtSignal(str)

    def __init__(self, port, cfg):
        super().__init__()
        self.port = port
        self.cfg  = cfg

    def run(self):
        try:
            cfg = self.cfg
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

            ser = serial.Serial(self.port, UART_BAUDRATE, timeout=2)
            time.sleep(0.5)
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            # 1) 'V' : fait sortir le µC du mode capture (SEND_YUV_FRAME)
            #    vers la réception de config (RECEIVE_PIPES_CONFIG)
            self.status.emit("Envoi de 'V' (passage en réception de config)…")
            ser.write(b'V')
            ser.flush()
            time.sleep(0.3)

            # 2) La structure Config_t
            self.status.emit("Envoi de la structure de configuration…")
            ser.write(data)
            ser.flush()

            # 3) Attendre l'acquittement 'V' (le µC répond 'F' tant que la
            #    config n'est pas validée)
            timeout = time.time() + 5.0
            while time.time() < timeout:
                answer = ser.read(1)
                if answer == b'V':
                    ser.close()
                    self.result.emit(True, "✔  Config reçue et validée par le microcontrôleur.")
                    return
                elif answer == b'F':
                    pass  # en attente côté µC
            ser.close()
            self.result.emit(False, "⚠  Pas de réponse du microcontrôleur (timeout).")

        except serial.SerialException as e:
            self.result.emit(False, f"Erreur série : {e}")
        except Exception as e:
            self.result.emit(False, f"Erreur : {e}")


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
QPushButton#btn-capture { border-color: #5a9a5a; color: #2f7a2f; }
QPushButton#btn-capture:hover { background-color: #ecf6ec; border-color: #3f9a3f; color: #1f6a1f; }
QPushButton#btn-try { border-color: #c09a4a; color: #96702a; }
QPushButton#btn-try:hover { background-color: #faf4e6; border-color: #caa050; color: #7a5a1a; }
QPushButton#btn-send { border-color: #4a6ac4; color: #2a4aa4; font-weight: bold; }
QPushButton#btn-send:hover { background-color: #eaf0fc; border-color: #2a5ad4; color: #1a3a94; }
QPushButton#btn-refresh { border-color: #c9d2e0; color: #4a5a78; padding: 4px 8px; min-height: 24px; }
QPushButton#btn-refresh:hover { border-color: #4a7ad4; color: #2a3442; }
QStatusBar {
    background-color: #e8ecf4;
    color: #4a5a78;
    font-size: 9px;
    border-top: 1px solid #c9d2e0;
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
        self.resize(1300, 850)
        self.setStyleSheet(STYLE)

        self._last_image     = None   # numpy array
        self._capture_thread = None
        self._send_thread    = None
        self._auto_port      = None   # port de la carte ST détectée

        self._build_ui()
        self._connect_signals()
        self._set_buttons_state("idle")

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

        title = QLabel("CONFIG\nCAMÉRA")
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

        # ── Dossier de sauvegarde — désactivé (non nécessaire pour l'instant) ─
        # dg = QGroupBox("Dossier de sauvegarde")
        # dl = QVBoxLayout(dg); dl.setSpacing(5)
        # dr = QHBoxLayout()
        # self.dir_field = QLineEdit()
        # self.dir_field.setPlaceholderText("chemin/vers/dossier…")
        # self.dir_btn = QPushButton("…")
        # self.dir_btn.setFixedWidth(30); self.dir_btn.setFixedHeight(28)
        # dr.addWidget(self.dir_field); dr.addWidget(self.dir_btn)
        # dl.addLayout(dr)
        # ll.addWidget(dg)
        # ──────────────────────────────────────────────────────────────────────

        # ── Bouton Capture ────────────────────────────────────────────────────
        self.btn_capture = QPushButton("▶  Capturer")
        self.btn_capture.setObjectName("btn-capture")
        self.btn_capture.setMinimumHeight(34)
        self.btn_capture.setEnabled(False)
        ll.addWidget(self.btn_capture)

        # ── Bouton Confirm — supprimé (Try config disponible à tout moment) ──
        # self.btn_confirm = QPushButton("✔  Confirm")
        # self.btn_confirm.setObjectName("btn-confirm")
        # self.btn_confirm.setMinimumHeight(34)
        # self.btn_confirm.setEnabled(False)
        # ──────────────────────────────────────────────────────────────────────

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
        self.btn_try  = QPushButton("◈  Tester la config")
        self.btn_try.setObjectName("btn-try")
        self.btn_try.setMinimumHeight(32)
        ll.addWidget(self.btn_try)

        self.btn_send = QPushButton("⚡  Envoyer la config")
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

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Prêt.")
        self._log("Application démarrée — recherche de la carte ST (VID 0x0483)…")

    # ── Journal ───────────────────────────────────────────────────────────────

    def _log(self, msg):
        ts = datetime.now().strftime("%H:%M:%S")
        self.logbox.append(f"[{ts}]  {msg}")
        sb = self.logbox.verticalScrollBar()
        sb.setValue(sb.maximum())
        self.status_bar.showMessage(msg)

    # ── Signaux ───────────────────────────────────────────────────────────────

    def _connect_signals(self):
        # self.dir_btn.clicked.connect(self._browse_dir)   # dossier désactivé
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
                self.port_info.setText(f"✔ Connectée : {st_port.device}\n{desc}")
                self.port_info.setStyleSheet("color: #2f7a2f; font-size: 10px;")
                self._log(f"Carte ST détectée sur {st_port.device} ({desc}).")
                self.btn_capture.setEnabled(True)
        else:
            if self._auto_port is not None:
                self._log("Carte ST déconnectée.")
            self._auto_port = None
            self.port_info.setText("⌛ Recherche de la carte… (VID 0x0483)")
            self.port_info.setStyleSheet("color: #96702a; font-size: 10px;")
            self.btn_capture.setEnabled(False)

    # ── Dossier — désactivé ───────────────────────────────────────────────────

    # def _browse_dir(self):
    #     d = QFileDialog.getExistingDirectory(self, "Sélectionner le dossier de sauvegarde")
    #     if d:
    #         self.dir_field.setText(d)

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
        port = self._get_port()
        if not port: return
        self._set_buttons_state("busy")
        self._log("Capture demandée…")
        self._capture_thread = CaptureThread(port)   # pas de dossier de sauvegarde
        self._capture_thread.image_received.connect(self._on_image_received)
        self._capture_thread.error.connect(self._on_error)
        self._capture_thread.status.connect(self._log)
        self._capture_thread.finished.connect(lambda: self._set_buttons_state("idle"))
        self._capture_thread.start()

    def _on_image_received(self, img_np, desc):
        self._last_image = img_np
        size = (self.display.width(), self.display.height())
        pix = make_raw_pixmap(img_np, size)
        self.display.set_pixmap(pix)
        self._log(f"Image affichée ({desc}).")

    # ── Tester la config (local, disponible à tout moment) ───────────────────

    def _do_try(self):
        if self._last_image is None:
            self._log("⚠  Faites d'abord une capture.")
            return
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
        self._log(
            f"Test de la config  |  "
            f"Pipe1 Y[{cfg['crop_v_start_pipe1']}–{cfg['crop_v_start_pipe1']+cfg['crop_v_size_pipe1']}]  "
            f"Pipe2 Y[{cfg['crop_v_start_pipe2']}–{cfg['crop_v_start_pipe2']+cfg['crop_v_size_pipe2']}]"
        )

    # ── Envoyer la config ─────────────────────────────────────────────────────

    def _do_send(self):
        port = self._get_port()
        if not port: return
        cfg = self._read_config()
        if cfg is None: return
        self._set_buttons_state("busy")
        self._log("Envoi de la configuration…")
        self._send_thread = SendConfigThread(port, cfg)
        self._send_thread.result.connect(self._on_send_result)
        self._send_thread.status.connect(self._log)
        self._send_thread.finished.connect(lambda: self._set_buttons_state("idle"))
        self._send_thread.start()

    def _on_send_result(self, success, msg):
        self._log(msg)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _get_port(self):
        if not self._auto_port:
            self._log("⚠  Aucune carte ST détectée.")
            return None
        return self._auto_port

    def _read_config(self):
        def to_int(field, name):
            try:
                v = int(field.text())
                if v < 0: raise ValueError
                return v
            except ValueError:
                self._log(f"⚠  Valeur invalide : « {name} ».")
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
            self._log("⚠  Pipe 1 : la limite basse doit être > limite haute.")
            return None
        if p1_right <= p1_left:
            self._log("⚠  Pipe 1 : la limite droite doit être > limite gauche.")
            return None
        if p2_bot <= p2_top:
            self._log("⚠  Pipe 2 : la limite basse doit être > limite haute.")
            return None
        if p2_right <= p2_left:
            self._log("⚠  Pipe 2 : la limite droite doit être > limite gauche.")
            return None
        if p1_bs < 1 or p1_bs > 8:
            self._log("⚠  Pipe 1 : la taille de bloc doit être entre 1 et 8.")
            return None
        if p2_bs < 1:
            self._log("⚠  Pipe 2 : la taille de bloc doit être ≥ 1.")
            return None

        # Calcul decimation pipe 2
        dec2, ds2 = compute_pipe2_params(p2_bs)
        if dec2 is None:
            self._log(
                f"⚠  Pipe 2 : taille de bloc {p2_bs} invalide — aucune combinaison "
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
        self._set_buttons_state("idle")

    def _set_buttons_state(self, state):
        busy = (state == "busy")
        for w in (self.btn_try, self.btn_send):
            w.setEnabled(not busy)
        self.btn_capture.setEnabled(not busy and self._auto_port is not None)

    def closeEvent(self, event):
        self._poll_timer.stop()
        for t in (self._capture_thread, self._send_thread):
            if t and t.isRunning():
                t.wait(2000)
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
