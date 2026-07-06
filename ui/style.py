"""Dark theme for the desktop app — implements the "Drone LiDAR UI Redesign".

A Fusion base + dark QPalette (so native dialogs follow the theme) plus a QSS
stylesheet built from the redesign's design tokens (surfaces, borders, text,
accent/status colours, radii) and its component states (default / hover / focus /
pressed / disabled / error). apply_dark_theme(app) is called once at startup.

Type: the design specifies IBM Plex Sans / IBM Plex Mono. Those aren't bundled
offline, so UI text falls back to Segoe UI and numeric readouts to a monospace
stack (MONO). Drop the .ttf in and swap FONT_UI/MONO to match the mockup exactly.
"""
from PySide6.QtGui import QPalette, QColor
from PySide6.QtWidgets import QStyleFactory

# ── Design tokens ───────────────────────────────────────────────────────────
# Surfaces
APP_BG = "#0e1013"          # window background
PANEL = "#16191d"           # panels (sidebar, results, title/menu bars)
PANEL_RAISED = "#1b1f24"    # group boxes / raised cards
INPUT_BG = "#1a1e23"        # input fields

# Borders
BORDER = "#2c323a"          # subtle divider / panel border
BORDER_CTRL = "#384049"     # control (button/input) border
BORDER_STRONG = "#3a424c"   # emphasised border
FOCUS = "#5b8db8"           # focus ring (== accent)

# Text
TEXT = "#e3e6e9"            # primary
TEXT_SECONDARY = "#aab2bb"  # secondary
MUTED = "#8b96a3"           # labels / muted
FAINT = "#74808c"           # captions / status bar
DISABLED = "#4a525c"

# Accent + status
ACCENT = "#5b8db8"
ACCENT_HOVER = "#6b9dc8"
ACCENT_PRESS = "#4a7aa3"
ACCENT_TEXT = "#0d1114"
SUCCESS = "#4f9d7a"
WARNING = "#c9973f"
DANGER = "#c1584f"

# Status washes (banner backgrounds) + headline colours
SUCCESS_WASH_BG, SUCCESS_WASH_BORDER, SUCCESS_HEAD = "#1d2a24", "#2f5240", "#9fd6b8"
WARNING_WASH_BG, WARNING_WASH_BORDER, WARNING_HEAD = "#2a2318", "#52411f", "#e6c98a"
DANGER_WASH_BG,  DANGER_WASH_BORDER,  DANGER_HEAD  = "#2a1c1a", "#52302b", "#e7a8a2"
NEUTRAL_WASH_BG, NEUTRAL_WASH_BORDER = PANEL_RAISED, BORDER

# Type
FONT_UI = "'Segoe UI', 'IBM Plex Sans', sans-serif"
MONO = "'Consolas', 'IBM Plex Mono', monospace"

QSS = f"""
* {{ outline: none; }}
QWidget {{ color: {TEXT}; font-family: {FONT_UI}; font-size: 13px; }}
QMainWindow, QDialog {{ background: {APP_BG}; }}

/* Toolbar */
QToolBar {{ background: {PANEL}; border: none; border-bottom: 1px solid {BORDER};
           spacing: 8px; padding: 6px 12px; }}
QToolBar QToolButton {{ background: {PANEL_RAISED}; color: {TEXT_SECONDARY};
           padding: 6px 12px; border: 1px solid {BORDER_CTRL}; border-radius: 5px; }}
QToolBar QToolButton:hover {{ background: #2a3038; border-color: #465060; color: {TEXT}; }}
QToolBar QToolButton:pressed {{ background: #1e232a; }}
QToolBar QToolButton:checked {{ background: {ACCENT_PRESS}; color: {ACCENT_TEXT};
           border-color: {ACCENT_PRESS}; }}

/* Group boxes */
QGroupBox {{ background: {PANEL_RAISED}; border: 1px solid {BORDER}; border-radius: 6px;
            margin-top: 12px; padding: 12px 12px 10px 12px; font-size: 11px;
            font-weight: 600; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; top: 1px;
            padding: 0 4px; color: {TEXT}; font-weight: 600; }}

QLabel {{ background: transparent; }}

/* Inputs */
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QPlainTextEdit {{
    background: {INPUT_BG}; color: {TEXT}; border: 1px solid {BORDER_CTRL};
    border-radius: 4px; padding: 4px 10px; min-height: 22px;
    selection-background-color: {ACCENT}; selection-color: {ACCENT_TEXT}; }}
QLineEdit:hover, QSpinBox:hover, QDoubleSpinBox:hover, QComboBox:hover,
QPlainTextEdit:hover {{ border-color: #4a525c; background: #1e2227; }}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus,
QPlainTextEdit:focus {{ border: 1px solid {FOCUS}; }}
QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QComboBox:disabled,
QPlainTextEdit:disabled {{ background: #17191d; color: {DISABLED}; border-color: #22262c; }}
QLineEdit[error="true"], QDoubleSpinBox[error="true"], QSpinBox[error="true"] {{
    border: 1px solid {DANGER}; background: #241716; color: {DANGER_HEAD}; }}
QComboBox::drop-down {{ border: none; width: 20px; }}
QComboBox QAbstractItemView {{ background: {PANEL_RAISED}; border: 1px solid {BORDER_CTRL};
    selection-background-color: #2a3d4d; color: {TEXT}; }}
/* Leave spin arrows to Fusion so they stay visible; just widen the hit target. */
QSpinBox::up-button, QSpinBox::down-button,
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{ width: 18px; }}

/* Secondary buttons */
QPushButton {{ background: #232830; color: {TEXT_SECONDARY}; border: 1px solid {BORDER_CTRL};
    border-radius: 5px; padding: 6px 14px; font-size: 12px; }}
QPushButton:hover {{ background: #2a3038; border-color: #465060; color: {TEXT}; }}
QPushButton:pressed {{ background: #1e232a; }}
QPushButton:focus {{ border: 1px solid {FOCUS}; }}
QPushButton:disabled {{ background: {PANEL_RAISED}; color: {DISABLED}; border-color: {BORDER}; }}

/* Primary action button */
QPushButton#primary {{ background: {ACCENT}; color: {ACCENT_TEXT}; border: 1px solid {ACCENT_PRESS};
    border-radius: 5px; padding: 9px 16px; font-weight: 700; font-size: 13px; }}
QPushButton#primary:hover {{ background: {ACCENT_HOVER}; border-color: #5a94c4; }}
QPushButton#primary:pressed {{ background: {ACCENT_PRESS}; border-color: #3d6688; }}
QPushButton#primary:focus {{ border: 1px solid #7fb0d8; }}
QPushButton#primary:disabled {{ background: #232830; color: {DISABLED}; border-color: {BORDER}; }}

QToolButton {{ background: #232830; border: 1px solid {BORDER_CTRL}; color: {TEXT_SECONDARY};
    border-radius: 5px; padding: 5px 10px; }}
QToolButton:hover {{ background: #2a3038; border-color: #465060; color: {TEXT}; }}
QToolButton:checked {{ background: {ACCENT_PRESS}; color: {ACCENT_TEXT}; border-color: {ACCENT_PRESS}; }}

/* Disclosure header (collapsible section) — flat, quiet, not a chunky button */
QToolButton#disclosure {{ background: transparent; border: none; color: {MUTED};
    padding: 3px 2px; font-size: 11px; font-weight: 600; }}
QToolButton#disclosure:hover {{ background: transparent; color: {TEXT_SECONDARY}; }}
QToolButton#disclosure:checked {{ background: transparent; color: {TEXT_SECONDARY};
    border: none; }}

/* Checkbox */
QCheckBox {{ spacing: 7px; }}
QCheckBox::indicator {{ width: 16px; height: 16px; border: 1px solid {BORDER_CTRL};
    border-radius: 4px; background: {INPUT_BG}; }}
QCheckBox::indicator:hover {{ border-color: #4a525c; }}
QCheckBox::indicator:checked {{ background: {ACCENT}; border-color: {ACCENT}; }}

/* Tables (raw metrics) */
QTableWidget, QTableView {{ background: {PANEL_RAISED}; alternate-background-color: #191d22;
    border: 1px solid {BORDER}; border-radius: 6px; gridline-color: {BORDER};
    color: {TEXT}; font-size: 12px; selection-background-color: #2a3d4d;
    selection-color: {TEXT}; }}
QTableWidget::item, QTableView::item {{ padding: 6px 10px; border: none; }}
QHeaderView::section {{ background: #1e242b; color: {MUTED}; border: none;
    border-bottom: 1px solid {BORDER}; padding: 6px 10px; font-size: 11px; font-weight: 600; }}

/* Verdict banner — QFrame#verdictBanner + labels, driven by a "state" property */
QFrame#verdictBanner {{ border-radius: 8px; border: 1px solid {BORDER}; background: {PANEL_RAISED}; }}
QFrame#verdictBanner[state="danger"]  {{ background: {DANGER_WASH_BG};  border-color: {DANGER_WASH_BORDER}; }}
QFrame#verdictBanner[state="warning"] {{ background: {WARNING_WASH_BG}; border-color: {WARNING_WASH_BORDER}; }}
QFrame#verdictBanner[state="success"] {{ background: {SUCCESS_WASH_BG}; border-color: {SUCCESS_WASH_BORDER}; }}
QLabel#verdictHeadline {{ font-size: 16px; font-weight: 700; color: {TEXT}; }}
QLabel#verdictHeadline[state="danger"]  {{ color: {DANGER_HEAD}; }}
QLabel#verdictHeadline[state="warning"] {{ color: {WARNING_HEAD}; }}
QLabel#verdictHeadline[state="success"] {{ color: {SUCCESS_HEAD}; }}
QLabel#verdictReason {{ color: {MUTED}; font-size: 12px; }}

/* Progress */
QProgressBar {{ background: {INPUT_BG}; border: 1px solid {BORDER}; border-radius: 4px;
    height: 8px; text-align: center; color: {TEXT}; }}
QProgressBar::chunk {{ background: {ACCENT}; border-radius: 3px; }}
QProgressBar#energyBar {{ border: none; border-radius: 3px; background: #241b1a; height: 6px; }}
QProgressBar#energyBar::chunk {{ border-radius: 3px; background: {DANGER}; }}
QProgressBar#energyBar[state="danger"] {{ background: #3a2320; }}
QProgressBar#energyBar[state="danger"]::chunk {{ background: {DANGER}; }}
QProgressBar#energyBar[state="warning"] {{ background: #2a2318; }}
QProgressBar#energyBar[state="warning"]::chunk {{ background: {WARNING}; }}
QProgressBar#energyBar[state="success"] {{ background: #1d2a24; }}
QProgressBar#energyBar[state="success"]::chunk {{ background: {SUCCESS}; }}

/* Status bar */
QStatusBar {{ background: {PANEL}; border-top: 1px solid {BORDER}; color: {FAINT};
    font-size: 11px; }}
QStatusBar::item {{ border: none; }}
QStatusBar QLabel[state="danger"]  {{ color: {DANGER_HEAD}; }}
QStatusBar QLabel[state="warning"] {{ color: {WARNING_HEAD}; }}
QStatusBar QLabel[state="success"] {{ color: {SUCCESS_HEAD}; }}

/* Splitters */
QSplitter::handle {{ background: {BORDER}; }}
QSplitter::handle:horizontal {{ width: 1px; }}
QSplitter::handle:vertical {{ height: 1px; }}

/* Scrollbars */
QScrollBar:vertical {{ background: transparent; width: 11px; margin: 2px; }}
QScrollBar::handle:vertical {{ background: {BORDER_CTRL}; border-radius: 5px; min-height: 24px; }}
QScrollBar::handle:vertical:hover {{ background: {MUTED}; }}
QScrollBar:horizontal {{ background: transparent; height: 11px; margin: 2px; }}
QScrollBar::handle:horizontal {{ background: {BORDER_CTRL}; border-radius: 5px; min-width: 24px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; width: 0; }}
QScrollArea {{ border: none; background: transparent; }}

QToolTip {{ background: {PANEL_RAISED}; color: {TEXT}; border: 1px solid {BORDER_CTRL};
    padding: 4px 6px; }}
QMenuBar {{ background: {PANEL}; color: {TEXT_SECONDARY}; }}
QMenuBar::item {{ padding: 4px 10px; background: transparent; }}
QMenuBar::item:selected {{ background: {PANEL_RAISED}; color: {TEXT}; }}
QMenu {{ background: {PANEL}; border: 1px solid {BORDER}; }}
QMenu::item {{ padding: 5px 22px; }}
QMenu::item:selected {{ background: {ACCENT_PRESS}; color: {ACCENT_TEXT}; }}
"""


def apply_dark_theme(app):
    app.setStyle(QStyleFactory.create("Fusion"))
    pal = QPalette()
    pal.setColor(QPalette.Window, QColor(APP_BG))
    pal.setColor(QPalette.WindowText, QColor(TEXT))
    pal.setColor(QPalette.Base, QColor(INPUT_BG))
    pal.setColor(QPalette.AlternateBase, QColor(PANEL_RAISED))
    pal.setColor(QPalette.Text, QColor(TEXT))
    pal.setColor(QPalette.Button, QColor(PANEL_RAISED))
    pal.setColor(QPalette.ButtonText, QColor(TEXT))
    pal.setColor(QPalette.Highlight, QColor(ACCENT))
    pal.setColor(QPalette.HighlightedText, QColor(ACCENT_TEXT))
    pal.setColor(QPalette.ToolTipBase, QColor(PANEL_RAISED))
    pal.setColor(QPalette.ToolTipText, QColor(TEXT))
    pal.setColor(QPalette.PlaceholderText, QColor(MUTED))
    pal.setColor(QPalette.Link, QColor(ACCENT))
    pal.setColor(QPalette.Disabled, QPalette.Text, QColor(DISABLED))
    pal.setColor(QPalette.Disabled, QPalette.ButtonText, QColor(DISABLED))
    pal.setColor(QPalette.Disabled, QPalette.WindowText, QColor(DISABLED))
    app.setPalette(pal)
    app.setStyleSheet(QSS)
