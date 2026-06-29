"""Dark, UgCS-style theme for the desktop app.

A Fusion base + dark QPalette (so native dialogs follow the theme) plus a QSS
stylesheet for buttons, inputs, group boxes, toolbar, dock titles, progress and
scrollbars. apply_dark_theme(app) is called once at startup.
"""
from PySide6.QtGui import QPalette, QColor
from PySide6.QtWidgets import QStyleFactory

# Palette
BG = "#1b1d21"          # window background
PANEL = "#232629"       # panels / inputs
PANEL2 = "#2b2f33"      # hover / raised
BORDER = "#383c42"
TEXT = "#e6e8ea"
MUTED = "#9aa0a6"
ACCENT = "#3fb950"      # primary (green, UgCS-ish)
ACCENT_HOVER = "#4ac75e"
ACCENT_DIM = "#2c7a3c"
DANGER = "#e5534b"

QSS = f"""
* {{ outline: none; }}
QWidget {{ color: {TEXT}; font-size: 13px; }}
QMainWindow, QDialog {{ background: {BG}; }}

/* Toolbar */
QToolBar {{ background: {PANEL}; border: none; border-bottom: 1px solid {BORDER};
           spacing: 6px; padding: 5px 8px; }}
QToolBar QToolButton {{ background: transparent; color: {TEXT};
           padding: 6px 12px; border-radius: 6px; }}
QToolBar QToolButton:hover {{ background: {PANEL2}; }}
QToolBar QToolButton:pressed {{ background: {BORDER}; }}
QToolBar QToolButton:checked {{ background: {ACCENT_DIM}; color: white; }}

/* Group boxes */
QGroupBox {{ background: {PANEL}; border: 1px solid {BORDER}; border-radius: 8px;
            margin-top: 14px; padding: 10px 10px 10px 10px; font-weight: 600; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; top: 1px;
            padding: 0 4px; color: {MUTED}; font-weight: 600; }}

/* Labels */
QLabel {{ background: transparent; }}

/* Inputs */
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QPlainTextEdit {{
    background: {BG}; border: 1px solid {BORDER}; border-radius: 6px;
    padding: 5px 8px; selection-background-color: {ACCENT}; }}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus,
QPlainTextEdit:focus {{ border: 1px solid {ACCENT}; }}
QComboBox::drop-down {{ border: none; width: 18px; }}
QComboBox QAbstractItemView {{ background: {PANEL}; border: 1px solid {BORDER};
    selection-background-color: {ACCENT_DIM}; }}
/* Leave spin up/down buttons to Fusion so their increment/decrement arrows
   stay visible; only widen them a touch for an easier click target. */
QSpinBox::up-button, QSpinBox::down-button,
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{ width: 18px; }}

/* Buttons */
QPushButton {{ background: {PANEL2}; color: {TEXT}; border: 1px solid {BORDER};
    border-radius: 6px; padding: 7px 14px; }}
QPushButton:hover {{ background: {BORDER}; }}
QPushButton:pressed {{ background: {PANEL}; }}
QPushButton:disabled {{ color: {MUTED}; background: {PANEL}; border-color: {PANEL}; }}
QPushButton#primary {{ background: {ACCENT}; color: #08130b; border: none;
    font-weight: 700; padding: 9px 14px; }}
QPushButton#primary:hover {{ background: {ACCENT_HOVER}; }}
QPushButton#primary:disabled {{ background: {ACCENT_DIM}; color: #bfe9c8; }}
QToolButton {{ background: {PANEL2}; border: 1px solid {BORDER};
    border-radius: 6px; padding: 5px 10px; }}
QToolButton:hover {{ background: {BORDER}; }}
QToolButton:checked {{ background: {ACCENT_DIM}; color: white; border-color: {ACCENT_DIM}; }}

/* Checkbox */
QCheckBox {{ spacing: 7px; }}
QCheckBox::indicator {{ width: 16px; height: 16px; border: 1px solid {BORDER};
    border-radius: 4px; background: {BG}; }}
QCheckBox::indicator:checked {{ background: {ACCENT}; border-color: {ACCENT}; }}

/* Dock widgets */
QDockWidget {{ titlebar-close-icon: none; }}
QDockWidget::title {{ background: {PANEL}; padding: 6px 10px; color: {MUTED};
    border-bottom: 1px solid {BORDER}; }}

/* Progress */
QProgressBar {{ background: {BG}; border: 1px solid {BORDER}; border-radius: 6px;
    height: 8px; text-align: center; }}
QProgressBar::chunk {{ background: {ACCENT}; border-radius: 5px; }}

/* Status bar */
QStatusBar {{ background: {PANEL}; border-top: 1px solid {BORDER}; color: {MUTED}; }}
QStatusBar::item {{ border: none; }}

/* Scrollbars */
QScrollBar:vertical {{ background: transparent; width: 11px; margin: 2px; }}
QScrollBar::handle:vertical {{ background: {BORDER}; border-radius: 5px; min-height: 24px; }}
QScrollBar::handle:vertical:hover {{ background: {MUTED}; }}
QScrollBar:horizontal {{ background: transparent; height: 11px; margin: 2px; }}
QScrollBar::handle:horizontal {{ background: {BORDER}; border-radius: 5px; min-width: 24px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; width: 0; }}
QScrollArea {{ border: none; background: transparent; }}

QToolTip {{ background: {PANEL2}; color: {TEXT}; border: 1px solid {BORDER};
    padding: 4px 6px; }}
QMenuBar {{ background: {PANEL}; }}
QMenuBar::item:selected {{ background: {PANEL2}; }}
QMenu {{ background: {PANEL}; border: 1px solid {BORDER}; }}
QMenu::item:selected {{ background: {ACCENT_DIM}; }}
"""


def apply_dark_theme(app):
    app.setStyle(QStyleFactory.create("Fusion"))
    pal = QPalette()
    pal.setColor(QPalette.Window, QColor(BG))
    pal.setColor(QPalette.WindowText, QColor(TEXT))
    pal.setColor(QPalette.Base, QColor(BG))
    pal.setColor(QPalette.AlternateBase, QColor(PANEL))
    pal.setColor(QPalette.Text, QColor(TEXT))
    pal.setColor(QPalette.Button, QColor(PANEL))
    pal.setColor(QPalette.ButtonText, QColor(TEXT))
    pal.setColor(QPalette.Highlight, QColor(ACCENT))
    pal.setColor(QPalette.HighlightedText, QColor("#08130b"))
    pal.setColor(QPalette.ToolTipBase, QColor(PANEL2))
    pal.setColor(QPalette.ToolTipText, QColor(TEXT))
    pal.setColor(QPalette.PlaceholderText, QColor(MUTED))
    pal.setColor(QPalette.Link, QColor(ACCENT))
    pal.setColor(QPalette.Disabled, QPalette.Text, QColor(MUTED))
    pal.setColor(QPalette.Disabled, QPalette.ButtonText, QColor(MUTED))
    pal.setColor(QPalette.Disabled, QPalette.WindowText, QColor(MUTED))
    app.setPalette(pal)
    app.setStyleSheet(QSS)
