"""Entry point for the desktop (PySide6) build of the route planner.

    python desktop.py

Step-1 skeleton: parameter sidebar + compute loop + Summary/Safety panel,
wired to the existing model in src/. Map and 3D views are stubbed.
"""
import sys

from PySide6.QtWidgets import QApplication

from ui.main_window import MainWindow


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
