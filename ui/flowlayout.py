"""A layout that wraps its widgets onto more rows when the row gets too narrow.

Used for the map toolbar, so the map (and with it the whole window) can be made narrow
on small or high-scaling screens instead of being forced as wide as one row of buttons.
Hidden widgets take no space (the map shows / hides mode buttons as modes change).
"""
from PySide6.QtCore import Qt, QRect, QSize, QPoint
from PySide6.QtWidgets import QLayout


class FlowLayout(QLayout):
    def __init__(self, parent=None, spacing=6):
        super().__init__(parent)
        self._items = []
        self._spacing = spacing

    # ---- QLayout plumbing
    def addItem(self, item):
        self._items.append(item)

    def count(self):
        return len(self._items)

    def itemAt(self, i):
        return self._items[i] if 0 <= i < len(self._items) else None

    def takeAt(self, i):
        return self._items.pop(i) if 0 <= i < len(self._items) else None

    def expandingDirections(self):
        return Qt.Orientations(0)

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, width):
        return self._do_layout(QRect(0, 0, width, 0), apply=False)

    def setGeometry(self, rect):
        super().setGeometry(rect)
        self._do_layout(rect, apply=True)

    def sizeHint(self):
        # one row of everything visible
        m = self.contentsMargins()
        vis = [it for it in self._items if not it.isEmpty()]
        w = sum(it.sizeHint().width() for it in vis) + self._spacing * max(len(vis) - 1, 0)
        h = max((it.sizeHint().height() for it in vis), default=0)
        return QSize(w + m.left() + m.right(), h + m.top() + m.bottom())

    def minimumSize(self):
        # as narrow as the widest single widget (everything else wraps)
        m = self.contentsMargins()
        size = QSize()
        for it in self._items:
            if not it.isEmpty():
                size = size.expandedTo(it.minimumSize())
        return size + QSize(m.left() + m.right(), m.top() + m.bottom())

    # ---- placement
    def _do_layout(self, rect, apply):
        m = self.contentsMargins()
        r = rect.adjusted(m.left(), m.top(), -m.right(), -m.bottom())
        vis = [it for it in self._items if not it.isEmpty()]
        # break into rows that fit the width
        rows, row, row_w = [], [], 0
        for it in vis:
            w = it.sizeHint().width()
            need = w if not row else row_w + self._spacing + w
            if row and need > r.width():
                rows.append(row); row, row_w = [it], w
            else:
                row.append(it); row_w = need
        if row:
            rows.append(row)
        y = r.y()
        for row in rows:
            h = max(it.sizeHint().height() for it in row)
            if apply:
                x = r.x()
                for it in row:
                    sh = it.sizeHint()
                    it.setGeometry(QRect(QPoint(x, y + (h - sh.height()) // 2), sh))
                    x += sh.width() + self._spacing
            y += h + self._spacing
        return y - self._spacing - rect.y() + m.bottom() if rows else m.top() + m.bottom()
