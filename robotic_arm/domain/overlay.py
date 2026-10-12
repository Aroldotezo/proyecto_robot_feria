from typing import Protocol

from PySide6.QtGui import QPainter

class Overlay(Protocol):
    def paint(self, painter: QPainter, width: int, height: int) -> None:
        """Draws over the displayed frame; (width, height) is the size of the displayed image."""
