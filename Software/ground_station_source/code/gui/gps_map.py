"""
Fully offline GPS map widget for PyQt6 using Leaflet.js via QWebEngineView.
Plots GPS coordinates (latitude vs longitude) in real-time on an interactive map.
"""

import os
from PyQt6.QtWidgets import QWidget, QVBoxLayout
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtCore import QUrl, pyqtSlot


class GPSMapWidget(QWidget):
    """
    An embeddable PyQt6 widget that displays a live GPS track using Leaflet.js.
    Completely offline - uses local Leaflet assets and optional local tiles.
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        # --- WebEngine View ---
        self._view = QWebEngineView()
        
        # Connect console messages to Python terminal for debugging
        # self._view.page().consoleMessage.connect(self._handle_console_message)
        
        # Determine the path to the HTML file
        base_path = os.path.dirname(os.path.abspath(__file__))
        html_path = os.path.normpath(os.path.join(base_path, "..", "media", "map.html"))
        
        if not os.path.exists(html_path):
            print(f"CRITICAL: Map HTML not found at {html_path}")
        
        # Load the local HTML file
        self._view.setUrl(QUrl.fromLocalFile(html_path))
        
        # Layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._view)

        self._is_initialized = False

    def _handle_console_message(self, level, message, line, sourceID):
        # Forward JavaScript console messages to Python stdout
        print(f"GPS Map JS: {message} (line {line})")

    def add_point(self, lat, lon):
        """Add a new GPS point to the map via JavaScript."""
        # Validate coordinates
        if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
            return

        # Wait until the map is likely loaded or just call it
        # Leaflet script handles its own initialization
        js_code = f"window.addPoint({lat}, {lon});"
        self._view.page().runJavaScript(js_code)

    def reset(self):
        """Clear all GPS data and reset the map view."""
        self._view.page().runJavaScript("window.clearMap();")

    def set_zoom(self, zoom):
        """Set the map zoom level."""
        self._view.page().runJavaScript(f"window.setZoom({zoom});")

