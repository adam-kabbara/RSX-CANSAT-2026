"""
Fully offline GPS map widget for PyQt6 using Leaflet.js via QWebEngineView.
Plots GPS coordinates (latitude vs longitude) in real-time on an interactive map.
"""

import os
from pathlib import Path
import tomllib

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QApplication
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtCore import QUrl, pyqtSlot, QTimer


_MAP_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.toml"
_DEFAULT_TILE_CHECK_RANGES = {
    12: range(1141, 1143),
    13: range(2283, 2285),
    14: range(4567, 4570),
    15: range(9135, 9140),
    16: range(18271, 18280),
    17: range(36542, 36559),
    18: range(73085, 73117),
    19: range(146170, 146234),
}


class GPSMapWidget(QWidget):
    """
    An embeddable PyQt6 widget that displays a live GPS track using Leaflet.js.
    Completely offline - uses local Leaflet assets and optional local tiles.
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        # --- WebEngine View ---
        self._view = QWebEngineView()

        self._view.page().profile().setHttpAcceptLanguage("en-US,en;q=0.5")
        self._view.page().profile().setHttpUserAgent(
            "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/100.0.4896.127 Safari/537.36"
        )

        # Enable local content access for loading CSS/JS/tiles
        settings = self._view.settings()
        settings.setAttribute(settings.WebAttribute.LocalContentCanAccessFileUrls, True)
        settings.setAttribute(settings.WebAttribute.LocalContentCanAccessRemoteUrls, True)

        # Connect console messages to Python terminal for debugging
        # self._view.page().consoleMessage.connect(self._handle_console_message)
        
        # Determine the path to the HTML file
        base_path = os.path.dirname(os.path.abspath(__file__))
        html_path = os.path.normpath(os.path.join(base_path, "..", "media", "map.html"))
        tiles_path = os.path.normpath(os.path.join(base_path, "..", "media", "tiles"))
        
        if not os.path.exists(html_path):
            # wait until classes are initialized
            QTimer.singleShot(100, lambda: self._log_error(f"ERROR: Map HTML not found at {html_path}"))

        if not os.path.exists(tiles_path):
            QTimer.singleShot(100, lambda: self._log_error(f"ERROR: Map tiles not found at {tiles_path}"))
        else:
            tile_checker_config = self._tile_checker_config()
            QTimer.singleShot(200, lambda: self._check_specific_tiles(tiles_path, tile_checker_config))
        
        # Load the local HTML file
        self._view.setUrl(QUrl.fromLocalFile(html_path))
        
        # Layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._view)

        self._is_initialized = False

    def _handle_console_message(self, level, message, line, sourceID):
        # Forward JavaScript console messages to Python stdout
        # wait until classes are initialized
        QTimer.singleShot(100, lambda: self._log_error(f"GPS Map JS: {message} (line {line})"))

    def add_point(self, lat, lon):
        """Add a new GPS point to the map via JavaScript."""
        # Validate coordinates
        if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
            return

        # Wait until the map is likely loaded or just call it
        # Leaflet script handles its own initialization
        js_code = f"window.addPoint({lat}, {lon});"
        self._view.page().runJavaScript(js_code)

    @staticmethod
    def _build_draw_circle_js(lat, lon, radius):
        if not (-90 <= lat <= 90) or not (-180 <= lon <= 180) or radius <= 0:
            return None
        return f"window.drawGpsCircle({lat}, {lon}, {radius});"

    def draw_circle(self, lat, lon, radius):
        """Draw a single GPS circle overlay and replace the previous one."""
        js_code = self._build_draw_circle_js(lat, lon, radius)
        if js_code is None:
            return False
        self._view.page().runJavaScript(js_code)
        return True

    def reset(self):
        """Clear all GPS data and reset the map view."""
        self._view.page().runJavaScript("window.clearMap();")

    def set_zoom(self, zoom):
        """Set the map zoom level."""
        self._view.page().runJavaScript(f"window.setZoom({zoom});")

    def _log_error(self, msg):
        """Find CommandWindow and log error."""
        for widget in QApplication.topLevelWidgets():
            if type(widget).__name__ == 'CommandWindow':
                widget.update_gui_log_error(msg)
                return

    def _log_info(self, msg):
        """Find CommandWindow and log info."""
        for widget in QApplication.topLevelWidgets():
            if type(widget).__name__ == 'CommandWindow':
                widget.update_gui_log(msg)
                return

    @staticmethod
    def _tile_checker_config():
        try:
            with _MAP_CONFIG_PATH.open("rb") as config_file:
                config = tomllib.load(config_file)
        except (OSError, tomllib.TOMLDecodeError):
            return {}

        map_config = config.get("map", {})
        if not isinstance(map_config, dict):
            return {}

        tile_checker_config = map_config.get("tile_checker", {})
        if not isinstance(tile_checker_config, dict):
            return {}
        return tile_checker_config

    @staticmethod
    def _tile_check_enabled(tile_checker_config):
        enabled = tile_checker_config.get("enabled", True)
        if not isinstance(enabled, bool):
            return True
        return enabled

    @staticmethod
    def _tile_check_ranges(configured_ranges):
        if not isinstance(configured_ranges, dict):
            return _DEFAULT_TILE_CHECK_RANGES

        parsed_ranges = {}
        for zoom, bounds in configured_ranges.items():
            if not isinstance(bounds, list) or len(bounds) != 2:
                return _DEFAULT_TILE_CHECK_RANGES
            start, stop = bounds
            if not isinstance(start, int) or not isinstance(stop, int) or stop < start:
                return _DEFAULT_TILE_CHECK_RANGES

            try:
                zoom_level = int(zoom)
            except (TypeError, ValueError):
                return _DEFAULT_TILE_CHECK_RANGES

            parsed_ranges[zoom_level] = range(start, stop)

        if not parsed_ranges:
            return _DEFAULT_TILE_CHECK_RANGES
        return parsed_ranges

    def _check_specific_tiles(self, tiles_path, tile_checker_config=None):
        """Verify that the expected tile directories exist and contain images."""
        if tile_checker_config is None:
            tile_checker_config = self._tile_checker_config()

        if not self._tile_check_enabled(tile_checker_config):
            return

        expected_structure = self._tile_check_ranges(tile_checker_config.get("ranges"))

        missing_count = 0
        total_checked = 0

        for z, x_range in expected_structure.items():
            z_path = os.path.join(tiles_path, str(z))
            if not os.path.exists(z_path):
                missing_count += len(x_range)
                total_checked += len(x_range)
                continue

            for x in x_range:
                total_checked += 1
                x_path = os.path.join(z_path, str(x))
                if not os.path.exists(x_path):
                    missing_count += 1
                else:
                    try:
                        if not any(f.endswith('.png') for f in os.listdir(x_path)):
                            missing_count += 1
                    except (NotADirectoryError, OSError):
                        missing_count += 1

        if missing_count > 0:
            self._log_error(f"WARNING: {missing_count}/{total_checked} tile directories are missing or empty.")
        else:
            pass
            # self._log_info(f"SUCCESS: All {total_checked} tile directories verified.")
