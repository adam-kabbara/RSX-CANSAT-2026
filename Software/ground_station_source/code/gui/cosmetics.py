"""
Manage cosmetic settings for the GUI
"""

from pathlib import Path
import tomllib

from PyQt6.QtGui import QFont, QColor, QPalette
from PyQt6.QtWidgets import QApplication, QListWidgetItem

_COLOR_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.toml"
_DEFAULT_MODE = "day"
_DEFAULT_WINDOW_RESOLUTION = (800, 600)
_DEFAULT_WINDOW_DISPLAY_MODE = "fullscreen"
_WINDOW_DISPLAY_MODES = {"fullscreen", "borderless"}
_COLOR_CONFIG = None


def _load_color_config():
    global _COLOR_CONFIG

    if _COLOR_CONFIG is None:
        with _COLOR_CONFIG_PATH.open("rb") as config_file:
            _COLOR_CONFIG = tomllib.load(config_file)
    return _COLOR_CONFIG


def _active_mode():
    config = _load_color_config()
    night_mode = config.get("night_mode", False)
    high_contrast_mode = config.get("high_contrast_mode", False)

    if night_mode and high_contrast_mode:
        return "black_high_contrast"
    if night_mode:
        return "night"
    if high_contrast_mode:
        return "high_contrast"
    return _DEFAULT_MODE


def _theme_section(section):
    config = _load_color_config()
    return config["modes"][_active_mode()].get(section, {})


def _window_section(window_name):
    config = _load_color_config()
    return config.get("windows", {}).get(window_name, {})


def theme_color(section, key, default):
    return _theme_section(section).get(key, default)


def theme_rgb(section, key, default):
    value = theme_color(section, key, default)
    return tuple(value)


def theme_qcolor(section, key, default):
    value = theme_color(section, key, default)
    if isinstance(value, list):
        return QColor(*value)
    return QColor(value)


def window_resolution(window_name):
    resolution = _window_section(window_name).get("resolution", _DEFAULT_WINDOW_RESOLUTION)
    if not isinstance(resolution, (list, tuple)) or len(resolution) != 2:
        return _DEFAULT_WINDOW_RESOLUTION
    return int(resolution[0]), int(resolution[1])


def window_display_mode(window_name):
    display_mode = str(_window_section(window_name).get("display_mode", _DEFAULT_WINDOW_DISPLAY_MODE)).lower()
    if display_mode not in _WINDOW_DISPLAY_MODES:
        return _DEFAULT_WINDOW_DISPLAY_MODE
    return display_mode


def window_settings(window_name):
    return {
        "resolution": window_resolution(window_name),
        "display_mode": window_display_mode(window_name),
    }


def set_app_style(app: QApplication):
    app.setStyle('Fusion')
    app.setPalette(customPalette())

def gui_log_normal_color():
    return theme_color("logs", "gui_normal", "black")

def gui_log_error_color():
    return theme_color("logs", "gui_error", "red")

def sat_log_normal_color():
    return theme_color("logs", "sat_normal", "black")

def sat_log_error_color():
    return theme_color("logs", "sat_error", "red")

def data_status_init_color(text):
    return f'<span style="color:{theme_color("status", "init", "GREY")};">{text}'

def data_status_red(text):
    return f'<span style="color:{theme_color("status", "red", "RED")};">{text}' 

def data_status_green(text):
    return f'<span style="color:{theme_color("status", "green", "GREEN")};">{text}'

def data_status_blue(text):
    return f'<span style="color:{theme_color("status", "blue", "BLUE")};">{text}'

def state_label_font():
    return QFont("Roboto Mono", 14)

def set_previous_states(item: QListWidgetItem):
    item.setFont(QFont("Consolas", 14))
    item.setForeground(theme_qcolor("status", "previous_state", "green"))

def set_current_states(item: QListWidgetItem):
    item.setFont(QFont("Consolas", 14))
    item.setForeground(theme_qcolor("status", "current_state", "blue"))

def set_skipped_states(item: QListWidgetItem):
    item.setFont(QFont("Consolas", 14))
    item.setForeground(theme_qcolor("status", "skipped_state", "red"))

def set_next_states(item: QListWidgetItem):
    item.setFont(QFont("Consolas", 14))
    item.setForeground(theme_qcolor("status", "next_state", "grey"))

def graph_font():
    font = QFont("Roboto Mono")
    font.setPointSize(14)
    font.setWeight(QFont.Weight.Bold)
    return font

def plot_title_style(text):
    return f'<span style="font-family: Monospace; font-size:14pt; font-weight:bold;">{text}</span>'

def button_font():
    button_font = QFont()
    button_font.setPointSize(14)
    button_font.setWeight(QFont.Weight.Medium)
    return button_font

def command_font():
    command_status_font = QFont()
    command_status_font.setPointSize(14)
    command_status_font.setWeight(QFont.Weight.Medium)
    return command_status_font

def credit_font():
    credit_font = QFont("Courier New")
    credit_font.setPointSize(10)
    return credit_font

def state_title_font():
    state_title_font = QFont()
    state_title_font.setPointSize(14)
    state_title_font.setWeight(QFont.Weight.DemiBold)
    return state_title_font

def sidebar_group_box_stylesheet():
    border = theme_color("palette", "Mid", "#a0a0a0")
    background = theme_color("palette", "Window", "#f0f0f0")
    text = theme_color("palette", "WindowText", "#000000")
    return f"""
            QGroupBox {{
                border: 1px solid {border};
                margin-top: 12px;
                padding-top: 10px;
            }}

            QGroupBox::title {{
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 0 8px;
                background-color: {background};
                color: {text};
                font-size: 14pt;
                font-weight: 600;
            }}
        """

def log_font():
    log_font = QFont()
    log_font.setPointSize(14)
    log_font.setWeight(QFont.Weight.DemiBold)
    return log_font

def graph_font():
    graph_font = QFont("Consolas")
    graph_font.setPointSize(14)
    graph_font.setWeight(QFont.Weight.DemiBold)
    return graph_font

def graph_background():
    return theme_color("graph", "background", "w")

def graph_title(text):
    color = theme_color("graph", "axis_text", "black")
    return f'<span style="color:{color}; font-family: Monospace; font-size:14pt; font-weight:bold;">{text}</span>'

def graph_axis(text):
    color = theme_color("graph", "axis_text", "black")
    return f'<span style="color:{color}; font-family: Consolas; font-size:14pt; font-weight:bold;">{text}</span>'

def sidebar_data_font():
    live_graph_data_font = QFont("Consolas")
    live_graph_data_font.setPointSize(14)
    return live_graph_data_font

def sidebar_field_font():
    live_graph_field_font = QFont()
    live_graph_field_font.setPointSize(14)
    return live_graph_field_font

def graph_pen_color(line_num):
    pen_colors = _theme_section("graph").get(
        "pen_colors",
        [(255, 0, 0), (0, 255, 0), (0, 0, 255), (144, 144, 144)],
    )
    index = line_num if line_num < len(pen_colors) else len(pen_colors) - 1
    return tuple(pen_colors[index])

def graph_pen_size():
    return 3

def set_label_default(text):
    label_color = theme_color("status", "label_text", "black")
    init_color = theme_color("status", "init", "GREY")
    return f'<span style="color:{label_color};">{text} \
             </span><span style="color:{init_color};">N/A</span>'

def transparent_list_stylesheet():
    color = theme_color("status", "label_text", "black")
    return f"background-color: transparent; color:{color}; font-size: 10px; font-family: Roboto Mono;"

def current_state_stylesheet():
    color = theme_color("status", "current_state", "blue")
    return f"color: {color}; padding: 5px;"

def servo_val_stylesheet():
    return f"""
            QLineEdit {{
                background-color: {theme_color("inputs", "background", "#f0f0f0")};
                border: 1px solid {theme_color("inputs", "border", "#cccccc")};
                border-radius: 10px;
                padding: 4px;
                font-size: 14px;
            }}
            
            QLineEdit:focus {{
                border: 1px solid {theme_color("inputs", "focus_border", "#0078d4")};
                background-color: {theme_color("inputs", "focus_background", "#ffffff")};
            }}
        """

def team_id_stylesheet():
    return f"""
            QLineEdit {{
                background-color: {theme_color("inputs", "background", "#f0f0f0")};
                border: 1px solid {theme_color("inputs", "border", "#cccccc")};
                border-radius: 10px;
                padding: 4px;
                font-size: 14px;
            }}
            
            QLineEdit:focus {{
                border: 1px solid {theme_color("inputs", "focus_border", "#0078d4")};
                background-color: {theme_color("inputs", "focus_background", "#ffffff")};
            }}
        """

def log_overlay_stylesheet():
    return f"""
            background-color: {theme_color("logs", "overlay_background", "rgba(0, 0, 0, 215)")};
            color: {theme_color("logs", "overlay_text", "white")};
            font-size: 18px;
        """

def log_stylesheet():
    return f"""
            QTableWidget {{
                font-size: 18px;
                background-color: {theme_color("logs", "background", "#dcdcdc")};
                border-radius: 6px;
                padding: 3px;
            }}
        """

def customPalette():

    palette = QPalette()

    for role_name, color in _theme_section("palette").items():
        role = getattr(QPalette.ColorRole, role_name, None)
        if role is not None:
            palette.setColor(role, QColor(color))

    return palette
