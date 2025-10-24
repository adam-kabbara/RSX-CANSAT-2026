"""
Manage cosmetic settings for the GUI
"""

from PyQt6.QtGui import QFont, QColor, QPalette
from PyQt6.QtWidgets import QApplication

def set_app_style(app: QApplication):
    app.setStyle('Fusion')
    app.setPalette(customPalette())

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

def log_font():
    log_font = QFont()
    log_font.setPointSize(14)
    log_font.setWeight(QFont.Weight.DemiBold)
    return log_font

def sidebar_title_font():
    sidebar_title_font = QFont()
    sidebar_title_font.setPointSize(14)
    sidebar_title_font.setWeight(QFont.Weight.DemiBold)
    return sidebar_title_font

def graph_font():
    graph_font = QFont("Roboto Mono")
    graph_font.setPointSize(14)
    graph_font.setWeight(QFont.Weight.Bold)
    return graph_font

def graph_background():
    return 'W'

def graph_title(text):
    return f'<span style="font-family: Monospace; font-size:14pt; font-weight:bold;">{text}</span>'

def sidebar_data_font():
    live_graph_data_font = QFont("Roboto Mono")
    live_graph_data_font.setPointSize(14)
    return live_graph_data_font

def sidebar_field_font():
    live_graph_field_font = QFont()
    live_graph_field_font.setPointSize(14)
    return live_graph_field_font

def graph_pen_color(line_num):
    match line_num:
        case 0:
            return (255, 0, 0)
        case 1:
            return (0, 255, 0)
        case 2:
            return (0, 0, 255)
        case _:
            return (144,144,144)

def graph_pen_size():
    return 3

def set_label_default(text):
    return f'<span style="color:black;">{text} \
             </span><span style="color:GREY;">N/A</span>'

def servo_val_stylesheet():
    return """
            QLineEdit {
                background-color: #f0f0f0;
                border: 1px solid #cccccc;
                border-radius: 10px;
                padding: 4px;
                font-size: 14px;
            }
            
            QLineEdit:focus {
                border: 1px solid #0078d4;
                background-color: #ffffff;
            }
        """

def team_id_stylesheet():
    return """
            QLineEdit {
                background-color: #f0f0f0;
                border: 1px solid #cccccc;
                border-radius: 10px;
                padding: 4px;
                font-size: 14px;
            }
            
            QLineEdit:focus {
                border: 1px solid #0078d4;
                background-color: #ffffff;
            }
        """

def log_overlay_stylesheet():
    return """
            background-color: rgba(0, 0, 0, 215);
            color: white;
            font-size: 18px;
        """

def log_stylesheet():
    return """
            QTextEdit {
                font-size: 18px;
                background-color: #dcdcdc;
                border-radius: 6px;
                padding: 3px;
                font-family: monospace;
            }
        """

def customPalette():

    palette = QPalette()

    palette.setColor(QPalette.ColorRole.WindowText, QColor("#000000"))
    palette.setColor(QPalette.ColorRole.Button, QColor("#f0f0f0"))
    palette.setColor(QPalette.ColorRole.Light, QColor("#ffffff"))
    palette.setColor(QPalette.ColorRole.Midlight, QColor("#e3e3e3"))
    palette.setColor(QPalette.ColorRole.Dark, QColor("#a0a0a0"))
    palette.setColor(QPalette.ColorRole.Mid, QColor("#a0a0a0"))
    palette.setColor(QPalette.ColorRole.Text, QColor("#000000"))
    palette.setColor(QPalette.ColorRole.BrightText, QColor("#ffffff"))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor("#000000"))
    palette.setColor(QPalette.ColorRole.Base, QColor("#ffffff"))
    palette.setColor(QPalette.ColorRole.Window, QColor("#f0f0f0"))
    palette.setColor(QPalette.ColorRole.Shadow, QColor("#696969"))
    palette.setColor(QPalette.ColorRole.Highlight, QColor("#0078d7"))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor("#ffffff"))
    palette.setColor(QPalette.ColorRole.Link, QColor("#006770"))
    palette.setColor(QPalette.ColorRole.LinkVisited, QColor("#00343b"))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor("#e9e7e3"))
    palette.setColor(QPalette.ColorRole.ToolTipBase, QColor("#ffffdc"))
    palette.setColor(QPalette.ColorRole.ToolTipText, QColor("#000000"))
    palette.setColor(QPalette.ColorRole.PlaceholderText, QColor("#000000"))
    palette.setColor(QPalette.ColorRole.Accent, QColor("#009faa"))

    return palette