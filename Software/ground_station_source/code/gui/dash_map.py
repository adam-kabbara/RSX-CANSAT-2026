import sys

import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objects as go
import threading
import logging

gps_latitudes = []
gps_longitudes = []
web_port = 18050

def add_new_point(lat, lng):
    """external to update new data."""
    gps_latitudes.append(lat)
    gps_longitudes.append(lng)


# --- Dash App Setup ---
app = dash.Dash(__name__)
dash_log = logging.getLogger('dash')
# dash_log.disabled = True
dash_log.setLevel(logging.ERROR)

werkzeug_log = logging.getLogger('werkzeug')
werkzeug_log.disabled = True
werkzeug_log.setLevel(logging.ERROR)
sys.modules['flask.cli'].show_server_banner = lambda *x: None # Disable terminal flask output

app.layout = html.Div([
    dcc.Graph(id='live-map', animate=True),
    dcc.Interval(
        id='interval-component',
        interval=1000,  # Update every 1000ms (1 second)
        n_intervals=0
    )
])


@app.callback(Output('live-map', 'figure'),
              [Input('interval-component', 'n_intervals')])
def update_graph_live(n):
    # Determine center, default to Rotman
    if gps_latitudes:
        center_lat = gps_latitudes[-1]
        center_lon = gps_longitudes[-1]
    else:
        center_lat = 43.66541097463245
        center_lon = -79.3982646510055

    # Create the Plotly Mapbox figure
    fig = go.Figure(go.Scattermapbox(
        lat=gps_latitudes,
        lon=gps_longitudes,
        mode='markers',  # Display as points
        marker=go.scattermapbox.Marker(
            size=9,
            color='red',
            opacity=0.8
        ),
        text=[f"Pt {i}" for i in range(len(gps_latitudes))],
    ))

    fig.update_layout(
        margin={'l': 0, 't': 0, 'b': 0, 'r': 0},
        mapbox=dict(
            style="open-street-map",
            center=dict(lat=center_lat, lon=center_lon),
            zoom=15
        ),
        showlegend=False
    )

    return fig


def run_dash_server():
    """Starts the Dash server."""
    try:
        app.run(debug=False, use_reloader=False, port=web_port)
    except Exception as e:
        print(f"Dash Server Error: {e}")