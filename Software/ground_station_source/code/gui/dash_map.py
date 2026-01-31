import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objects as go
import logging
import math

# --- Shared Data Storage ---
gps_latitudes = []
gps_longitudes = []
web_port = 18050

def add_new_point(lat, lng):
    """Called by process.py to push new data."""
    gps_latitudes.append(lat)
    gps_longitudes.append(lng)


def calculate_zoom(lats, lons):
    """
    Calculates the optimal zoom level and center point to fit all provided coordinates.
    """
    if not lats or not lons:
        return 15, (43.664781, -79.398232)  # Default to Toronto if empty

    max_lat, min_lat = max(lats), min(lats)
    max_lon, min_lon = max(lons), min(lons)

    # Calculate Center
    center_lat = (max_lat + min_lat) / 2
    center_lon = (max_lon + min_lon) / 2
    center = (center_lat, center_lon)

    # Calculate Zoom
    # Get the larger span (lat or lon)
    lat_diff = max(abs(max_lat - min_lat), 0.0001)  # Avoid 0 division
    lon_diff = max(abs(max_lon - min_lon), 0.0001)
    max_diff = max(lat_diff, lon_diff)

    # Mapbox Zoom Formula: zoom = log2(360 / diff) - padding
    # 360 degrees is zoom 0.
    # We subtract 1.5 for padding so points aren't on the very edge.
    zoom = math.log2(360 / max_diff) - 1.5

    # Clamp zoom to reasonable limits (CanSat missions usually need 12-19)
    zoom = max(2, min(zoom, 18))

    return zoom, center


# --- Dash App Setup ---
app = dash.Dash(__name__)

# 1. Silence the 'werkzeug' logger to hide request logs (e.g., "GET / ... 200")
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

app.layout = html.Div([
    dcc.Graph(id='live-map', style={'height': '100vh', 'width': '100%'}),
    dcc.Interval(
        id='interval-component',
        interval=1000,  # Update every 1000ms (1 second)
        n_intervals=0
    )
])


@app.callback(Output('live-map', 'figure'),
              [Input('interval-component', 'n_intervals')])
def update_graph_live(n):
    # 1. Calculate dynamic zoom and center based on ALL points
    zoom_level, center_coords = calculate_zoom(gps_latitudes, gps_longitudes)

    # 2. Create the Map Figure
    fig = go.Figure(go.Scattermapbox(
        lat=gps_latitudes,
        lon=gps_longitudes,
        mode='markers+lines',  # Lines connect the points
        marker=go.scattermapbox.Marker(
            size=10,
            color='red',
            opacity=0.8
        ),
        line=dict(width=2, color='blue'),
        text=[f"Pt {i}" for i in range(len(gps_latitudes))],
    ))

    # 3. Apply the calculated View
    fig.update_layout(
        margin={'l': 0, 't': 0, 'b': 0, 'r': 0},
        mapbox=dict(
            style="open-street-map",
            center=dict(lat=center_coords[0], lon=center_coords[1]),
            zoom=zoom_level  # Apply the calculated zoom
        ),
        showlegend=False,
        # Setting uirevision to 'n' ensures the zoom updates automatically
        # when n_intervals ticks up.
        uirevision=n
    )

    return fig


def run_dash_server():
    try:
        # 2. Pass log_startup=False to hide the "Dash is running on..." banner
        # Note: This requires Werkzeug 2.1+ (standard in modern installs)
        app.run(
            debug=False,
            use_reloader=False,
            port=web_port,
            log_startup=False
        )
    except TypeError:
        # Fallback for older versions that don't support log_startup
        app.run(debug=False, use_reloader=False, port=web_port)
    except Exception as e:
        print(f"Dash Server Error: {e}")