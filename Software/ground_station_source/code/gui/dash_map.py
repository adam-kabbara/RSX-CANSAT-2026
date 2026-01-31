import dash
from dash import dcc, html
from dash.dependencies import Input, Output, State
import plotly.graph_objects as go
import logging
import math

# --- Shared Data Storage ---
gps_latitudes = []
gps_longitudes = []
web_port = 18050

# Track the current view mode: 'ALL' or 'TAIL'
# Default to 'ALL' (Show entire mission)
current_view_mode = 'ALL'


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
    lat_diff = max(abs(max_lat - min_lat), 0.0001)
    lon_diff = max(abs(max_lon - min_lon), 0.0001)
    max_diff = max(lat_diff, lon_diff)

    # Mapbox Zoom Formula
    zoom = math.log2(360 / max_diff) - 1.5
    zoom = max(2, min(zoom, 18))

    return zoom, center


# --- Dash App Setup ---
app = dash.Dash(__name__)

# Silence 'werkzeug' logger
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# Define shared button style for consistency
BUTTON_STYLE = {
    'padding': '4px 8px',  # Smaller padding
    'fontSize': '12px',  # Smaller font
    'fontWeight': 'bold',
    'backgroundColor': 'white',
    'border': '1px solid #ccc',
    'cursor': 'pointer'
}

app.layout = html.Div([
    dcc.Graph(
        id='live-map',
        style={'height': '100vh', 'width': '100%'},
        # config scrollZoom allows user interaction, but our callback will snap it back
        config={'scrollZoom': True}
    ),

    # Control Buttons Container
    html.Div([
        html.Button('Show All', id='btn-show-all', n_clicks=0, style={
            **BUTTON_STYLE,
            'marginRight': '5px'
        }),
        html.Button('Trace Last 3', id='btn-follow-tail', n_clicks=0, style={
            **BUTTON_STYLE
        }),
    ], style={'position': 'absolute', 'top': '10px', 'left': '10px', 'zIndex': '1000'}),

    dcc.Interval(
        id='interval-component',
        interval=1000,  # Update every 1000ms (1 second)
        n_intervals=0
    )
])


@app.callback(
    [Output('live-map', 'figure'),
     Output('btn-show-all', 'style'),
     Output('btn-follow-tail', 'style')],
    [Input('interval-component', 'n_intervals'),
     Input('btn-show-all', 'n_clicks'),
     Input('btn-follow-tail', 'n_clicks')],
    [State('btn-show-all', 'style'),
     State('btn-follow-tail', 'style')]
)
def update_graph_live(n, btn_all_clicks, btn_tail_clicks, style_all, style_tail):
    global current_view_mode

    # 1. Determine Trigger & Update Mode
    ctx = dash.callback_context
    if ctx.triggered:
        prop_id = ctx.triggered[0]['prop_id']

        if 'btn-show-all' in prop_id:
            current_view_mode = 'ALL'
        elif 'btn-follow-tail' in prop_id:
            current_view_mode = 'TAIL'

    # 2. Prepare Data for Zoom Calculation
    target_lats = []
    target_lons = []

    if current_view_mode == 'TAIL':
        # Slice the last 3 points
        if len(gps_latitudes) > 0:
            target_lats = gps_latitudes[-3:]
            target_lons = gps_longitudes[-3:]
    else:
        # Default/ALL uses all points
        target_lats = gps_latitudes
        target_lons = gps_longitudes

    # 3. Calculate View (Zoom & Center)
    zoom_level, center_coords = calculate_zoom(target_lats, target_lons)

    # 4. Build Mapbox Config
    mapbox_config = dict(
        style="open-street-map",
        center=dict(lat=center_coords[0], lon=center_coords[1]),
        zoom=zoom_level
    )

    # 5. Create Figure
    fig = go.Figure(go.Scattermapbox(
        lat=gps_latitudes,
        lon=gps_longitudes,
        mode='markers+lines',  # <--- ENSURES POINTS ARE CONNECTED
        marker=go.scattermapbox.Marker(
            size=6,  # <--- SMALLER POINT SIZE (was 10)
            color='red',
            opacity=0.8
        ),
        line=dict(width=2, color='blue'),
        text=[f"Pt {i}" for i in range(len(gps_latitudes))],
    ))

    fig.update_layout(
        margin={'l': 0, 't': 0, 'b': 0, 'r': 0},
        mapbox=mapbox_config,
        showlegend=False,
        uirevision=n  # Force update every tick
    )

    # 6. Update Button Styles
    base_style_all = BUTTON_STYLE.copy()
    base_style_all['marginRight'] = '5px'

    base_style_tail = BUTTON_STYLE.copy()

    active_style = {'backgroundColor': '#90EE90', 'border': '2px solid green'}

    if current_view_mode == 'ALL':
        base_style_all.update(active_style)
    elif current_view_mode == 'TAIL':
        base_style_tail.update(active_style)

    return fig, base_style_all, base_style_tail


def run_dash_server():
    try:
        app.run(
            debug=False,
            use_reloader=False,
            port=web_port,
            log_startup=False
        )
    except TypeError:
        app.run(debug=False, use_reloader=False, port=web_port)
    except Exception as e:
        print(f"Dash Server Error: {e}")