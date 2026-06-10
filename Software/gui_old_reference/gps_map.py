import os
from flask import Flask, render_template, request, jsonify
import folium

HERE = os.path.dirname(__file__)
app = Flask(__name__, template_folder=HERE)

@app.route('/')
def index():
    return render_template('index.html')

# TODO: Add the function below for live GPS coordinates
# @app.route('/map', methods=['POST'])
# def map_view):
#     try:
#         latitude = float(request.form.get('latitude', 40))
#         longitude = float(request.form.get('longitude', -74))
#     except (TypeError, ValueError):
#         latitude, longitude = 40, -74

#     # Create a Folium map centered on the provided location
#     m = folium.Map(location=[latitude, longitude], zoom_start=0)
#     folium.Marker([latitude, longitude], popup='You are here').add_to(m)

#     # Save map to an HTML file inside the configured template folder so
#     # `render_template('map.html')` can find it.
#     map_html = os.path.join(app.template_folder, "map.html")
#     m.save(map_html)

#     return render_template('map.html')


# Demo coordinates: a short track near San Francisco as an example.
DEMO_COORDS = [
    {"lat": 37.7749, "lon": -122.4194},
    {"lat": 37.7755, "lon": -122.4185},
    {"lat": 37.7760, "lon": -122.4170},
    {"lat": 37.7765, "lon": -122.4160},
    {"lat": 37.7770, "lon": -122.4150},
    {"lat": 37.7775, "lon": -122.4140},
    {"lat": 37.7780, "lon": -122.4130},
]


@app.route('/coords')
def coords():
    """Return a demo list of coordinates as JSON.

    In a real setup you could stream or return live data here. The client
    will fetch this list and animate one point per second.
    """
    return jsonify(DEMO_COORDS)

def main(host: str = '127.0.0.1', port: int = 5000):
    app.run(debug=True, host=host, port=port)


if __name__ == '__main__':
    main()