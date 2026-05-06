import json
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores

def extract_gps():
    bag_path = Path('/home/dragon/DPVO/logs2/sim_bag_20260424-095644')
    coordinates = []
    
    typestore = get_typestore(Stores.LATEST)

    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [x for x in reader.connections if x.topic == '/mavros/global_position/raw/fix']
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            lat, lon = msg.latitude, msg.longitude
            if lat != 0.0 and lon != 0.0:
                coordinates.append([lat, lon])

    if not coordinates:
        print("No GPS coordinates found.")
        return

    # Generate an HTML file with Leaflet
    center_lat = sum(c[0] for c in coordinates) / len(coordinates)
    center_lon = sum(c[1] for c in coordinates) / len(coordinates)

    html_content = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Flight Visualization</title>
        <meta charset="utf-8" />
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.3/dist/leaflet.css"/>
        <script src="https://unpkg.com/leaflet@1.9.3/dist/leaflet.js"></script>
        <style>
            #map {{ height: 100vh; width: 100vw; margin: 0; padding: 0; }}
            body {{ margin: 0; padding: 0; }}
        </style>
    </head>
    <body>
        <div id="map"></div>
        <script>
            var map = L.map('map').setView([{center_lat}, {center_lon}], 15);
            L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
                maxZoom: 19,
                attribution: '&copy; OpenStreetMap contributors'
            }}).addTo(map);

            var pathCoords = {json.dumps(coordinates)};
            var polyline = L.polyline(pathCoords, {{color: 'red', weight: 3}}).addTo(map);
            map.fitBounds(polyline.getBounds());
            
            // Add start and end markers
            if (pathCoords.length > 0) {{
                L.marker(pathCoords[0]).addTo(map).bindPopup("Start");
                L.marker(pathCoords[pathCoords.length - 1]).addTo(map).bindPopup("End");
            }}
        </script>
    </body>
    </html>
    """
    
    out_path = '/home/dragon/vtol_ws/flight_visualization.html'
    with open(out_path, 'w') as f:
        f.write(html_content)
    print(f"Generated visualization: {out_path}")

if __name__ == '__main__':
    extract_gps()
