"""
Map Tile Downloader Tool
Allows users to select an area on a Leaflet map and download map tiles from Google Maps.

Features:
- Interactive web-based area selection using Leaflet
- Multi-threaded downloads (8 threads by default, configurable)
- Real-time progress tracking via HTML dashboard
- Export tile URLs for IDM/FDM/other download managers
- Automatic skipping of existing tiles
- Multiple export formats (simple URLs, IDM batch, FDM list, etc.)

Usage Examples:

1. Interactive mode (recommended):
   python tiles.py
   Choose option 1, draw area on map, click "Copy Python Code"

2. Programmatic usage:
   from tiles import get_tiles_for_bbox, generate_tile_urls, download_tiles

   tiles_by_zoom = get_tiles_for_bbox(
       lat1=38.37, lon1=-79.61, lat2=38.38, lon2=-79.60,
       zoom_levels=[12, 13, 14, 15, 16]
   )
   tile_urls = generate_tile_urls(tiles_by_zoom)
   download_tiles(tile_urls, num_threads=8)

3. Export for download managers:
   from tiles import export_for_download_manager

   export_for_download_manager(
       tile_urls,
       output_file="tiles.txt",
       format="simple"  # or "idm_batch", "fdm_list", "with_paths"
   )
"""

import json
import math
import tempfile
import threading
import time
import urllib.request
import webbrowser
from concurrent.futures import ThreadPoolExecutor, as_completed
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path


def deg2num(lat_deg, lon_deg, zoom):
    """Convert lat/lon to tile coordinates"""
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return (xtile, ytile)


def num2deg(xtile, ytile, zoom):
    """Convert tile coordinates to lat/lon"""
    n = 2.0 ** zoom
    lon_deg = xtile / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
    lat_deg = math.degrees(lat_rad)
    return (lat_deg, lon_deg)


def get_tiles_for_bbox(lat1, lon1, lat2, lon2, zoom_levels):
    """
    Calculate all tile coordinates needed for a bounding box at given zoom levels.

    Args:
        lat1, lon1: Southwest corner coordinates
        lat2, lon2: Northeast corner coordinates
        zoom_levels: List of zoom levels to download (e.g., [12, 13, 14, 15])

    Returns:
        Dictionary with zoom levels as keys and list of (x, y) tile coordinates as values
    """
    # Ensure correct order (sw to ne)
    min_lat = min(lat1, lat2)
    max_lat = max(lat1, lat2)
    min_lon = min(lon1, lon2)
    max_lon = max(lon1, lon2)

    tiles_by_zoom = {}

    for zoom in zoom_levels:
        x1, y1 = deg2num(max_lat, min_lon, zoom)  # NW corner
        x2, y2 = deg2num(min_lat, max_lon, zoom)  # SE corner

        # Ensure correct order
        min_x = min(x1, x2)
        max_x = max(x1, x2)
        min_y = min(y1, y2)
        max_y = max(y1, y2)

        tiles = []
        for x in range(min_x, max_x + 1):
            for y in range(min_y, max_y + 1):
                tiles.append((x, y))

        tiles_by_zoom[zoom] = tiles

    return tiles_by_zoom


def generate_tile_urls(tiles_by_zoom, server_url="https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}"):
    """
    Generate download URLs for all tiles.

    Args:
        tiles_by_zoom: Dictionary from get_tiles_for_bbox()
        server_url: URL template for tile server (default: Google satellite + labels)

    Returns:
        List of tuples: (url, local_path, zoom, x, y)
    """
    urls = []

    for zoom, tiles in tiles_by_zoom.items():
        for x, y in tiles:
            url = server_url.format(x=x, y=y, z=zoom)
            local_path = f"{zoom}/{x}/{y}.png"
            urls.append((url, local_path, zoom, x, y))

    return urls


def export_for_download_manager(tile_urls, output_file="tile_download_list.txt", format="simple", serve_via_http=True, port=8766):
    """
    Export tile URLs to a file compatible with download managers (IDM, FDM, etc.)
    and optionally serve it via HTTP for easy browser download.

    Args:
        tile_urls: List from generate_tile_urls()
        output_file: Output filename
        format: Export format
            - "simple": One URL per line (most compatible)
            - "with_paths": URL and local path on each line
            - "idm_batch": IDM batch format with filenames
            - "idm_cmd": Windows .bat script that queues downloads in IDM with folder structure
            - "fdm_list": Free Download Manager format
        serve_via_http: If True, start HTTP server and serve file for download
        port: Port for HTTP server (default: 8766)

    Returns:
        Path to the exported file
    """
    # Use temp directory for the file
    temp_dir = Path(tempfile.gettempdir()) / "tile_exports"
    temp_dir.mkdir(exist_ok=True)
    output_path = temp_dir / output_file

    total = len(tile_urls)

    print(f"\n📝 Exporting {total} tile URLs to {output_path}")
    print(f"   Format: {format}")

    with open(output_path, 'w', encoding='utf-8') as f:
        if format == "simple":
            # Just URLs, one per line - works with most download managers
            for url, _, _, _, _ in tile_urls:
                f.write(f"{url}\n")

        elif format == "with_paths":
            # URL followed by local path
            for url, local_path, _, _, _ in tile_urls:
                f.write(f"{url}\t{local_path}\n")

        elif format == "idm_batch":
            # IDM batch download format
            f.write("# Internet Download Manager Batch File\n")
            f.write(f"# Generated by Map Tile Downloader\n")
            f.write(f"# Total tiles: {total}\n\n")
            for url, local_path, zoom, x, y in tile_urls:
                filename = f"{zoom}_{x}_{y}.png"
                f.write(f"<\n")
                f.write(f"{url}\n")
                f.write(f"file={filename}\n")
                f.write(f">\n")

        elif format == "idm_cmd":
            # Windows batch script: queue each URL in IDM and preserve tiles/z/x/y.png structure
            f.write("@echo off\n")
            f.write("setlocal enabledelayedexpansion\n\n")
            f.write("set \"IDM=C:\\Program Files (x86)\\Internet Download Manager\\IDMan.exe\"\n")
            f.write("if not exist \"%IDM%\" set \"IDM=C:\\Program Files\\Internet Download Manager\\IDMan.exe\"\n")
            f.write("if not exist \"%IDM%\" (\n")
            f.write("  echo ERROR: IDMan.exe not found. Update IDM path in this script.\n")
            f.write("  pause\n")
            f.write("  exit /b 1\n")
            f.write(")\n\n")
            f.write("set \"BASE=media\"\n")
            f.write("echo Queueing downloads to IDM...\n")

            created_dirs = set()
            for url, local_path, _, _, _ in tile_urls:
                normalized_path = local_path.replace('/', '\\\\')
                folder_path, filename = normalized_path.rsplit('\\\\', 1)
                full_dir = f"%BASE%\\\\{folder_path}"

                if full_dir not in created_dirs:
                    f.write(f"if not exist \"{full_dir}\" mkdir \"{full_dir}\"\n")
                    created_dirs.add(full_dir)

                f.write(f"\"%IDM%\" /d \"{url}\" /p \"{full_dir}\" /f \"{filename}\" /n /a\n")

            f.write("\n\"%IDM%\" /s\n")
            f.write("echo Done. Downloads started in IDM.\n")
            f.write("endlocal\n")

        elif format == "fdm_list":
            # Free Download Manager format (URL per line with optional comments)
            f.write(f"# Free Download Manager Download List\n")
            f.write(f"# Generated by Map Tile Downloader\n")
            f.write(f"# Total tiles: {total}\n\n")
            for url, local_path, zoom, x, y in tile_urls:
                f.write(f"{url}\n")
                f.write(f"# Save as: {local_path}\n")

    print(f"✅ Export complete!")

    if serve_via_http:
        # Start HTTP server to serve the file
        print(f"\n🌐 Starting HTTP server to serve the file...")
        server = launch_file_server(output_path, output_file, port)
        print(f"\n📋 Import instructions:")
        print(f"   1. Download the file from the browser")
        print(f"   2. Import into IDM: File → Import → Import from text file")
        print(f"   3. Import into FDM: Downloads → Import → Import from text file")
        print(f"   Note: You may need to manually set the download directory to 'media/tiles'")
        print(f"\n⚡ Press Ctrl+C to stop the server")

        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print("\n🛑 File server stopped")
            server.shutdown()
    else:
        print(f"\n📋 Import instructions:")
        print(f"   IDM: File → Import → Import from text file → Select {output_path}")
        print(f"   FDM: Downloads → Import → Import from text file → Select {output_path}")
        print(f"   Note: You may need to manually set the download directory to 'media/tiles'")

    return str(output_path)


class ProgressTracker:
    """Thread-safe progress tracker for tile downloads (in-memory, no file I/O)"""
    def __init__(self, total):
        self.total = total
        self.downloaded = 0
        self.skipped = 0
        self.failed = 0
        self.lock = threading.Lock()
        self.failed_tiles = []
        self.start_time = time.time()

    def increment(self, status='downloaded'):
        """Increment counter based on status: 'downloaded', 'skipped', or 'failed'"""
        with self.lock:
            if status == 'downloaded':
                self.downloaded += 1
            elif status == 'skipped':
                self.skipped += 1
            elif status == 'failed':
                self.failed += 1

    def add_failed(self, zoom, x, y, error):
        """Add a failed tile"""
        with self.lock:
            self.failed_tiles.append({'zoom': zoom, 'x': x, 'y': y, 'error': str(error)})

    def as_dict(self):
        """Return current progress as a serialisable dict (thread-safe snapshot)"""
        with self.lock:
            elapsed = time.time() - self.start_time
            completed = self.downloaded + self.skipped + self.failed
            progress_pct = (completed / self.total * 100) if self.total > 0 else 0

            if 0 < completed < self.total:
                rate = completed / elapsed if elapsed > 0 else 0
                eta_seconds = (self.total - completed) / rate if rate > 0 else 0
                eta_str = f"{int(eta_seconds // 60)}m {int(eta_seconds % 60)}s"
            elif completed >= self.total:
                eta_str = "Done"
            else:
                eta_str = "Calculating..."

            return {
                'total': self.total,
                'downloaded': self.downloaded,
                'skipped': self.skipped,
                'failed': self.failed,
                'completed': completed,
                'progress_pct': round(progress_pct, 1),
                'elapsed_time': round(elapsed, 1),
                'eta': eta_str,
                'status': 'idle' if self.total == 0 else ('complete' if completed >= self.total else 'downloading'),
                'failed_tiles': self.failed_tiles[:20],
            }


def _download_single_tile(args):
    """Download a single tile (for use with ThreadPoolExecutor)"""
    url, local_path, zoom, x, y, output_dir = args
    base_path = Path(output_dir)
    full_path = base_path / local_path

    # Skip if already exists
    if full_path.exists():
        return ('skipped', zoom, x, y, None)

    # Create parent directory
    full_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        req = urllib.request.Request(url, headers={
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36'
        })
        with urllib.request.urlopen(req, timeout=20) as response:
            data = response.read()
        with open(full_path, 'wb') as f:
            f.write(data)
        return ('downloaded', zoom, x, y, None)
    except Exception as e:
        return ('failed', zoom, x, y, str(e))


def download_tiles(tile_urls, output_dir="media/tiles", num_threads=8, tracker=None):
    """
    Download tiles from the generated URL list using multi-threading.

    Args:
        tile_urls: List from generate_tile_urls()
        output_dir: Base directory to save tiles
        num_threads: Number of concurrent download threads (default: 8)
        tracker: Optional external ProgressTracker; one is created if not supplied
    """
    base_path = Path(output_dir)
    total = len(tile_urls)

    print(f"\n🗺️  Downloading {total} tiles to {base_path.absolute()}")
    print(f"⚡ Using {num_threads} threads for parallel downloads\n")

    if tracker is None:
        tracker = ProgressTracker(total)

    # Prepare download tasks
    tasks = [(url, local_path, zoom, x, y, output_dir)
             for url, local_path, zoom, x, y in tile_urls]

    # Execute downloads in parallel
    with ThreadPoolExecutor(max_workers=num_threads) as executor:
        futures = {executor.submit(_download_single_tile, task): task for task in tasks}

        for future in as_completed(futures):
            status, zoom, x, y, error = future.result()

            if status == 'downloaded':
                tracker.increment('downloaded')
                print(f"✅ Downloaded z={zoom} x={x} y={y}")
            elif status == 'skipped':
                tracker.increment('skipped')
                print(f"⏭️  Skipped z={zoom} x={x} y={y} (exists)")
            elif status == 'failed':
                tracker.increment('failed')
                tracker.add_failed(zoom, x, y, error)
                print(f"❌ Failed z={zoom} x={x} y={y}: {error}")

    # Summary
    snap = tracker.as_dict()
    print(f"\n✅ Download complete:")
    print(f"   Downloaded: {snap['downloaded']}")
    print(f"   Skipped: {snap['skipped']}")
    print(f"   Failed: {snap['failed']}")
    print(f"   Total: {snap['completed']}/{total}")

    if snap['failed'] > 0:
        print(f"\n❌ {snap['failed']} tiles failed")
        for tile in snap['failed_tiles'][:10]:
            print(f"   - z={tile['zoom']} x={tile['x']} y={tile['y']}: {tile['error']}")


def create_progress_html(progress_file_path):
    """Create HTML page for real-time download progress"""
    html_content = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Map Tile Downloader - Progress</title>
    <style>
        * {{ margin: 0; padding: 0; box-sizing: border-box; }}
        body {{
            font-family: 'Segoe UI', Arial, sans-serif;
            background: linear-gradient(135deg, #1a1a1a 0%, #2c3e50 100%);
            color: white;
            height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
            padding: 20px;
        }}
        .container {{
            background: rgba(44, 62, 80, 0.9);
            border-radius: 12px;
            padding: 30px;
            max-width: 800px;
            width: 100%;
            box-shadow: 0 10px 40px rgba(0, 0, 0, 0.5);
        }}
        h1 {{
            text-align: center;
            margin-bottom: 30px;
            color: #00d4ff;
        }}
        .progress-bar-container {{
            background: #1a1a1a;
            border-radius: 8px;
            height: 40px;
            overflow: hidden;
            margin-bottom: 20px;
            position: relative;
        }}
        .progress-bar {{
            height: 100%;
            background: linear-gradient(90deg, #00d4ff 0%, #0080ff 100%);
            transition: width 0.3s ease;
            display: flex;
            align-items: center;
            justify-content: center;
            font-weight: bold;
            font-size: 14px;
        }}
        .stats {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 15px;
            margin-bottom: 20px;
        }}
        .stat-box {{
            background: #1a1a1a;
            padding: 15px;
            border-radius: 8px;
            text-align: center;
        }}
        .stat-value {{
            font-size: 28px;
            font-weight: bold;
            color: #00d4ff;
            display: block;
            margin-bottom: 5px;
        }}
        .stat-label {{
            font-size: 12px;
            color: #95a5a6;
            text-transform: uppercase;
        }}
        .failed-tiles {{
            background: #1a1a1a;
            padding: 15px;
            border-radius: 8px;
            max-height: 200px;
            overflow-y: auto;
            font-family: 'Consolas', monospace;
            font-size: 12px;
        }}
        .failed-tiles h3 {{
            color: #e74c3c;
            margin-bottom: 10px;
            font-size: 14px;
        }}
        .failed-tile {{
            padding: 5px 0;
            border-bottom: 1px solid #34495e;
        }}
        .status-badge {{
            display: inline-block;
            padding: 5px 15px;
            border-radius: 20px;
            font-size: 12px;
            font-weight: bold;
            margin-bottom: 20px;
        }}
        .status-downloading {{ background: #f39c12; color: #1a1a1a; }}
        .status-complete {{ background: #27ae60; color: white; }}
        .hidden {{ display: none; }}
    </style>
</head>
<body>
    <div class="container">
        <h1>🗺️ Tile Download Progress</h1>

        <div id="statusBadge" class="status-badge status-downloading">⚡ DOWNLOADING</div>

        <div class="progress-bar-container">
            <div id="progressBar" class="progress-bar" style="width: 0%">0%</div>
        </div>

        <div class="stats">
            <div class="stat-box">
                <span id="statTotal" class="stat-value">0</span>
                <span class="stat-label">Total Tiles</span>
            </div>
            <div class="stat-box">
                <span id="statDownloaded" class="stat-value">0</span>
                <span class="stat-label">Downloaded</span>
            </div>
            <div class="stat-box">
                <span id="statSkipped" class="stat-value">0</span>
                <span class="stat-label">Skipped</span>
            </div>
            <div class="stat-box">
                <span id="statFailed" class="stat-value">0</span>
                <span class="stat-label">Failed</span>
            </div>
            <div class="stat-box">
                <span id="statETA" class="stat-value">--</span>
                <span class="stat-label">ETA</span>
            </div>
            <div class="stat-box">
                <span id="statElapsed" class="stat-value">0s</span>
                <span class="stat-label">Elapsed</span>
            </div>
        </div>

        <div id="failedTilesContainer" class="failed-tiles hidden">
            <h3>❌ Failed Tiles</h3>
            <div id="failedTilesList"></div>
        </div>
    </div>

    <script>
        function updateProgress() {{
            fetch('/progress')
                .then(response => response.json())
                .then(data => {{
                    // Update progress bar
                    const progressBar = document.getElementById('progressBar');
                    progressBar.style.width = data.progress_pct + '%';
                    progressBar.textContent = data.progress_pct + '%';

                    // Update stats
                    document.getElementById('statTotal').textContent = data.total;
                    document.getElementById('statDownloaded').textContent = data.downloaded;
                    document.getElementById('statSkipped').textContent = data.skipped;
                    document.getElementById('statFailed').textContent = data.failed;
                    document.getElementById('statETA').textContent = data.eta;
                    document.getElementById('statElapsed').textContent = data.elapsed_time + 's';

                    // Update status badge
                    const statusBadge = document.getElementById('statusBadge');
                    if (data.status === 'complete') {{
                        statusBadge.textContent = '✅ COMPLETE';
                        statusBadge.className = 'status-badge status-complete';
                    }}

                    // Show failed tiles if any
                    if (data.failed > 0) {{
                        const container = document.getElementById('failedTilesContainer');
                        const list = document.getElementById('failedTilesList');
                        container.classList.remove('hidden');

                        list.innerHTML = data.failed_tiles.map(tile =>
                            `<div class="failed-tile">z=${{tile.zoom}} x=${{tile.x}} y=${{tile.y}}: ${{tile.error}}</div>`
                        ).join('');
                    }}
                }})
                .catch(err => console.error('Failed to fetch progress:', err));
        }}

        // Update every 500ms
        setInterval(updateProgress, 500);
        updateProgress();  // Initial update
    </script>
</body>
</html>"""
    return html_content


class ProgressHTTPHandler(SimpleHTTPRequestHandler):
    """Custom HTTP handler for serving progress data"""
    progress_file = None

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            html = create_progress_html(self.progress_file)
            self.wfile.write(html.encode())
        elif self.path == '/progress':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            try:
                with open(self.progress_file, 'r') as f:
                    self.wfile.write(f.read().encode())
            except Exception:
                self.wfile.write(b'{{"error": "Progress file not found"}}')
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        # Suppress HTTP server logs
        pass


def launch_progress_page(progress_file_path, port=8765):
    """Launch HTTP server to display download progress"""
    # Set the progress file path for the handler
    ProgressHTTPHandler.progress_file = progress_file_path

    # Start server in a separate thread
    server = HTTPServer(('localhost', port), ProgressHTTPHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    # Open browser
    url = f"http://localhost:{port}"
    print(f"📊 Progress page launched: {url}")
    webbrowser.open(url)

    return server


class FileDownloadHandler(SimpleHTTPRequestHandler):
    """Custom HTTP handler for serving file downloads"""
    file_path = None
    file_name = None

    def do_GET(self):
        if self.path == '/':
            # Serve a simple download page
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()

            # Get file size
            file_size = Path(self.file_path).stat().st_size
            file_size_mb = file_size / (1024 * 1024)

            with open(self.file_path, 'r', encoding='utf-8') as f:
                line_count = sum(1 for _ in f)

            html = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Download Tile List</title>
    <style>
        * {{ margin: 0; padding: 0; box-sizing: border-box; }}
        body {{
            font-family: 'Segoe UI', Arial, sans-serif;
            background: linear-gradient(135deg, #1a1a1a 0%, #2c3e50 100%);
            color: white;
            height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
            padding: 20px;
        }}
        .container {{
            background: rgba(44, 62, 80, 0.9);
            border-radius: 12px;
            padding: 40px;
            max-width: 600px;
            width: 100%;
            box-shadow: 0 10px 40px rgba(0, 0, 0, 0.5);
            text-align: center;
        }}
        h1 {{
            color: #00d4ff;
            margin-bottom: 20px;
            font-size: 28px;
        }}
        .file-info {{
            background: #1a1a1a;
            padding: 20px;
            border-radius: 8px;
            margin: 20px 0;
            text-align: left;
        }}
        .info-row {{
            display: flex;
            justify-content: space-between;
            padding: 8px 0;
            border-bottom: 1px solid #34495e;
        }}
        .info-row:last-child {{
            border-bottom: none;
        }}
        .info-label {{
            color: #95a5a6;
            font-size: 14px;
        }}
        .info-value {{
            color: #00d4ff;
            font-weight: bold;
            font-size: 14px;
        }}
        .download-btn {{
            background: linear-gradient(90deg, #00d4ff 0%, #0080ff 100%);
            color: white;
            border: none;
            padding: 15px 40px;
            font-size: 18px;
            font-weight: bold;
            border-radius: 8px;
            cursor: pointer;
            text-decoration: none;
            display: inline-block;
            margin: 10px 0;
            transition: transform 0.2s;
        }}
        .download-btn:hover {{
            transform: scale(1.05);
        }}
        .instructions {{
            background: rgba(26, 26, 26, 0.5);
            padding: 20px;
            border-radius: 8px;
            margin-top: 20px;
            font-size: 14px;
            text-align: left;
            line-height: 1.6;
        }}
        .instructions h3 {{
            color: #00d4ff;
            margin-bottom: 10px;
            font-size: 16px;
        }}
        .instructions ol {{
            margin-left: 20px;
        }}
        .instructions li {{
            margin: 5px 0;
        }}
    </style>
</head>
<body>
    <div class="container">
        <h1>📥 Download Tile List File</h1>

        <div class="file-info">
            <div class="info-row">
                <span class="info-label">Filename:</span>
                <span class="info-value">{self.file_name}</span>
            </div>
            <div class="info-row">
                <span class="info-label">File Size:</span>
                <span class="info-value">{file_size_mb:.2f} MB ({file_size:,} bytes)</span>
            </div>
            <div class="info-row">
                <span class="info-label">Total URLs:</span>
                <span class="info-value">{line_count:,}</span>
            </div>
        </div>

        <a href="/download" class="download-btn">⬇️ Download File</a>

        <div class="instructions">
            <h3>📋 How to Import:</h3>
            <ol>
                <li><strong>Internet Download Manager (IDM):</strong><br>
                    File → Import → Import from text file → Select downloaded file</li>
                <li><strong>Free Download Manager (FDM):</strong><br>
                    Downloads → Import → Import from text file → Select downloaded file</li>
                <li><strong>Other Download Managers:</strong><br>
                    Look for "Import URLs" or "Batch Download" options</li>
            </ol>
            <p style="margin-top: 15px; color: #e74c3c;">
                <strong>Note:</strong> Remember to set the download directory to <code>media/tiles</code> in your download manager.
            </p>
        </div>
    </div>
</body>
</html>"""
            self.wfile.write(html.encode())

        elif self.path == '/download':
            # Serve the file for download
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.send_header('Content-Disposition', f'attachment; filename="{self.file_name}"')
            self.send_header('Content-Length', str(Path(self.file_path).stat().st_size))
            self.end_headers()

            with open(self.file_path, 'rb') as f:
                self.wfile.write(f.read())
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        # Suppress HTTP server logs
        pass


def launch_file_server(file_path, file_name, port=8766):
    """Launch HTTP server to serve file for download"""
    # Set the file info for the handler
    FileDownloadHandler.file_path = str(file_path)
    FileDownloadHandler.file_name = file_name

    # Start server
    server = HTTPServer(('localhost', port), FileDownloadHandler)

    # Open browser
    url = f"http://localhost:{port}"
    print(f"📥 File download page launched: {url}")
    webbrowser.open(url)

    return server


# ---------------------------------------------------------------------------
# Unified web-server: selector UI + download trigger + progress API
# ---------------------------------------------------------------------------

# Global tracker shared between the download thread and the HTTP handler
_active_tracker = None  # type: ProgressTracker | None
_active_tracker_lock = threading.Lock()


def _set_active_tracker(tracker):
    global _active_tracker
    with _active_tracker_lock:
        _active_tracker = tracker


def _get_active_tracker():
    with _active_tracker_lock:
        return _active_tracker


class TileServerHandler(SimpleHTTPRequestHandler):
    """
    Serves:
      GET  /            -> interactive selector + progress UI (HTML)
      GET  /progress    -> JSON progress snapshot
      POST /download    -> trigger server-side tile download
    """

    output_dir = "media/tiles"
    num_threads = 8

    # ------------------------------------------------------------------ GET
    def do_GET(self):
        if self.path == '/':
            self._serve_html()
        elif self.path == '/progress':
            self._serve_progress()
        else:
            self.send_error(404)

    # ------------------------------------------------------------------ POST
    def do_POST(self):
        if self.path == '/download':
            self._handle_download()
        else:
            self.send_error(404)

    # ------------------------------------------------------------------ helpers
    def _serve_html(self):
        html = create_tile_selector_html()
        body = html.encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_progress(self):
        tracker = _get_active_tracker()
        if tracker is None:
            data = {'status': 'idle', 'total': 0, 'downloaded': 0, 'skipped': 0,
                    'failed': 0, 'completed': 0, 'progress_pct': 0,
                    'elapsed_time': 0, 'eta': '--', 'failed_tiles': []}
        else:
            data = tracker.as_dict()
        body = json.dumps(data).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _handle_download(self):
        # KMZ_BBOX = {
        #     "lat1": 38.37560099,   # SW lat
        #     "lon1": -79.61154982,  # SW lon
        #     "lat2": 38.38188891,   # NE lat
        #     "lon2": -79.60701948,  # NE lon
        # }

        length = int(self.headers.get('Content-Length', 0))
        raw = self.rfile.read(length)
        try:
            params = json.loads(raw)
            lat1 = float(params['lat1'])
            lon1 = float(params['lon1'])
            lat2 = float(params['lat2'])
            lon2 = float(params['lon2'])
            # lat1, lon1 = KMZ_BBOX["lat1"], KMZ_BBOX["lon1"]
            # lat2, lon2 = KMZ_BBOX["lat2"], KMZ_BBOX["lon2"]
            zoom_min = int(params.get('zoom_min', 12))
            zoom_max = int(params.get('zoom_max', 16))
        except (KeyError, ValueError, json.JSONDecodeError) as exc:
            self._json_response(400, {'error': f'Bad request: {exc}'})
            return

        # Reject if a download is already running
        existing = _get_active_tracker()
        if existing is not None:
            snap = existing.as_dict()
            if snap['status'] == 'downloading':
                self._json_response(409, {'error': 'A download is already in progress'})
                return

        zoom_levels = list(range(zoom_min, zoom_max + 1))
        tiles_by_zoom = get_tiles_for_bbox(lat1, lon1, lat2, lon2, zoom_levels)
        tile_urls = generate_tile_urls(tiles_by_zoom)
        total = len(tile_urls)

        tracker = ProgressTracker(total)
        _set_active_tracker(tracker)

        output_dir = self.output_dir
        num_threads = self.num_threads

        def _run():
            download_tiles(tile_urls, output_dir=output_dir,
                           num_threads=num_threads, tracker=tracker)

        t = threading.Thread(target=_run, daemon=True)
        t.start()

        self._json_response(202, {'status': 'started', 'total': total})

    def _json_response(self, code, data):
        body = json.dumps(data).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format, *args):
        pass  # suppress access logs


def launch_server(port=8765, output_dir="media/tiles", num_threads=8):
    """
    Start the unified tile-downloader web server and open the browser.
    Blocks until Ctrl-C.
    """
    TileServerHandler.output_dir = output_dir
    TileServerHandler.num_threads = num_threads

    server = HTTPServer(('localhost', port), TileServerHandler)
    url = f"http://localhost:{port}"
    print(f"🌐 Tile downloader server running at {url}")
    print("   Draw an area, choose zoom levels, click Download.")
    print("   Press Ctrl+C to stop.\n")
    webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n🛑 Server stopped")
        server.shutdown()


# ---------------------------------------------------------------------------
# Selector / progress HTML
# ---------------------------------------------------------------------------

def create_tile_selector_html():
    """Create an interactive HTML map for area selection with integrated progress UI"""
    html_content = """<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Map Tile Downloader - Area Selection</title>
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <link rel="stylesheet" href="https://unpkg.com/leaflet-draw@1.0.4/dist/leaflet.draw.css" />
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <script src="https://unpkg.com/leaflet-draw@1.0.4/dist/leaflet.draw.js"></script>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        html, body { height: 100%; font-family: 'Segoe UI', Arial, sans-serif; background: #1a232e; color: #e0e0e0; display: flex; flex-direction: column; }
        #controls { padding: 12px 15px; background: #2c3e50; flex-shrink: 0; }
        #controls h2 { font-size: 15px; color: #00d4ff; margin-bottom: 10px; }
        #map { flex: 1 1 0; min-height: 0; }
        #info { padding: 10px 15px; background: #253040; flex-shrink: 0; min-height: 60px; }
        #progress-panel { background: #1a232e; border-top: 1px solid #34495e; padding: 12px 15px; flex-shrink: 0; display: none; }

        .button {
            background: #2980b9; color: white; border: none;
            padding: 8px 16px; margin: 3px; cursor: pointer;
            border-radius: 5px; font-size: 13px; font-weight: 600;
            transition: background .15s;
        }
        .button:hover { background: #3498db; }
        .button:disabled { background: #4a5a6a; cursor: not-allowed; color: #888; }
        .button.green { background: #27ae60; }
        .button.green:hover { background: #2ecc71; }
        .button.green:disabled { background: #4a5a6a; color: #888; }

        label { margin: 0 6px 0 12px; font-size: 13px; }
        input[type="number"] { width: 55px; padding: 5px 6px; background: #1a232e; border: 1px solid #4a6070; color: #e0e0e0; border-radius: 4px; font-size: 13px; }

        #output { font-family: 'Consolas', monospace; font-size: 12px; color: #7ecf7e; white-space: pre-wrap; }
        .warning { color: #e74c3c; font-weight: bold; }

        /* Progress bar */
        #pb-wrap { background: #0d1a26; border-radius: 6px; height: 28px; overflow: hidden; margin-bottom: 10px; position: relative; }
        #pb-bar { height: 100%; width: 0%; background: linear-gradient(90deg,#00aacc,#0066ff); transition: width .4s ease; display: flex; align-items: center; justify-content: center; font-size: 13px; font-weight: bold; color: #fff; min-width: 36px; }
        #pb-label { position: absolute; left: 50%; top: 50%; transform: translate(-50%,-50%); font-size: 13px; font-weight: bold; pointer-events: none; }

        /* Stats strip */
        .stats { display: flex; gap: 10px; flex-wrap: wrap; margin-top: 6px; }
        .stat { background: #0d1a26; padding: 6px 12px; border-radius: 5px; text-align: center; flex: 1 1 80px; }
        .stat-val { font-size: 20px; font-weight: bold; color: #00d4ff; display: block; }
        .stat-label { font-size: 10px; color: #7a9ab0; text-transform: uppercase; }
        #pb-status { font-size: 12px; margin-top: 6px; color: #7a9ab0; }
        .failed-list { margin-top: 6px; font-family: monospace; font-size: 11px; color: #e74c3c; max-height: 80px; overflow-y: auto; }
    </style>
</head>
<body>
    <div id="controls">
        <h2>🗺️ Map Tile Downloader</h2>
        <div style="display:flex; align-items:center; flex-wrap:wrap; gap:4px;">
            <button class="button" onclick="clearSelection()">Clear</button>
            <label>Zoom:</label>
            <input type="number" id="zoomMin" value="12" min="0" max="19">
            <span style="margin:0 4px;">to</span>
            <input type="number" id="zoomMax" value="16" min="0" max="19">
            <button class="button green" id="serverDlBtn" onclick="startServerDownload()" disabled>⬇ Download (server)</button>
            <button class="button" id="exportBtn" onclick="exportTileList()" disabled>Export URL list</button>
            <button class="button" id="copyBtn" onclick="copyToClipboard()" disabled>Copy Python code</button>
        </div>
        <div style="margin-top:7px; font-size:11px; opacity:.7;">
            Draw a rectangle → set zoom range → click <strong>Download (server)</strong> to download tiles to <code>media/tiles/</code> via the server.
        </div>
    </div>

    <div id="map"></div>

    <div id="info">
        <div id="output">▶ Draw a rectangle on the map to select an area...</div>
    </div>

    <!-- Live progress panel (shown once download starts) -->
    <div id="progress-panel">
        <div id="pb-wrap">
            <div id="pb-bar">0%</div>
        </div>
        <div class="stats">
            <div class="stat"><span id="st-total" class="stat-val">0</span><span class="stat-label">Total</span></div>
            <div class="stat"><span id="st-done" class="stat-val">0</span><span class="stat-label">Downloaded</span></div>
            <div class="stat"><span id="st-skip" class="stat-val">0</span><span class="stat-label">Skipped</span></div>
            <div class="stat"><span id="st-fail" class="stat-val">0</span><span class="stat-label">Failed</span></div>
            <div class="stat"><span id="st-eta" class="stat-val">--</span><span class="stat-label">ETA</span></div>
            <div class="stat"><span id="st-elapsed" class="stat-val">0s</span><span class="stat-label">Elapsed</span></div>
        </div>
        <div id="pb-status">Initialising…</div>
        <div id="pb-failed" class="failed-list"></div>
    </div>

    <script>
        /* ── Map setup ─────────────────────────────────────────── */
        var map = L.map('map').setView([38.3760167, -79.6078722], 13);
        L.tileLayer('https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}', {
            maxZoom: 19, attribution: 'Map data © Google'
        }).addTo(map);

        var drawnItems = new L.FeatureGroup();
        map.addLayer(drawnItems);
        var drawControl = new L.Control.Draw({
            position: 'topleft',
            draw: {
                polygon: false, polyline: false, circle: false,
                circlemarker: false, marker: false,
                rectangle: { shapeOptions: { color: '#00d4ff', weight: 3 } }
            },
            edit: { featureGroup: drawnItems, remove: true }
        });
        map.addControl(drawControl);

        var selectedBounds = null;

        map.on('draw:created', function(e) {
            drawnItems.clearLayers();
            drawnItems.addLayer(e.layer);
            selectedBounds = e.layer.getBounds();
            updateInfo();
        });
        map.on('draw:deleted', function() { selectedBounds = null; updateInfo(); });

        /* ── Tile maths ────────────────────────────────────────── */
        function lon2tile(lon, z) { return Math.floor((lon + 180) / 360 * Math.pow(2, z)); }
        function lat2tile(lat, z) {
            return Math.floor((1 - Math.log(Math.tan(lat * Math.PI / 180) + 1 / Math.cos(lat * Math.PI / 180)) / Math.PI) / 2 * Math.pow(2, z));
        }
        function tileCount(lat1, lon1, lat2, lon2, z) {
            var x1 = lon2tile(Math.min(lon1,lon2),z), x2 = lon2tile(Math.max(lon1,lon2),z);
            var y1 = lat2tile(Math.max(lat1,lat2),z), y2 = lat2tile(Math.min(lat1,lat2),z);
            return (Math.abs(x2-x1)+1) * (Math.abs(y2-y1)+1);
        }

        /* ── Info panel ────────────────────────────────────────── */
        function updateInfo() {
            var out = document.getElementById('output');
            var enabled = !!selectedBounds;
            ['serverDlBtn','exportBtn','copyBtn'].forEach(function(id){
                document.getElementById(id).disabled = !enabled;
            });

            if (!enabled) { out.textContent = '▶ Draw a rectangle on the map to select an area...'; return; }

            var sw = selectedBounds.getSouthWest(), ne = selectedBounds.getNorthEast();
            var zMin = parseInt(document.getElementById('zoomMin').value);
            var zMax = parseInt(document.getElementById('zoomMax').value);
            var total = 0, breakdown = [];
            for (var z = zMin; z <= zMax; z++) {
                var c = tileCount(sw.lat, sw.lng, ne.lat, ne.lng, z);
                total += c; breakdown.push('z=' + z + ': ' + c);
            }
            var warn = total > 1000 ? '\\n⚠️  ' + total + ' tiles is large — consider reducing area or zoom.' : '';
            out.textContent = '▶ SW: ' + sw.lat.toFixed(6) + ', ' + sw.lng.toFixed(6) +
                '\\n▶ NE: ' + ne.lat.toFixed(6) + ', ' + ne.lng.toFixed(6) +
                '\\n▶ Zoom ' + zMin + '–' + zMax + ' → ' + total + ' tiles  (' + breakdown.join(', ') + ')' + warn;
        }
        document.getElementById('zoomMin').addEventListener('change', updateInfo);
        document.getElementById('zoomMax').addEventListener('change', updateInfo);

        /* ── Server-side download ──────────────────────────────── */
        var pollTimer = null;

        function startServerDownload() {
            if (!selectedBounds) return;
            var sw = selectedBounds.getSouthWest(), ne = selectedBounds.getNorthEast();
            var payload = {
                lat1: sw.lat, lon1: sw.lng,
                lat2: ne.lat, lon2: ne.lng,
                zoom_min: parseInt(document.getElementById('zoomMin').value),
                zoom_max: parseInt(document.getElementById('zoomMax').value)
            };

            document.getElementById('serverDlBtn').disabled = true;
            document.getElementById('pb-status').textContent = 'Sending request to server…';
            document.getElementById('progress-panel').style.display = 'block';

            fetch('/download', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(payload)
            })
            .then(function(r) { return r.json(); })
            .then(function(data) {
                if (data.error) {
                    document.getElementById('pb-status').textContent = '❌ ' + data.error;
                    document.getElementById('serverDlBtn').disabled = false;
                    return;
                }
                document.getElementById('pb-status').textContent = 'Download started — ' + data.total + ' tiles queued.';
                startPolling();
            })
            .catch(function(err) {
                document.getElementById('pb-status').textContent = '❌ Request failed: ' + err;
                document.getElementById('serverDlBtn').disabled = false;
            });
        }

        function startPolling() {
            if (pollTimer) clearInterval(pollTimer);
            pollTimer = setInterval(pollProgress, 600);
            pollProgress();
        }

        function pollProgress() {
            fetch('/progress')
            .then(function(r) { return r.json(); })
            .then(function(d) {
                var bar = document.getElementById('pb-bar');
                bar.style.width = d.progress_pct + '%';
                bar.textContent = d.progress_pct + '%';

                document.getElementById('st-total').textContent = d.total;
                document.getElementById('st-done').textContent = d.downloaded;
                document.getElementById('st-skip').textContent = d.skipped;
                document.getElementById('st-fail').textContent = d.failed;
                document.getElementById('st-eta').textContent = d.eta;
                document.getElementById('st-elapsed').textContent = d.elapsed_time + 's';

                if (d.status === 'complete') {
                    document.getElementById('pb-status').textContent = '✅ Complete — ' + d.downloaded + ' downloaded, ' + d.skipped + ' skipped, ' + d.failed + ' failed.';
                    clearInterval(pollTimer); pollTimer = null;
                    document.getElementById('serverDlBtn').disabled = false;
                } else if (d.status === 'downloading') {
                    document.getElementById('pb-status').textContent = 'Downloading… ' + d.completed + ' / ' + d.total;
                }

                if (d.failed_tiles && d.failed_tiles.length) {
                    document.getElementById('pb-failed').innerHTML =
                        d.failed_tiles.map(function(t){ return 'z='+t.zoom+' x='+t.x+' y='+t.y+': '+t.error; }).join('<br>');
                }
            })
            .catch(function(err) { console.warn('Progress poll failed:', err); });
        }

        /* ── Export URL list (client-side) ─────────────────────── */
        function deg2num(lat, lon, z) {
            var lr = lat * Math.PI / 180, n = Math.pow(2, z);
            return { x: Math.floor((lon+180)/360*n), y: Math.floor((1-Math.log(Math.tan(lr)+1/Math.cos(lr))/Math.PI)/2*n) };
        }

        function exportTileList() {
            if (!selectedBounds) return;
            var sw = selectedBounds.getSouthWest(), ne = selectedBounds.getNorthEast();
            var zMin = parseInt(document.getElementById('zoomMin').value);
            var zMax = parseInt(document.getElementById('zoomMax').value);
            var lines = [];
            for (var z = zMin; z <= zMax; z++) {
                var nw = deg2num(ne.lat, sw.lng, z), se = deg2num(sw.lat, ne.lng, z);
                for (var x = Math.min(nw.x,se.x); x <= Math.max(nw.x,se.x); x++)
                    for (var y = Math.min(nw.y,se.y); y <= Math.max(nw.y,se.y); y++)
                        lines.push('https://mt1.google.com/vt/lyrs=y&x='+x+'&y='+y+'&z='+z);
            }
            var blob = new Blob([lines.join('\\n')], {type:'text/plain'});
            var a = document.createElement('a');
            a.href = URL.createObjectURL(blob);
            a.download = 'tile_download_list.txt';
            a.click();
        }

        /* ── Copy Python snippet ───────────────────────────────── */
        function copyToClipboard() {
            if (!selectedBounds) return;
            var sw = selectedBounds.getSouthWest(), ne = selectedBounds.getNorthEast();
            var zMin = parseInt(document.getElementById('zoomMin').value);
            var zMax = parseInt(document.getElementById('zoomMax').value);
            var code = 'from tiles import get_tiles_for_bbox, generate_tile_urls, download_tiles\\n\\n' +
                'tiles_by_zoom = get_tiles_for_bbox(\\n' +
                '    lat1=' + sw.lat.toFixed(6) + ', lon1=' + sw.lng.toFixed(6) + ',\\n' +
                '    lat2=' + ne.lat.toFixed(6) + ', lon2=' + ne.lng.toFixed(6) + ',\\n' +
                '    zoom_levels=list(range(' + zMin + ', ' + (zMax+1) + '))\\n)\\n\\n' +
                'download_tiles(generate_tile_urls(tiles_by_zoom), num_threads=8)\\n';
            navigator.clipboard.writeText(code)
                .then(function(){ alert('Python code copied!'); })
                .catch(function(){ alert('Copy failed — see console.'); console.log(code); });
        }

        function clearSelection() { drawnItems.clearLayers(); selectedBounds = null; updateInfo(); }
    </script>
</body>
</html>"""

    return html_content


def launch_selector():
    """Launch the tile selector tool in the browser"""
    html_content = create_tile_selector_html()

    # Create temporary HTML file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.html', delete=False, encoding='utf-8') as f:
        f.write(html_content)
        temp_path = f.name

    print("🗺️  Launching Map Tile Selector...")
    print(f"📂 Temporary HTML file: {temp_path}")
    print("\n📋 Instructions:")
    print("   1. Draw a rectangle on the map to select your area")
    print("   2. Adjust zoom levels if needed (12-16 recommended)")
    print("   3. Click 'Copy Python Code' to get the download code")
    print("   4. Run the generated code to download tiles\n")

    webbrowser.open(f'file://{temp_path}')

    return temp_path


def main():
    """Main entry point for the tile downloader"""
    print("=" * 60)
    print("🗺️  Map Tile Downloader Tool")
    print("=" * 60)
    print("\nThis tool helps you download map tiles for offline use.")
    print("\nOptions:")
    print("  1. Launch web server (draw area, click Download — live progress)")
    print("  2. Download tiles for a specific area (manual CLI)")
    print("  3. Export tile list for IDM/FDM (manual CLI)")
    print("\nChoice (1/2/3): ", end="")

    choice = input().strip()

    if choice == "1":
        launch_server()  # blocks until Ctrl-C

    elif choice == "2":
        # Manual mode - example usage
        print("\n📍 Enter coordinates for the area:")
        lat1 = float(input("  Southwest Latitude: "))
        lon1 = float(input("  Southwest Longitude: "))
        lat2 = float(input("  Northeast Latitude: "))
        lon2 = float(input("  Northeast Longitude: "))

        zoom_min = int(input("\n🔍 Minimum zoom level (e.g., 12): "))
        zoom_max = int(input("  Maximum zoom level (e.g., 16): "))

        zoom_levels = list(range(zoom_min, zoom_max + 1))

        print("\n🔄 Calculating tiles...")
        tiles_by_zoom = get_tiles_for_bbox(lat1, lon1, lat2, lon2, zoom_levels)

        total = sum(len(tiles) for tiles in tiles_by_zoom.values())
        print(f"📊 Total tiles to download: {total}")

        for zoom, tiles in tiles_by_zoom.items():
            print(f"   Zoom {zoom}: {len(tiles)} tiles")

        if total > 1000:
            print("\n⚠️  Warning: This is a large download! Consider reducing the area.")
            confirm = input("Continue? (yes/no): ")
            if confirm.lower() != 'yes':
                print("Cancelled.")
                return

        print("\n🌐 Generating URLs...")
        tile_urls = generate_tile_urls(tiles_by_zoom)

        download_tiles(tile_urls)

    elif choice == "3":
        # Export mode - for download managers
        print("\n📍 Enter coordinates for the area:")
        lat1 = float(input("  Southwest Latitude: "))
        lon1 = float(input("  Southwest Longitude: "))
        lat2 = float(input("  Northeast Latitude: "))
        lon2 = float(input("  Northeast Longitude: "))

        zoom_min = int(input("\n🔍 Minimum zoom level (e.g., 12): "))
        zoom_max = int(input("  Maximum zoom level (e.g., 16): "))

        zoom_levels = list(range(zoom_min, zoom_max + 1))

        print("\n🔄 Calculating tiles...")
        tiles_by_zoom = get_tiles_for_bbox(lat1, lon1, lat2, lon2, zoom_levels)

        total = sum(len(tiles) for tiles in tiles_by_zoom.values())
        print(f"📊 Total tiles to export: {total}")

        print("\n📝 Export formats:")
        print("  1. Simple (just URLs) - most compatible")
        print("  2. With paths (URL + local path)")
        print("  3. IDM batch format")
        print("  4. FDM list format")
        print("  5. IDM command script (.bat) - preserves folders")
        format_choice = input("Format (1/2/3/4/5): ").strip()

        format_map = {
            "1": "simple",
            "2": "with_paths",
            "3": "idm_batch",
            "4": "fdm_list",
            "5": "idm_cmd"
        }
        format_type = format_map.get(format_choice, "simple")

        output_file = input("Output filename (default: tile_download_list.txt): ").strip()
        if not output_file:
            output_file = "tile_download_list.txt"

        print("\n🌐 Generating URLs...")
        tile_urls = generate_tile_urls(tiles_by_zoom)

        export_for_download_manager(tile_urls, output_file, format_type)

    else:
        print("Invalid choice.")


if __name__ == "__main__":
    main()





