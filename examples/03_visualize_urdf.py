#!/usr/bin/env python3
"""Online URDF Visualization for Synria Robot Descriptions.

Starts a local web server and opens a browser-based 3D viewer for URDF
robot models using Three.js and urdf-loader. Works on Windows and Linux.

Usage:
    # Default: visualize Alicia_D v5_6 leader_ur
    python 03_visualize_urdf.py

    # Specify robot via synriard API:
    python 03_visualize_urdf.py --name Alicia_D --version v5_6 --variant gripper_100mm

    # Specify URDF path directly:
    python 03_visualize_urdf.py --urdf synriard/urdf/Alicia_D_v5_6/Alicia_D_v5_6_leader_ur.urdf

Requirements:
    - Python 3.7+
    - A modern web browser
    (No internet connection required - all JS dependencies are bundled locally)
"""

import argparse
import http.server
import mimetypes
import os
import socket
import sys
import threading
import webbrowser
from functools import partial
from pathlib import Path

# Register MIME types for robot description files
mimetypes.add_type("application/octet-stream", ".stl")
mimetypes.add_type("application/octet-stream", ".STL")
mimetypes.add_type("application/xml", ".urdf")
mimetypes.add_type("application/javascript", ".js")

VIEWER_HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>URDF Viewer - __ROBOT_NAME__</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { overflow: hidden; font-family: 'Segoe UI', Arial, sans-serif; background: #1a1a2e; }
        #viewer { width: 100vw; height: 100vh; display: block; }
        #controls {
            position: fixed;
            top: 10px;
            right: 10px;
            background: rgba(20, 20, 40, 0.92);
            color: #e0e0e0;
            padding: 16px;
            border-radius: 8px;
            max-height: 90vh;
            overflow-y: auto;
            min-width: 280px;
            backdrop-filter: blur(10px);
            border: 1px solid rgba(79, 195, 247, 0.3);
        }
        #controls h2 {
            font-size: 15px;
            margin-bottom: 12px;
            color: #4fc3f7;
            border-bottom: 1px solid rgba(79, 195, 247, 0.3);
            padding-bottom: 8px;
        }
        .joint-control {
            margin-bottom: 8px;
            padding: 4px 0;
            border-bottom: 1px solid rgba(255,255,255,0.04);
        }
        .joint-control label {
            display: flex;
            justify-content: space-between;
            font-size: 13px;
            margin-bottom: 3px;
            color: #b0bec5;
        }
        .joint-control input[type="range"] {
            width: 100%;
            accent-color: #4fc3f7;
        }
        .joint-value {
            font-size: 11px;
            color: #78909c;
            font-family: 'Courier New', monospace;
        }
        .btn {
            background: rgba(79, 195, 247, 0.15);
            color: #4fc3f7;
            border: 1px solid rgba(79, 195, 247, 0.4);
            padding: 8px 16px;
            border-radius: 4px;
            cursor: pointer;
            font-size: 13px;
            width: 100%;
            margin-top: 12px;
            transition: background 0.2s;
        }
        .btn:hover { background: rgba(79, 195, 247, 0.3); }
        #loading {
            position: fixed;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            color: #4fc3f7;
            font-size: 18px;
            text-align: center;
            font-family: 'Segoe UI', Arial, sans-serif;
        }
        .spinner {
            border: 3px solid rgba(79, 195, 247, 0.15);
            border-top: 3px solid #4fc3f7;
            border-radius: 50%;
            width: 40px;
            height: 40px;
            animation: spin 1s linear infinite;
            margin: 0 auto 15px;
        }
        @keyframes spin { to { transform: rotate(360deg); } }
    </style>
    <script type="importmap">
    {
        "imports": {
            "three": "./examples/vendor/three.module.js",
            "three/addons/": "./examples/vendor/addons/",
            "three/examples/jsm/": "./examples/vendor/addons/"
        }
    }
    </script>
</head>
<body>
    <canvas id="viewer"></canvas>
    <div id="loading">
        <div class="spinner"></div>
        Loading URDF model...
    </div>
    <div id="controls" style="display:none;">
        <h2>__ROBOT_NAME__</h2>
        <div id="joint-controls"></div>
        <button class="btn" onclick="window.resetJoints()">Reset All Joints</button>
    </div>
    <script type="module">
        import * as THREE from 'three';
        import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
        import URDFLoader from './examples/vendor/URDFLoader.js';

        const URDF_PATH = '__URDF_PATH__';

        // ---- Scene ----
        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0x1a1a2e);

        // ---- Camera ----
        const camera = new THREE.PerspectiveCamera(
            50, window.innerWidth / window.innerHeight, 0.001, 100);
        camera.up.set(0, 0, 1);
        camera.position.set(0.5, -0.5, 0.4);

        // ---- Renderer ----
        const canvas = document.getElementById('viewer');
        const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
        renderer.setSize(window.innerWidth, window.innerHeight);
        renderer.setPixelRatio(window.devicePixelRatio);
        renderer.shadowMap.enabled = true;
        renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        renderer.outputColorSpace = THREE.SRGBColorSpace;
        renderer.toneMapping = THREE.ACESFilmicToneMapping;
        renderer.toneMappingExposure = 1.2;

        // ---- Controls ----
        const controls = new OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.1;
        controls.target.set(0, 0, 0.15);
        controls.update();

        // ---- Lighting ----
        scene.add(new THREE.AmbientLight(0xffffff, 0.6));
        scene.add(new THREE.HemisphereLight(0xb0d4f1, 0x404040, 0.8));

        const dirLight1 = new THREE.DirectionalLight(0xffffff, 1.5);
        dirLight1.position.set(3, 5, 3);
        dirLight1.castShadow = true;
        dirLight1.shadow.mapSize.set(2048, 2048);
        scene.add(dirLight1);

        const dirLight2 = new THREE.DirectionalLight(0xffffff, 0.5);
        dirLight2.position.set(-2, 3, -1);
        scene.add(dirLight2);

        const dirLight3 = new THREE.DirectionalLight(0xffffff, 0.3);
        dirLight3.position.set(0, -1, 3);
        scene.add(dirLight3);

        // ---- Ground & Helpers (Z-up) ----
        const grid = new THREE.GridHelper(2, 20, 0x444466, 0x333355);
        grid.rotation.x = Math.PI / 2;
        scene.add(grid);
        scene.add(new THREE.AxesHelper(0.15));

        const groundMat = new THREE.ShadowMaterial({ opacity: 0.3 });
        const ground = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), groundMat);
        ground.receiveShadow = true;
        scene.add(ground);

        // ---- Load URDF ----
        let robot = null;
        const jointSliders = {};

        const loader = new URDFLoader();
        loader.load(URDF_PATH, (result) => {
            robot = result;
            scene.add(robot);

            // Upgrade materials for better appearance
            robot.traverse((child) => {
                if (child.isMesh && child.material) {
                    const oldColor = child.material.color
                        ? child.material.color.clone()
                        : new THREE.Color(0.5, 0.5, 0.5);
                    child.material = new THREE.MeshPhysicalMaterial({
                        color: oldColor,
                        metalness: 0.3,
                        roughness: 0.5,
                        clearcoat: 0.1,
                    });
                    child.castShadow = true;
                    child.receiveShadow = true;
                }
            });

            // Auto-fit camera to robot
            const box = new THREE.Box3().setFromObject(robot);
            const center = box.getCenter(new THREE.Vector3());
            const size = box.getSize(new THREE.Vector3());
            const maxDim = Math.max(size.x, size.y, size.z);

            controls.target.copy(center);
            camera.position.set(
                center.x + maxDim * 1.5,
                center.y - maxDim * 1.5,
                center.z + maxDim * 1.0
            );
            controls.update();

            // Build joint control UI
            createJointControls(robot);

            document.getElementById('loading').style.display = 'none';
            document.getElementById('controls').style.display = 'block';
        },
        undefined,
        (err) => {
            document.getElementById('loading').innerHTML =
                '<div style="color:#ff5252;">Failed to load URDF.<br>Check the console for details.</div>';
            console.error('URDF load error:', err);
        });

        function createJointControls(robot) {
            const container = document.getElementById('joint-controls');
            container.innerHTML = '';

            for (const jointName in robot.joints) {
                const joint = robot.joints[jointName];
                if (joint.jointType === 'fixed') continue;

                const lower = joint.limit.lower;
                const upper = joint.limit.upper;
                const range = upper - lower;
                if (range === 0) continue;

                const div = document.createElement('div');
                div.className = 'joint-control';

                const label = document.createElement('label');
                label.innerHTML =
                    '<span>' + jointName + '</span>' +
                    '<span class="joint-value" id="val-' + jointName + '">0.000 rad</span>';

                const slider = document.createElement('input');
                slider.type = 'range';
                slider.min = lower;
                slider.max = upper;
                slider.step = range / 200;
                slider.value = 0;

                slider.addEventListener('input', () => {
                    const val = parseFloat(slider.value);
                    robot.setJointValue(jointName, val);
                    document.getElementById('val-' + jointName).textContent =
                        val.toFixed(3) + ' rad';
                });

                jointSliders[jointName] = slider;
                div.appendChild(label);
                div.appendChild(slider);
                container.appendChild(div);
            }
        }

        window.resetJoints = function () {
            if (!robot) return;
            for (const name in jointSliders) {
                jointSliders[name].value = 0;
                robot.setJointValue(name, 0);
                document.getElementById('val-' + name).textContent = '0.000 rad';
            }
        };

        // ---- Resize ----
        window.addEventListener('resize', () => {
            camera.aspect = window.innerWidth / window.innerHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(window.innerWidth, window.innerHeight);
        });

        // ---- Render Loop ----
        (function animate() {
            requestAnimationFrame(animate);
            controls.update();
            renderer.render(scene, camera);
        })();
    </script>
</body>
</html>"""


class ViewerHandler(http.server.SimpleHTTPRequestHandler):
    """Serves the viewer page at / and static files for URDF + meshes."""

    def __init__(self, *args, viewer_html="", **kwargs):
        self._viewer_html = viewer_html
        super().__init__(*args, **kwargs)

    def do_GET(self):
        if self.path in ("/", "/viewer"):
            content = self._viewer_html.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(content)
        else:
            super().do_GET()

    def log_message(self, format, *args):
        # Quiet logging: only print errors
        if len(args) >= 2 and "404" in str(args[1]):
            super().log_message(format, *args)


def main():
    parser = argparse.ArgumentParser(
        description="Visualize URDF robot models in a web browser."
    )
    parser.add_argument(
        "--name", type=str, default="Alicia_D", help="Robot name (default: Alicia_D)"
    )
    parser.add_argument(
        "--version", type=str, default="v5_6", help="Robot version (default: v5_6)"
    )
    parser.add_argument(
        "--variant",
        type=str,
        default="leader_ur",
        help="Robot variant (default: leader_ur)",
    )
    parser.add_argument(
        "--urdf",
        type=str,
        default=None,
        help="Direct path to a URDF file (overrides --name/--version/--variant)",
    )
    parser.add_argument(
        "--host",
        type=str,
        default="0.0.0.0",
        help="Host address to bind the server to (default: 0.0.0.0)",
    )
    parser.add_argument(
        "--port", type=int, default=7654, help="HTTP server port (default: 7654)"
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="Do not open browser automatically",
    )
    args = parser.parse_args()

    project_root = Path(__file__).resolve().parent.parent

    # ---- Resolve URDF path ----
    if args.urdf:
        urdf_abs = Path(args.urdf).resolve()
    else:
        try:
            sys.path.insert(0, str(project_root))
            from synriard import get_model_path

            urdf_abs = Path(get_model_path(args.name, version=args.version, variant=args.variant))
        except (ImportError, ValueError):
            robot_dir = f"{args.name}_{args.version}"
            urdf_file = f"{robot_dir}_{args.variant}.urdf" if args.variant else f"{robot_dir}.urdf"
            urdf_abs = project_root / "synriard" / "urdf" / robot_dir / urdf_file

    if not urdf_abs.exists():
        print(f"Error: URDF file not found: {urdf_abs}")
        sys.exit(1)

    try:
        urdf_rel = urdf_abs.relative_to(project_root).as_posix()
    except ValueError:
        print(f"Error: URDF file must be inside the project directory:\n  {project_root}")
        sys.exit(1)

    robot_name = urdf_abs.stem

    # ---- Generate viewer HTML ----
    viewer_html = VIEWER_HTML_TEMPLATE.replace("__URDF_PATH__", urdf_rel).replace(
        "__ROBOT_NAME__", robot_name
    )

    # ---- Start HTTP server ----
    os.chdir(project_root)
    handler = partial(ViewerHandler, viewer_html=viewer_html)

    try:
        server = http.server.ThreadingHTTPServer((args.host, args.port), handler)
    except OSError as e:
        print(f"Error: Could not start server on {args.host}:{args.port}: {e}")
        print("Try a different port with --port <number>")
        sys.exit(1)

    url = f"http://127.0.0.1:{args.port}/"
    print(f"URDF Viewer: {url}")
    if args.host == "0.0.0.0":
        hostname = socket.gethostname()
        try:
            local_ip = socket.gethostbyname(hostname)
        except socket.gaierror:
            local_ip = None
        if local_ip:
            print(f"  also at:   http://{local_ip}:{args.port}/")
    print(f"Model:       {urdf_rel}")
    print("Press Ctrl+C to stop.\n")

    if not args.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nServer stopped.")
        server.server_close()


if __name__ == "__main__":
    main()
