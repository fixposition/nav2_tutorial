import dash
import dash_bootstrap_components as dbc
from dash import html, dcc, Output, Input, State, ctx
import os
import subprocess
import glob
import threading
import queue
import yaml
import math

# ----- Constants -----
TRAJECTORY_DIR = '/home/dev/ros_ws/src/nav2_tutorial/trajectories'
RECORD_SCRIPT = '/home/dev/ros_ws/src/nav2_tutorial/src/loggers/gps_periodic_logger.py'
PRECISE_SCRIPT = '/home/dev/ros_ws/src/nav2_tutorial/src/precise_wp_follower.py'
SMOOTH_SCRIPT = '/home/dev/ros_ws/src/nav2_tutorial/src/smooth_wp_follower.py'
INTERACTIVE_SCRIPT = '/home/jetson/dev/nav2_tutorial/src/nav2_tutorial/src/interactive_wp_follower.py'

def get_trajectories():
    return sorted(glob.glob(os.path.join(TRAJECTORY_DIR, '*.yaml')))

def load_trajectory_from_yaml(yaml_path):
    try:
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)
        waypoints = data.get("waypoints", [])
        lats = [wp["latitude"] for wp in waypoints]
        lons = [wp["longitude"] for wp in waypoints]
        yaws = [wp["yaw"] if "yaw" in wp else 0 for wp in waypoints]
        return lats, lons, yaws
    except Exception:
        return [], [], []

# --- Process management (combined) ---
process = None
process_thread = None
process_log_queue = queue.Queue()
process_log_buffer = []
PROCESS_LOG_BUFFER_MAX_LINES = 100
process_running = threading.Event()
last_process_type = "logger"  # or "follower"

def run_process(cmd, process_type):
    global process, last_process_type
    process_running.set()
    last_process_type = process_type
    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1
    )
    try:
        for line in process.stdout:
            process_log_queue.put(line)
    except Exception as e:
        process_log_queue.put(f"[Process error]: {e}\n")
    process_running.clear()

def start_process(cmd, process_type):
    global process_thread, process_running
    if process_running.is_set():
        process_log_queue.put("[A process is already running]\n")
        return
    process_thread = threading.Thread(target=run_process, args=(cmd, process_type), daemon=True)
    process_thread.start()

def stop_process():
    global process, process_running
    if process and process_running.is_set():
        process.terminate()
        process_running.clear()
        process_log_queue.put("[Process stopped]\n")

def get_combined_live_log():
    # Compose context message at the top
    if process_running.is_set():
        if last_process_type == "follower":
            header = "Displaying output from trajectory follower...\n\n"
        else:
            header = "Displaying output from logger...\n\n"
    else:
        header = "No active process.\n\n"
    # Gather lines from queue
    while not process_log_queue.empty():
        line = process_log_queue.get()
        process_log_buffer.append(line)
        if len(process_log_buffer) > PROCESS_LOG_BUFFER_MAX_LINES:
            process_log_buffer.pop(0)
    return header + ''.join(process_log_buffer)

# ----- App Layout -----
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])

app.layout = dbc.Container([
    html.H2("Trajectory Dashboard"),
    dbc.Row([
        dbc.Col([
            dcc.Graph(id="trajectory-map", style={"height": "520px", "margin-bottom": "8px"}),
            dbc.Checkbox(id="show-orientation", value=False, className="mb-2", style={"margin-left": "2px"}),
            html.Label("Show orientation", htmlFor="show-orientation", style={"margin-left": "25px", "font-size": "90%"}),
        ], width=12),
    ], style={"margin-bottom": "10px"}),
    dbc.Row([
        dbc.Col([
            html.H4("Trajectory Controls"),
            dcc.Dropdown(
                id="trajectory-dropdown",
                options=[{"label": os.path.basename(f), "value": f} for f in get_trajectories()],
                placeholder="Select a trajectory",
                style={"margin-bottom": "10px"}
            ),
            dbc.Button("Use Last", id="use-last-btn", color="secondary", style={"width": "100%", "margin-bottom": "10px"}),
            dbc.Badge("Using last", id="using-last-indicator", color="info", pill=True, style={"display": "none", "margin-bottom": "10px"}),
            dbc.Button("Reverse", id="reverse-btn", color="warning", style={"width": "100%", "margin-bottom": "10px"}),
            dbc.Badge("Reverse", id="reverse-indicator", color="warning", pill=True, style={"display": "none", "margin-bottom": "10px"}),
        ], width=3),
        dbc.Col([
            html.H4("Action"),
            dbc.RadioItems(
                id="mode-radio",
                options=[
                    {"label": "Precise", "value": "precise"},
                    {"label": "Smooth", "value": "smooth"},
                    {"label": "Interactive", "value": "interactive"},
                ],
                value="precise",
                inline=True,
                inputStyle={"margin-right": "8px", "margin-left": "16px"}
            ),
            dbc.Button("Play Trajectory", id="play-btn", color="primary", style={"width": "100%", "margin-top": "10px"}),
            dbc.Button("Record Trajectory", id="record-btn", color="success", style={"width": "100%", "margin-top": "10px"}),
            dbc.Button("Stop Process", id="stop-process-btn", color="danger", style={"width": "100%", "margin-top": "10px"}),
            html.Div(id="status", style={"margin-top": "20px"}),
        ], width=3),
        dbc.Col([
            html.H4("Live Output"),
            html.Pre(id='live-output', style={'height': '340px', 'overflow': 'auto', 'background': '#141414', 'color': '#e2e2e2', "font-size": "14px", "padding": "12px"}),
            dcc.Interval(id='live-output-interval', interval=1000, n_intervals=0),
        ], width=6),
    ]),
    dcc.Store(id="using-last-store", data=False),
    dcc.Store(id="reverse-store", data=False),
])

# ----- CALLBACKS -----

# Trajectory map
@app.callback(
    Output("trajectory-map", "figure"),
    Input("trajectory-dropdown", "value"),
    Input("using-last-store", "data"),
    Input("show-orientation", "value"),
)
def update_trajectory_map(selected_file, using_last, show_orientation):
    if using_last or selected_file == "USING_LAST":
        files = get_trajectories()
        if not files:
            return {
                "data": [],
                "layout": {"title": "No trajectories found", "height": 500}
            }
        yaml_path = files[-1]
    elif selected_file:
        yaml_path = selected_file
    else:
        return {
            "data": [],
            "layout": {"title": "No trajectory selected", "height": 500}
        }
    lats, lons, yaws = load_trajectory_from_yaml(yaml_path)
    if not lats or not lons:
        return {
            "data": [],
            "layout": {"title": "No waypoints in file", "height": 500}
        }
    # Trajectory line
    data = [{
        "type": "scattermapbox",
        "lat": lats,
        "lon": lons,
        "mode": "lines+markers",
        "marker": {"size": 8, "color": "red"},
        "line": {"width": 2, "color": "blue"},
        "name": "Trajectory"
    }]
    # Orientation arrows
    if show_orientation:
        arrow_scale = 0.000009
        arrow_lats, arrow_lons = [], []
        for lat, lon, yaw in zip(lats, lons, yaws):
            dlat = math.cos(yaw) * arrow_scale
            dlon = math.sin(yaw) * arrow_scale / max(math.cos(math.radians(lat)), 1e-6)
            arrow_lats += [lat, lat + dlat, None]
            arrow_lons += [lon, lon + dlon, None]
        data.append({
            "type": "scattermapbox",
            "lat": arrow_lats,
            "lon": arrow_lons,
            "mode": "lines",
            "line": {"width": 2, "color": "orange"},
            "name": "Yaw",
            "showlegend": False
        })
    return {
        "data": data,
        "layout": {
            "mapbox": {
                "style": "open-street-map",
                "center": {"lat": lats[0], "lon": lons[0]},
                "zoom": 17,
            },
            "margin": {"l": 0, "r": 0, "t": 30, "b": 0},
            "title": os.path.basename(yaml_path),
            "height": 500,
        }
    }

# "Use Last" and dropdown management
@app.callback(
    Output("using-last-store", "data"),
    Output("using-last-indicator", "style"),
    Output("trajectory-dropdown", "value"),
    Output("trajectory-dropdown", "options"),
    Output("trajectory-dropdown", "placeholder"),
    Input("use-last-btn", "n_clicks"),
    Input("trajectory-dropdown", "value"),
    State("trajectory-dropdown", "options"),
    State("using-last-store", "data"),
    prevent_initial_call=True,
)
def manage_use_last_and_dropdown(use_last_clicks, dropdown_value, options, is_using_last):
    triggered = ctx.triggered_id
    if triggered == "use-last-btn":
        using_last = True
        style = {"display": "inline-block", "margin-bottom": "10px"}
        options = [{"label": os.path.basename(f), "value": f} for f in get_trajectories()]
        return using_last, style, "USING_LAST", options, "Using last recording"
    if triggered == "trajectory-dropdown":
        if dropdown_value != "USING_LAST":
            return False, {"display": "none"}, dropdown_value, options, "Select a trajectory"
        else:
            return True, {"display": "inline-block", "margin-bottom": "10px"}, "USING_LAST", options, "Using last recording"
    return is_using_last, {"display": "inline-block" if is_using_last else "none", "margin-bottom": "10px"}, dropdown_value, options, "Using last recording" if is_using_last else "Select a trajectory"

# Reverse button toggle & indicator
@app.callback(
    Output("reverse-store", "data"),
    Output("reverse-indicator", "style"),
    Input("reverse-btn", "n_clicks"),
    State("reverse-store", "data"),
    prevent_initial_call=True
)
def toggle_reverse(n, is_reverse):
    is_reverse = not is_reverse
    style = {"display": "inline-block", "margin-bottom": "10px"} if is_reverse else {"display": "none"}
    return is_reverse, style

# Play trajectory: launches follower
@app.callback(
    Output("status", "children", allow_duplicate=True),
    Input("play-btn", "n_clicks"),
    State("mode-radio", "value"),
    State("trajectory-dropdown", "value"),
    State("using-last-store", "data"),
    State("reverse-store", "data"),
    prevent_initial_call=True
)
def play_trajectory(n, mode, dropdown_value, using_last, reverse):
    if process_running.is_set():
        return dbc.Alert("Another process is already running!", color="warning")
    if using_last or dropdown_value == "USING_LAST":
        arg = "--last"
        traj_display = "last recording"
    elif dropdown_value:
        arg = dropdown_value
        traj_display = os.path.basename(dropdown_value)
    else:
        return dbc.Alert("Please select a trajectory!", color="warning")
    if mode == "precise":
        script = PRECISE_SCRIPT
    elif mode == "smooth":
        script = SMOOTH_SCRIPT
    else:
        script = INTERACTIVE_SCRIPT
    cmd = ["python3", script]
    if arg == "--last":
        cmd.append("--last")
    else:
        cmd.append(arg)
    if reverse:
        cmd.append("--reverse")
    try:
        start_process(cmd, "follower")
        mode_label = {"precise": "Precise", "smooth": "Smooth", "interactive": "Interactive"}[mode]
        details = f"Playing {traj_display} in {mode_label} mode"
        if reverse:
            details += " (Reverse)"
        return dbc.Alert(details, color="info")
    except Exception as e:
        return dbc.Alert(f"Error: {e}", color="danger")

# Start logger (record trajectory)
@app.callback(
    Output("status", "children", allow_duplicate=True),
    Input("record-btn", "n_clicks"),
    prevent_initial_call=True
)
def start_recording(n):
    if process_running.is_set():
        return dbc.Alert("Another process is already running!", color="warning")
    start_process(['python3', RECORD_SCRIPT], "logger")
    return dbc.Alert("Recording started!", color="success")

# Stop logger or follower
@app.callback(
    Output("status", "children", allow_duplicate=True),
    Input("stop-process-btn", "n_clicks"),
    prevent_initial_call=True
)
def stop_process_callback(n):
    if not process_running.is_set():
        return dbc.Alert("No process is running.", color="warning")
    stop_process()
    return dbc.Alert("Process stopped.", color="info")

# Combined live output
@app.callback(
    Output('live-output', 'children'),
    Input('live-output-interval', 'n_intervals')
)
def update_live_output(_):
    return get_combined_live_log()

if __name__ == "__main__":
    app.run(debug=True, host="0.0.0.0", port=8055)
