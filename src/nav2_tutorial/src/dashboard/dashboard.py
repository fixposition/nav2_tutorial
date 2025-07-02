import dash
import dash_bootstrap_components as dbc
from dash import html, dcc, Output, Input, State
import os
import subprocess
import glob
import threading
import queue
import time

# Globals
log_queue = queue.Queue()
log_buffer = []
LOG_BUFFER_MAX_LINES = 100
TRAJECTORY_DIR = '/home/dev/ros_ws/src/nav2_tutorial/trajectories'
RECORD_SCRIPT = '/home/dev/ros_ws/src/nav2_tutorial/src/loggers/gps_periodic_logger.py'
PRECISE_SCRIPT = '/home/dev/ros_ws/src/nav2_tutorial/src/precise_wp_follower.py'
SMOOTH_SCRIPT = '/home/dev/ros_ws/src/nav2_tutorial/src/smooth_wp_follower.py'

def get_trajectories():
    return sorted(glob.glob(os.path.join(TRAJECTORY_DIR, '*.yaml')))

# --- Logger process management ---
logger_process = None
log_queue = queue.Queue()
logger_thread = None
logger_running = threading.Event()

def run_logger_process():
    global logger_process
    logger_running.set()
    logger_process = subprocess.Popen(
        ['python3', RECORD_SCRIPT],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1
    )
    try:
        for line in logger_process.stdout:
            log_queue.put(line)
    except Exception as e:
        log_queue.put(f"[Logger error]: {e}\n")
    logger_running.clear()

def start_logger():
    global logger_thread, logger_running
    if logger_running.is_set():
        return  # Already running
    logger_thread = threading.Thread(target=run_logger_process, daemon=True)
    logger_thread.start()

def stop_logger():
    global logger_process, logger_running
    if logger_process and logger_running.is_set():
        logger_process.terminate()
        logger_running.clear()
        log_queue.put("[Logger stopped]\n")

def get_live_log():
    while not log_queue.empty():
        line = log_queue.get()
        log_buffer.append(line)
        if len(log_buffer) > LOG_BUFFER_MAX_LINES:
            log_buffer.pop(0)
    return ''.join(log_buffer)

# --- Dash app setup ---
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])

app.layout = dbc.Container([
    html.H2("Trajectory Dashboard"),
    dbc.Row([
        dbc.Col([
            html.H4("Recorded Trajectories"),
            dcc.Dropdown(
                id="trajectory-dropdown",
                options=[{"label": os.path.basename(f), "value": f} for f in get_trajectories()],
                placeholder="Select a trajectory",
                style={"margin-bottom": "10px"}
            ),
            dbc.Button("Use Last", id="use-last-btn", color="secondary", style={"width": "100%", "margin-bottom": "20px"}),
            dbc.Button("Record Trajectory", id="record-btn", color="success", style={"width": "100%", "margin-bottom": "10px"}),
            dbc.Button("Stop Recording", id="stop-record-btn", color="danger", style={"width": "100%", "margin-bottom": "10px"}),
        ], width=3),
        dbc.Col([
            html.H4("Controls"),
            html.Div([
                html.Label("Mode:"),
                dcc.Slider(
                    id="mode-slider",
                    min=0, max=1, step=1,
                    marks={0: "Precise", 1: "Smooth"},
                    value=0,
                )
            ], style={"margin-bottom": "20px"}),
            dbc.Button("Play Trajectory", id="play-btn", color="primary", style={"width": "100%"}),
            html.Div(id="status", style={"margin-top": "20px"}),
            html.Hr(),
            html.H5("Live Logger Output"),
            html.Pre(id='live-log', style={'height': '300px', 'overflow': 'auto', 'background': '#1a1a1a', 'color': '#22ee22'}),
            dcc.Interval(id='log-interval', interval=1000, n_intervals=0),  # Refresh logs every second
        ], width=9),
    ])
], fluid=True)

# Refresh dropdown when new trajectories are created
@app.callback(
    Output("trajectory-dropdown", "options"),
    Input("record-btn", "n_clicks"),
    Input("play-btn", "n_clicks"),
    Input("use-last-btn", "n_clicks"),
    prevent_initial_call=True
)
def refresh_trajectories(*_):
    return [{"label": os.path.basename(f), "value": f} for f in get_trajectories()]

# Start recording
@app.callback(
    Output("status", "children", allow_duplicate=True),
    Input("record-btn", "n_clicks"),
    prevent_initial_call=True
)
def start_recording(n):
    if logger_running.is_set():
        return dbc.Alert("Logger already running!", color="warning")
    start_logger()
    return dbc.Alert("Recording started!", color="success")

# Stop recording
@app.callback(
    Output("status", "children", allow_duplicate=True),
    Input("stop-record-btn", "n_clicks"),
    prevent_initial_call=True
)
def stop_recording(n):
    if not logger_running.is_set():
        return dbc.Alert("Logger is not running!", color="warning")
    stop_logger()
    return dbc.Alert("Recording stopped.", color="info")

# Play trajectory
@app.callback(
    Output("status", "children", allow_duplicate=True),
    Input("play-btn", "n_clicks"),
    State("trajectory-dropdown", "value"),
    State("mode-slider", "value"),
    prevent_initial_call=True
)
def play_trajectory(n, yaml_file, mode):
    if not yaml_file:
        return dbc.Alert("Please select a trajectory!", color="warning")
    script = PRECISE_SCRIPT if mode == 0 else SMOOTH_SCRIPT
    try:
        subprocess.Popen(['python3', script, '--file', yaml_file])
        return dbc.Alert(f"Playing {os.path.basename(yaml_file)} in {'Precise' if mode == 0 else 'Smooth'} mode", color="info")
    except Exception as e:
        return dbc.Alert(f"Error: {e}", color="danger")

# Use last trajectory
@app.callback(
    Output("trajectory-dropdown", "value"),
    Output("status", "children", allow_duplicate=True),
    Input("use-last-btn", "n_clicks"),
    prevent_initial_call=True
)
def use_last(n):
    files = get_trajectories()
    if not files:
        return dash.no_update, dbc.Alert("No trajectories found!", color="warning")
    last_file = files[-1]
    return last_file, dbc.Alert(f"Selected last trajectory: {os.path.basename(last_file)}", color="info")

# Live logger output
@app.callback(
    Output('live-log', 'children'),
    Input('log-interval', 'n_intervals')
)
def update_log(_):
    return get_live_log()

if __name__ == "__main__":
    app.run(debug=True, host="0.0.0.0", port=8051)
