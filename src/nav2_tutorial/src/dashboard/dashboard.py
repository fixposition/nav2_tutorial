import dash
import dash_bootstrap_components as dbc
from dash import html, dcc, Output, Input, State, ctx
import os
import subprocess
import glob
import threading
import queue

# ----- Constants -----
TRAJECTORY_DIR = '/home/dev/ros_ws/src/nav2_tutorial/trajectories'
RECORD_SCRIPT = '/home/dev/ros_ws/src/nav2_tutorial/src/loggers/gps_periodic_logger.py'
PRECISE_SCRIPT = '/home/dev/ros_ws/src/nav2_tutorial/src/precise_wp_follower.py'
SMOOTH_SCRIPT = '/home/dev/ros_ws/src/nav2_tutorial/src/smooth_wp_follower.py'
INTERACTIVE_SCRIPT = '/home/jetson/dev/nav2_tutorial/src/nav2_tutorial/src/interactive_wp_follower.py'

def get_trajectories():
    return sorted(glob.glob(os.path.join(TRAJECTORY_DIR, '*.yaml')))

# --- Logger process management ---
logger_process = None
log_queue = queue.Queue()
log_buffer = []
LOG_BUFFER_MAX_LINES = 100
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

# ----- App Layout -----
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])

app.layout = dbc.Container([
    html.H2("Trajectory Dashboard"),
    dbc.Row([
        dbc.Col([
            html.H4("Trajectories"),
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
            dbc.Button("Record Trajectory", id="record-btn", color="success", style={"width": "100%", "margin-bottom": "10px"}),
            dbc.Button("Stop Recording", id="stop-record-btn", color="danger", style={"width": "100%", "margin-bottom": "10px"}),
        ], width=3),
        dbc.Col([
            html.H4("Controls"),
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
            html.Div(id="status", style={"margin-top": "20px"}),
            html.Hr(),
            html.H5("Live Logger Output"),
            html.Pre(id='live-log', style={'height': '300px', 'overflow': 'auto', 'background': '#1a1a1a', 'color': '#22ee22'}),
            dcc.Interval(id='log-interval', interval=1000, n_intervals=0),
        ], width=9),
    ]),
    # Hidden stores for state
    dcc.Store(id="using-last-store", data=False),
    dcc.Store(id="reverse-store", data=False),
])

# ----- CALLBACKS -----

# "Use Last" and dropdown management (fixes all Output collision issues)
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

    # Fallback (should not happen)
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

# Play trajectory logic
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
    if using_last or dropdown_value == "USING_LAST":
        arg = "--last"
        traj_display = "last recording"
    elif dropdown_value:
        arg = dropdown_value
        traj_display = os.path.basename(dropdown_value)
    else:
        return dbc.Alert("Please select a trajectory!", color="warning")

    # Pick script path
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
        subprocess.Popen(cmd)
        mode_label = {"precise": "Precise", "smooth": "Smooth", "interactive": "Interactive"}[mode]
        details = f"Playing {traj_display} in {mode_label} mode"
        if reverse:
            details += " (Reverse)"
        return dbc.Alert(details, color="info")
    except Exception as e:
        return dbc.Alert(f"Error: {e}", color="danger")

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

# Live logger output
@app.callback(
    Output('live-log', 'children'),
    Input('log-interval', 'n_intervals')
)
def update_log(_):
    return get_live_log()

if __name__ == "__main__":
    app.run(debug=True, host="0.0.0.0", port=8051)
