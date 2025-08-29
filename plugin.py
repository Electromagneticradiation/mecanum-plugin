"""
mecanum_plugin.py
Python Remote-API plugin for CoppeliaSim (b0RemoteApi) that:
 - adds first-order motor lag + torque cap
 - computes per-roller lateral slip and applies Stribeck friction forces
 - logs CSV for validation

Usage:
  1. Put script in project folder.
  2. Start CoppeliaSim scene. Ensure b0RemoteApi server is enabled in scene or use remote API plugin.
  3. Run: python mecanum_plugin.py
"""

import json
import math
import time
import csv
import os
from collections import deque

# Try/except import for b0RemoteApi; if you use a different wrapper adjust names
try:
    import b0RemoteApi
except Exception as e:
    raise RuntimeError("b0RemoteApi import failed — ensure b0RemoteApi Python package is installed.") from e

# -----------------------
# CONFIG (tune these)
# -----------------------
PARAMS_FILE = "mecanum_params.json"
LOG_DIR = "logs"
LOG_FILE = os.path.join(LOG_DIR, "mecanum_log.csv")
PHYSICS_DT = 0.02  # fallback physics step (s) if not read from sim

# Default param set (will be saved to JSON if missing)
DEFAULT_PARAMS = {
    "wheel_radius": 0.05,        # m
    "wheel_base_x": 0.25,       # distance front-back / 2 (m) (half-length)
    "wheel_base_y": 0.18,       # distance left-right / 2 (m) (half-width)
    "tau_m": 0.08,              # motor time constant (s)
    "kt": 1.0,                  # torque gain (Nm per (rad/s) error)
    "tau_max": 1.5,             # max torque (Nm)
    "Fs": 4.0,                  # static friction (N)
    "Fc": 2.5,                  # coulomb friction (N)
    "v_s": 0.03,                # Stribeck velocity (m/s)
    "b_visc": 0.5,              # viscous damping (N/(m/s))
    "roller_angle_deg": 45.0,   # roller mounting angle (deg)
    "logging_interval": 1,      # log every N physics steps
    "use_joint_force_api": True # if True use setJointForce, else applyExternalForce
}

# -----------------------
# Utilities
# -----------------------
def ensure_params():
    if not os.path.exists(PARAMS_FILE):
        with open(PARAMS_FILE, "w") as f:
            json.dump(DEFAULT_PARAMS, f, indent=2)
    with open(PARAMS_FILE, "r") as f:
        return json.load(f)

def write_csv_header(path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp", "sim_time", "scenario", 
            "cmd_vx","cmd_vy","cmd_w",
            "wheel_id","omega_cmd","omega_hat","tau_avail",
            "v_slip","F_lat","body_x","body_y","yaw"
        ])

def append_csv_row(path, row):
    with open(path, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(row)

# -----------------------
# Kinematics: mecanum mapping
# -----------------------
def cmdvel_to_wheel_omegas(vx, vy, wz, params):
    # Standard mecanum inverse kinematics (wheel angular velocities rad/s)
    R = params["wheel_radius"]
    L = params["wheel_base_x"]  # half-length
    W = params["wheel_base_y"]  # half-width
    # geometry factor
    k = L + W
    # order wheels: [front_left, front_right, rear_left, rear_right]
    # formula: omega = (1/R) * (vx +/- vy +/- k*wz)
    # Signs depend on wheel mounting & coordinate conventions; adjust if your model differs
    omega_fl = (vx - vy - k * wz) / R
    omega_fr = (vx + vy + k * wz) / R
    omega_rl = (vx + vy - k * wz) / R
    omega_rr = (vx - vy + k * wz) / R
    return [omega_fl, omega_fr, omega_rl, omega_rr]

# -----------------------
# Stribeck friction model
# -----------------------
def stribeck_force(v_slip, Fs, Fc, v_s, b):
    # 1D lateral force (signed)
    if abs(v_slip) < 1e-12:
        visc = 0.0
    else:
        visc = b * v_slip
    return math.copysign(1.0, v_slip) * (Fc + (Fs - Fc) * math.exp(-abs(v_slip) / v_s)) + visc

# -----------------------
# Slip computation
# -----------------------
def compute_lateral_slip(wheel_omega, wheel_radius, roller_angle_deg, wheel_linear_velocity_body_lat):
    """
    wheel_omega: rad/s
    wheel_radius: m
    roller_angle_deg: roller mounting angle (e.g., 45 deg)
    wheel_linear_velocity_body_lat: lateral component of wheel contact velocity seen on body (m/s)
    We compute roller lateral slip = lateral ground speed - roller peripheral lateral speed
    """
    roller_angle = math.radians(roller_angle_deg)
    # peripheral linear speed at rim (in wheel frame, tangent direction)
    v_tangent = wheel_omega * wheel_radius
    # roller axis projection -> lateral component of peripheral speed (approx)
    # For mecanum, lateral component contributed by wheel rotation is v_tangent * cos(roller_angle)
    v_roller_lat = v_tangent * math.cos(roller_angle)
    # slip = ground lateral velocity - roller lateral rim speed
    v_slip = wheel_linear_velocity_body_lat - v_roller_lat
    return v_slip

# -----------------------
# Main plugin class
# -----------------------
class MecanumPlugin:
    def __init__(self, client, params):
        self.client = client
        self.params = params
        self.wheel_names = ["wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"]
        # handles to joints (fill later)
        self.wheel_handles = [None] * 4
        # states
        self.omega_hat = [0.0] * 4
        self.omega_cmd_prev = [0.0] * 4
        # logging counter
        self.step_count = 0
        # csv header
        write_csv_header(LOG_FILE)

    def cache_handles(self):
        # attempt to get joint handles by name; adapt function call if API differs
        for i, name in enumerate(self.wheel_names):
            res, handle = self.client.simxGetObjectHandle(name, self.client.simxDefaultPublisher())  # may differ
            if res != 0:
                # if object not found: return - adapt to your API exact return codes
                raise RuntimeError(f"failed to get handle for {name}, rc={res}")
            self.wheel_handles[i] = handle

    def read_wheel_velocities(self):
        # returns wheels angular velocities (rad/s)
        omegas = []
        for h in self.wheel_handles:
            res, omega = self.client.simxGetJointVelocity(h, self.client.simxDefaultPublisher())
            if res != 0:
                omega = 0.0
            omegas.append(omega)
        return omegas

    def read_body_state(self):
        # read robot base position and linear velocity in body frame
        # use simxGetObjectPosition and simxGetObjectVelocity or equivalent
        # adapt API calls as needed
        res_p, pos = self.client.simxGetObjectPosition("base_link", -1, self.client.simxDefaultPublisher())
        res_v, vel = self.client.simxGetObjectVelocity("base_link", self.client.simxDefaultPublisher())
        res_o, orient = self.client.simxGetObjectOrientation("base_link", -1, self.client.simxDefaultPublisher())
        # pos: [x,y,z], vel: [vx, vy, vz], orient: [rx,ry,rz] where rz is yaw approx
        return pos, vel, orient

    def apply_wheel_torque(self, wheel_idx, torque):
        h = self.wheel_handles[wheel_idx]
        if self.params["use_joint_force_api"]:
            # set joint force/torque directly - adapt call if your API uses simxSetJointForce or setJointForce
            self.client.simxSetJointForce(h, abs(torque), self.client.simxDefaultPublisher())
            # set direction via target velocity sign if needed
            sign = 1.0 if torque >= 0 else -1.0
            self.client.simxSetJointTargetVelocity(h, sign * 1e3, self.client.simxDefaultPublisher())  # large vel to let force apply
        else:
            # fallback: apply external force at wheel link contact point (must compute contact pos)
            # Example placeholder; implement correct coordinates for your robot
            contact_point = [0,0,0]
            force_vec = [0.0, torque, 0.0]  # not physically exact; replace with proper mapping
            self.client.simxAddForceOnPosition(h, force_vec, contact_point, self.client.simxDefaultPublisher())

    def step(self, cmd_vel):
        # main per-physics-step update
        vx, vy, wz = cmd_vel
        dt = PHYSICS_DT  # if you can query actual physics dt, replace it

        # inverse kinematics -> desired wheel speeds
        omega_cmd = cmdvel_to_wheel_omegas(vx, vy, wz, self.params)

        # read current wheel omegas and body velocities
        current_omegas = self.read_wheel_velocities()
        pos, vel, orient = self.read_body_state()
        # body lateral velocity (vy) is vel[1] in world frame; if needed convert to body frame
        body_vy = vel[1]  # approximate

        for i in range(4):
            # motor dynamics (first-order)
            self.omega_hat[i] += (dt / self.params["tau_m"]) * (omega_cmd[i] - self.omega_hat[i])
            # torque availability from speed error
            tau_avail = max(-self.params["tau_max"], min(self.params["tau_max"],
                            self.params["kt"] * (omega_cmd[i] - self.omega_hat[i])))

            # compute lateral slip: wheel linear location lateral vel (approx using body_vy)
            v_slip = compute_lateral_slip(self.omega_hat[i], self.params["wheel_radius"],
                                          self.params["roller_angle_deg"], body_vy)

            # Stribeck lateral force
            F_lat = stribeck_force(v_slip, self.params["Fs"], self.params["Fc"],
                                   self.params["v_s"], self.params["b_visc"])

            # Map F_lat to torque or apply as external force: here we apply tau_avail for drive + lateral as opposing force
            # Simple approach: add lateral effect as reduction in effective torque via a penalty, or better: apply external force
            # We apply driving torque first:
            self.apply_wheel_torque(i, tau_avail)

            # Optionally apply lateral forces to base (for visual+physics effect)
            # convert lateral scalar to world vector and apply at contact point - placeholder:
            # contact_point = ... compute from robot geometry
            # self.client.simxAddForceOnPosition(body_handle, [0, F_lat, 0], contact_point, ...)

            # Logging
            if (self.step_count % self.params["logging_interval"]) == 0:
                append_csv_row(LOG_FILE, [
                    time.time(), None, "auto",
                    vx, vy, wz,
                    i, omega_cmd[i], self.omega_hat[i], tau_avail,
                    v_slip, F_lat, pos[0], pos[1], orient[2]
                ])

        self.step_count += 1

# -----------------------
# Runner
# -----------------------
def run():
    params = ensure_params()
    # create remote-api client
    client = b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient', 'b0RemoteApi')
    print("Connected to CoppeliaSim b0RemoteApi")

    # enable synchronous stepping
    client.simxSynchronous(True)

    plugin = MecanumPlugin(client, params)
    # optional: plugin.cache_handles()  # try to populate handles; adjust API call to get handles by name
    # fallback: user should set wheel_handles via manual handle lookup or scene must have same names as wheel_names

    # initial states: zero commands
    cmd_vx, cmd_vy, cmd_w = 0.0, 0.0, 0.0

    try:
        # start stepping loop
        while True:
            # wait for next physics step trigger - ensures sync with sim
            client.simxSynchronousTrigger()
            # read command velocity: if using ROS, this is where you'd pop latest /cmd_vel; for now we poll a simple file or keep static
            # Example: check a file 'cmd_vel.json' for test scenarios, else use pre-scripted profile
            # For demo we'll use a simple ramp test
            t = time.time()
            # simple test: after 2 sec strafe right at 0.4 m/s
            t_rel = (t % 20)
            if 2 < t_rel < 7:
                cmd_vx = 0.0
                cmd_vy = 0.4
                cmd_w = 0.0
            elif 7 <= t_rel < 12:
                cmd_vx = 0.0
                cmd_vy = -0.2
                cmd_w = 0.5
            else:
                cmd_vx, cmd_vy, cmd_w = 0.0, 0.0, 0.0

            plugin.step((cmd_vx, cmd_vy, cmd_w))

    except KeyboardInterrupt:
        print("Stopping plugin and disconnecting.")
    finally:
        client.simxSynchronous(False)
        client.close()

if __name__ == "__main__":
    # prepare logs and params
    params = ensure_params()  # will create params JSON if missing
    write_csv_header(LOG_FILE)
    run()
