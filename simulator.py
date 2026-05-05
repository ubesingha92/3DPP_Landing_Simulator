import csv
import os
import signal
import tempfile
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

# =============================================================================
# User-editable simulation settings
# =============================================================================

# Set True to watch the PyBullet GUI. Set False for faster headless runs.
VISUALIZE = True

# Number of parallel CPU workers used when VISUALIZE is False.
MAX_PARALLEL_WORKERS = max(1, (os.cpu_count() or 1) - 1)

# Platform base-size sweep in metres.
BASE_SIZE_MIN = 0.100
BASE_SIZE_MAX = 0.400
BASE_SIZE_INTERVAL = 0.01

# Platform angle sweep in degrees.
PLATFORM_ANGLE_MIN_DEG = 30
PLATFORM_ANGLE_MAX_DEG = 75
PLATFORM_ANGLE_INTERVAL_DEG = 15

# Spacing between tested drone start positions in the landing grid, in metres.
LANDING_GRID_RESOLUTION = 0.005

# Gravity, physics time step, and attempt timing.
GRAVITY = 9.81
TIME_STEP = 0.001
SETTLE_DURATION = 1.0
MAX_LANDING_ATTEMPT_TIME = 3.0

# Drone release clearance above the shallowest platform peak, in metres.
DROP_CLEARANCE = 0.04

# Landing-leg length sweep in metres.
DRONE_ARM_LENGTH_MIN = 0.300
DRONE_ARM_LENGTH_MAX = 0.600
DRONE_ARM_LENGTH_INTERVAL = 0.01

# Landing-leg geometry and mass used when generating the temporary drone URDF.
DRONE_ARM_THICKNESS = 0.002
DRONE_ARM_MASS = 0.01

# Landing success criteria.
SUCCESS_POSITION_LIMIT = 0.005
MAX_TILT_ANGLE_DEG = 10
MIN_LANDING_CONTACTS = 1

# Heatmap colors for successful, collision-success, and unsuccessful attempts.
RISK_COLORS = ["#62BB46", "#FFC20E", "#F15B40"]
RISK_LABELS = ["Success", "Success but collision detected", "Unsuccessful"]


# =============================================================================
# Derived values and output paths
# =============================================================================

SCRIPT_DIR = Path(__file__).resolve().parent
URDF_DIR = SCRIPT_DIR / "urdf"
CSV_OUTPUT_DIR = SCRIPT_DIR / "csv_results"
PLOT_OUTPUT_DIR = SCRIPT_DIR / "plot_results"
SUCCESS_RATE_FILE = CSV_OUTPUT_DIR / "success_rate_summary.csv"

BASE_SIZES = np.round(
    np.arange(
        BASE_SIZE_MIN,
        BASE_SIZE_MAX + BASE_SIZE_INTERVAL / 10,
        BASE_SIZE_INTERVAL,
    ),
    3,
).tolist()

ANGLES = [
    int(angle)
    for angle in np.arange(
        PLATFORM_ANGLE_MIN_DEG,
        PLATFORM_ANGLE_MAX_DEG + PLATFORM_ANGLE_INTERVAL_DEG / 10,
        PLATFORM_ANGLE_INTERVAL_DEG,
    )
]

DRONE_ARM_LENGTHS = np.round(
    np.arange(
        DRONE_ARM_LENGTH_MIN,
        DRONE_ARM_LENGTH_MAX + DRONE_ARM_LENGTH_INTERVAL / 10,
        DRONE_ARM_LENGTH_INTERVAL,
    ),
    3,
).tolist()
DEFAULT_DRONE_ARM_LENGTH = DRONE_ARM_LENGTHS[0]

# Maximum positional drift (metres) allowed during the settle window for the
# drone to be considered stationary.  This is an independent physics tolerance
# and must not be coupled to construction parameters like arm thickness.
SETTLE_DRIFT_LIMIT = 0.002
LINEAR_VELOCITY_LIMIT = SETTLE_DRIFT_LIMIT / SETTLE_DURATION
MAX_TILT_ANGLE = np.radians(MAX_TILT_ANGLE_DEG)


# =============================================================================
# Internal labels and PyBullet link identifiers
# =============================================================================

SUCCESS_RESULT = "Successful"
COLLISION_RESULT = "Success but collision detected"
LEGACY_COLLISION_RESULT = "Collision detected"
UNSUCCESSFUL_RESULT = "Unsuccessful"
SUCCESSFUL_RESULTS = {SUCCESS_RESULT, COLLISION_RESULT}
RESULT_ORDER = [SUCCESS_RESULT, COLLISION_RESULT, UNSUCCESSFUL_RESULT]

SUCCESS_RATE_FIELDNAMES = [
    "Base size (m)",
    "Angle (deg)",
    "Drone leg length (m)",
    "Max 100% round dimension - Successful only (m)",
    "Max 100% round dimension - Successful only of base (%)",
    "Max 100% round dimension - Success + collision (m)",
    "Max 100% round dimension - Success + collision of base (%)",
    "Total attempts",
    "Successful attempts",
    "Success + collision attempts",
    "Unsuccessful attempts",
    "Success rate (%)",
    "Success + collision rate (%)",
]

# These names must match link names in urdf/drone.urdf.
DRONE_ARM_LINK_NAMES = {"link2", "link3"}
DRONE_PROPELLER_LINK_NAMES = {
    "propeller_front_right",
    "propeller_front_left",
    "propeller_rear_left",
    "propeller_rear_right",
}

DRONE_BODY_LINK_INDEX = -1
# PyBullet contact-tuple index for bodyA's link index.  Only valid when the
# drone is passed as bodyA in getContactPoints() calls.
CONTACT_TUPLE_LINK_A_INDEX = 3
# PyBullet contact-tuple index for bodyB's unique ID.
CONTACT_TUPLE_BODY_B_INDEX = 2
DESCENT_EPSILON = 0.001
FLOAT_TOLERANCE = 1e-9

p = None
pybullet_data = None


class ProgressBar:
    def __init__(self, total, label):
        self.total = max(1, total)
        self.label = label
        self.rendered = False

    def update(self, completed):
        width = 32
        completed = min(completed, self.total)
        filled = int(width * completed / self.total)
        bar = "#" * filled + "-" * (width - filled)
        percent = (completed / self.total) * 100
        print(
            f"\r{self.label}: |{bar}| {completed}/{self.total} ({percent:5.1f}%)",
            end="",
            flush=True,
        )
        self.rendered = True

    def close(self):
        if self.rendered:
            print()
            self.rendered = False


def format_parameter(value):
    if isinstance(value, (float, np.floating)):
        text = f"{float(value):g}"
    else:
        text = str(value)
    return text.replace("-", "neg").replace(".", "p")


def format_urdf_float(value):
    return f"{float(value):.10g}"


def validate_positive_dimension(name, value):
    if value <= 0:
        raise ValueError(f"{name} must be positive, got {value!r}")


def get_box_inertia(mass, x_size, y_size, z_size):
    scale = mass / 12
    return {
        "ixx": scale * (y_size**2 + z_size**2),
        "ixy": 0,
        "ixz": 0,
        "iyy": scale * (x_size**2 + z_size**2),
        "iyz": 0,
        "izz": scale * (x_size**2 + y_size**2),
    }


def update_drone_arm_link(link, drone_arm_length):
    size = (
        f"{format_urdf_float(drone_arm_length)} "
        f"{format_urdf_float(DRONE_ARM_THICKNESS)} "
        f"{format_urdf_float(DRONE_ARM_THICKNESS)}"
    )

    for box in link.findall("./collision/geometry/box"):
        box.set("size", size)
    for box in link.findall("./visual/geometry/box"):
        box.set("size", size)

    mass = link.find("./inertial/mass")
    if mass is not None:
        mass.set("value", format_urdf_float(DRONE_ARM_MASS))

    inertia = link.find("./inertial/inertia")
    if inertia is not None:
        for key, value in get_box_inertia(
            DRONE_ARM_MASS,
            drone_arm_length,
            DRONE_ARM_THICKNESS,
            DRONE_ARM_THICKNESS,
        ).items():
            inertia.set(key, format_urdf_float(value))


def get_parameterized_drone_urdf_path(drone_arm_length=DEFAULT_DRONE_ARM_LENGTH):
    validate_positive_dimension("drone_arm_length", drone_arm_length)
    validate_positive_dimension("DRONE_ARM_THICKNESS", DRONE_ARM_THICKNESS)
    validate_positive_dimension("DRONE_ARM_MASS", DRONE_ARM_MASS)

    tree = ET.parse(URDF_DIR / "drone.urdf")
    root = tree.getroot()
    updated_links = set()

    for link in root.findall("link"):
        link_name = link.get("name")
        if link_name in DRONE_ARM_LINK_NAMES:
            update_drone_arm_link(link, drone_arm_length)
            updated_links.add(link_name)

    missing_links = DRONE_ARM_LINK_NAMES - updated_links
    if missing_links:
        missing = ", ".join(sorted(missing_links))
        raise ValueError(f"Missing drone arm link(s) in drone.urdf: {missing}")

    generated_dir = Path(tempfile.gettempdir()) / "3dpp_landing_simulator"
    generated_dir.mkdir(parents=True, exist_ok=True)
    generated_file = generated_dir / (
        f"drone_arm_{format_parameter(drone_arm_length)}m_"
        f"thickness_{format_parameter(DRONE_ARM_THICKNESS)}m_"
        f"pid_{os.getpid()}.urdf"
    )
    tree.write(generated_file, encoding="utf-8", xml_declaration=True)
    return generated_file


def get_pybullet():
    global p, pybullet_data

    if p is None:
        import contextlib
        import io
        import sys

        try:
            stdout_fd = sys.stdout.fileno()
            stderr_fd = sys.stderr.fileno()
            saved_stdout_fd = os.dup(stdout_fd)
            saved_stderr_fd = os.dup(stderr_fd)

            try:
                with open(os.devnull, "w") as devnull:
                    os.dup2(devnull.fileno(), stdout_fd)
                    os.dup2(devnull.fileno(), stderr_fd)
                    import pybullet as pybullet_module

            finally:
                os.dup2(saved_stdout_fd, stdout_fd)
                os.dup2(saved_stderr_fd, stderr_fd)
                os.close(saved_stdout_fd)
                os.close(saved_stderr_fd)

        except (AttributeError, OSError, ValueError):
            with contextlib.redirect_stdout(io.StringIO()):
                with contextlib.redirect_stderr(io.StringIO()):
                    import pybullet as pybullet_module

        import pybullet_data as pybullet_data_module

        p = pybullet_module
        pybullet_data = pybullet_data_module

    return p


def get_case_stem(base_size, angle, drone_arm_length):
    return (
        f"base_{format_parameter(base_size)}m_"
        f"angle_{format_parameter(angle)}deg_"
        f"leg_{format_parameter(drone_arm_length)}m_"
        f"grid_res_{format_parameter(LANDING_GRID_RESOLUTION)}m"
    )


def normalize_result_values(df):
    if "Result" in df.columns:
        df["Result"] = df["Result"].replace({LEGACY_COLLISION_RESULT: COLLISION_RESULT})
    return df


def process_csv(filename):
    import pandas as pd

    # Replace unsuccessful landing times only when a successful reference exists.
    # Write the processed data to a separate file to preserve the raw simulation output.
    df = normalize_result_values(pd.read_csv(filename))
    min_time_success = df[df["Result"].isin(SUCCESSFUL_RESULTS)]["Time (s)"].min()

    if pd.notna(min_time_success):
        df.loc[df["Result"] == UNSUCCESSFUL_RESULT, "Time (s)"] = min_time_success

    raw_path = Path(filename)
    processed_path = raw_path.parent / f"{raw_path.stem}_processed{raw_path.suffix}"
    df.to_csv(processed_path, index=False)


def get_max_full_success_round_dimension(df, accepted_results):
    if df.empty:
        return 0.0

    result_by_radius = (
        df.assign(Radius=np.round(np.hypot(df["StartX"], df["StartY"]), 6))
        .sort_values("Radius")
        .groupby("Radius", sort=True)["Result"]
        .apply(lambda results: results.isin(accepted_results).all())
    )

    max_success_radius = 0.0
    for radius, is_successful_ring in result_by_radius.items():
        if not is_successful_ring:
            break
        max_success_radius = radius

    return round(2 * max_success_radius, 4)


def get_base_dimension_percentage(dimension, base_size):
    if base_size <= FLOAT_TOLERANCE:
        return 0.0
    return round((dimension / base_size) * 100, 2)


def get_success_summary(filename, base_size, angle, drone_arm_length):
    import pandas as pd

    raw_path = Path(filename)
    processed_path = raw_path.parent / f"{raw_path.stem}_processed{raw_path.suffix}"
    df = normalize_result_values(pd.read_csv(processed_path))
    total_attempts = len(df)
    successful_attempts = int((df["Result"] == SUCCESS_RESULT).sum())
    success_with_collision_attempts = int(df["Result"].isin(SUCCESSFUL_RESULTS).sum())
    unsuccessful_attempts = total_attempts - success_with_collision_attempts
    max_clean_success_dimension = get_max_full_success_round_dimension(
        df,
        {SUCCESS_RESULT},
    )
    max_success_with_collision_dimension = get_max_full_success_round_dimension(
        df,
        SUCCESSFUL_RESULTS,
    )
    success_rate = (
        (successful_attempts / total_attempts) * 100 if total_attempts else 0.0
    )
    success_with_collision_rate = (
        (success_with_collision_attempts / total_attempts) * 100
        if total_attempts
        else 0.0
    )

    return {
        "Base size (m)": round(base_size, 4),
        "Angle (deg)": angle,
        "Drone leg length (m)": round(drone_arm_length, 4),
        "Max 100% round dimension - Successful only (m)": max_clean_success_dimension,
        "Max 100% round dimension - Successful only of base (%)": get_base_dimension_percentage(
            max_clean_success_dimension,
            base_size,
        ),
        "Max 100% round dimension - Success + collision (m)": max_success_with_collision_dimension,
        "Max 100% round dimension - Success + collision of base (%)": get_base_dimension_percentage(
            max_success_with_collision_dimension,
            base_size,
        ),
        "Total attempts": total_attempts,
        "Successful attempts": successful_attempts,
        "Success + collision attempts": success_with_collision_attempts,
        "Unsuccessful attempts": unsuccessful_attempts,
        "Success rate (%)": round(success_rate, 2),
        "Success + collision rate (%)": round(success_with_collision_rate, 2),
    }


def initialize_success_rate_csv():
    CSV_OUTPUT_DIR.mkdir(exist_ok=True)
    with open(SUCCESS_RATE_FILE, "w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=SUCCESS_RATE_FIELDNAMES)
        writer.writeheader()


def append_success_summary(summary):
    with open(SUCCESS_RATE_FILE, "a", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=SUCCESS_RATE_FIELDNAMES)
        writer.writerow(summary)


def meters_to_mm(value):
    return float(value) * 1000


def format_mm(value):
    return f"{meters_to_mm(value):g}"


def get_plot_axis_positions_mm():
    # Intentionally uses max(BASE_SIZES) so that every heatmap has identical
    # axis limits, making plots directly comparable across different base sizes.
    return np.array(
        [
            round(meters_to_mm(position), 6)
            for position in get_axis_positions(max(BASE_SIZES))
        ]
    )


def get_plot_extent_mm(axis_positions_mm):
    half_cell_mm = meters_to_mm(LANDING_GRID_RESOLUTION) / 2
    return [
        axis_positions_mm[0] - half_cell_mm,
        axis_positions_mm[-1] + half_cell_mm,
        axis_positions_mm[0] - half_cell_mm,
        axis_positions_mm[-1] + half_cell_mm,
    ]


def create_plot(filename, summary):
    import logging
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import BoundaryNorm, ListedColormap
    from matplotlib.patches import Patch
    import pandas as pd

    logging.getLogger("matplotlib.font_manager").setLevel(logging.ERROR)
    plt.rcParams["font.family"] = "Times New Roman"

    raw_path = Path(filename)
    processed_path = raw_path.parent / f"{raw_path.stem}_processed{raw_path.suffix}"
    df = normalize_result_values(pd.read_csv(processed_path))
    axis_positions_mm = get_plot_axis_positions_mm()
    plot_extent_mm = get_plot_extent_mm(axis_positions_mm)

    # Map categorical results to numeric values for the heatmap grid.
    result_map = {SUCCESS_RESULT: 0, COLLISION_RESULT: 1, UNSUCCESSFUL_RESULT: 2}
    grid_size = len(axis_positions_mm)
    grid = np.full((grid_size, grid_size), np.nan)

    for _, row in df.iterrows():
        x_mm = round(row["StartX"] * 1000, 6)
        y_mm = round(row["StartY"] * 1000, 6)
        xi = np.searchsorted(axis_positions_mm, x_mm)
        yi = np.searchsorted(axis_positions_mm, y_mm)
        if 0 <= xi < grid_size and 0 <= yi < grid_size:
            grid[yi, xi] = result_map.get(row["Result"], 2)

    cmap = ListedColormap(RISK_COLORS)
    norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5], cmap.N)

    fig, ax = plt.subplots(figsize=(7, 6.2), constrained_layout=True)
    ax.imshow(
        grid,
        cmap=cmap,
        norm=norm,
        extent=plot_extent_mm,
        origin="lower",
        interpolation="nearest",
        aspect="equal",
    )

    ax.set_xlabel("X Axis (mm)")
    ax.set_ylabel("Y Axis (mm)")
    ax.set_xlim(plot_extent_mm[0], plot_extent_mm[1])
    ax.set_ylim(plot_extent_mm[2], plot_extent_mm[3])

    ax.set_title(
        f"Base {format_mm(summary['Base size (m)'])} mm, "
        f"Angle {summary['Angle (deg)']} deg, "
        f"Leg {format_mm(summary['Drone leg length (m)'])} mm"
    )

    legend_handles = [
        Patch(facecolor=color, label=label)
        for label, color in zip(RISK_LABELS, RISK_COLORS)
    ]
    ax.legend(handles=legend_handles, title="Result", loc="upper right", frameon=True)

    PLOT_OUTPUT_DIR.mkdir(exist_ok=True)
    plot_file = PLOT_OUTPUT_DIR / f"{Path(filename).stem}.png"

    fig.savefig(plot_file, dpi=300, bbox_inches="tight")
    plt.close(fig)


def step_simulation():
    p.stepSimulation()
    if VISUALIZE:
        time.sleep(TIME_STEP)


def advance_simulation(duration):
    for _ in range(max(1, int(round(duration / TIME_STEP)))):
        step_simulation()


def get_tilt_angle(orientation):
    body_up_z = p.getMatrixFromQuaternion(orientation)[8]
    return np.arccos(np.clip(body_up_z, -1.0, 1.0))


def get_active_contacts(body_a, body_b):
    # contactNormalForce (index 9) is the magnitude of the separating force.
    # Contacts with zero force exist in the solver manifold but are inactive;
    # filtering them out avoids counting non-pressing touch points.
    return [
        contact
        for contact in p.getContactPoints(bodyA=body_a, bodyB=body_b)
        if contact[9] > 0
    ]


def get_landing_base_contacts(drone, base_segment_ids):
    # Single getContactPoints call instead of one per segment.  The body-B ID
    # (index 2) is checked against the pre-built set of segment IDs.
    return [
        contact
        for contact in p.getContactPoints(bodyA=drone)
        if contact[9] > 0 and contact[CONTACT_TUPLE_BODY_B_INDEX] in base_segment_ids
    ]


def get_drone_link_indices(drone, link_names):
    link_indices = {}

    for joint_index in range(p.getNumJoints(drone)):
        joint_info = p.getJointInfo(drone, joint_index)
        link_name = joint_info[12].decode("utf-8")
        if link_name in link_names:
            link_indices[link_name] = joint_index

    missing_links = link_names - set(link_indices)
    if missing_links:
        missing = ", ".join(sorted(missing_links))
        raise ValueError(f"Missing drone link(s) in loaded URDF: {missing}")

    return set(link_indices.values())


def has_drone_body_contact(contacts):
    return any(
        contact[CONTACT_TUPLE_LINK_A_INDEX] == DRONE_BODY_LINK_INDEX
        for contact in contacts
    )


def has_drone_propeller_contact(contacts, propeller_link_indices):
    return any(
        contact[CONTACT_TUPLE_LINK_A_INDEX] in propeller_link_indices
        for contact in contacts
    )


def get_landing_arm_contacts(contacts, landing_arm_link_indices):
    return [
        contact
        for contact in contacts
        if contact[CONTACT_TUPLE_LINK_A_INDEX] in landing_arm_link_indices
    ]


def configure_physics():
    p.setRealTimeSimulation(0)
    p.setTimeStep(TIME_STEP)
    p.setPhysicsEngineParameter(
        fixedTimeStep=TIME_STEP,
        numSubSteps=1,
        numSolverIterations=150,
        contactBreakingThreshold=0.001,
    )
    p.setGravity(0, 0, -GRAVITY)


def load_base_segments():
    return [
        p.loadURDF(
            str(URDF_DIR / "I_shape.urdf"),
            [0, 0, 0],
            p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=True,
        )
        for _ in range(8)
    ]


def position_base_segments(segments, base_size, base_height, radian):
    positions = [
        ([base_size / 2, base_size / 2, base_height], [-radian, 0, 0]),
        ([base_size / 2, base_size / 2, base_height], [0, radian, 0]),
        ([base_size / 2, -base_size / 2, base_height], [radian, 0, 0]),
        ([base_size / 2, -base_size / 2, base_height], [0, radian, 0]),
        ([-base_size / 2, base_size / 2, base_height], [-radian, 0, 0]),
        ([-base_size / 2, base_size / 2, base_height], [0, -radian, 0]),
        ([-base_size / 2, -base_size / 2, base_height], [radian, 0, 0]),
        ([-base_size / 2, -base_size / 2, base_height], [0, -radian, 0]),
    ]

    for segment, (position, euler) in zip(segments, positions):
        p.resetBasePositionAndOrientation(
            segment, position, p.getQuaternionFromEuler(euler)
        )


def reset_drone(drone, x, y, drop_height):
    p.resetBasePositionAndOrientation(
        drone,
        [x, y, drop_height],
        p.getQuaternionFromEuler([0, 0, 0]),
    )
    p.resetBaseVelocity(drone, [0, 0, 0], [0, 0, 0])


def get_angular_velocity_limit(drone_arm_length):
    return LINEAR_VELOCITY_LIMIT / (drone_arm_length / 2)


def run_landing_attempt(
    drone,
    base_segment_ids,
    ground_plane,
    landing_arm_link_indices,
    propeller_link_indices,
    x,
    y,
    drop_height,
    drone_arm_length,
):
    l_count = 0
    stable_steps = 0
    has_descended = False
    body_i_shape_collision_detected = False
    propeller_i_shape_collision_detected = False
    settle_steps = max(1, int(round(SETTLE_DURATION / TIME_STEP)))
    max_attempt_steps = max(1, int(round(MAX_LANDING_ATTEMPT_TIME / TIME_STEP)))
    angular_velocity_limit = get_angular_velocity_limit(drone_arm_length)

    while True:
        step_simulation()
        l_count += 1

        pos, orientation = p.getBasePositionAndOrientation(drone)
        linear_vel, angular_vel = p.getBaseVelocity(drone)
        current_landing_contacts = get_landing_base_contacts(drone, base_segment_ids)
        body_i_shape_collision_detected = (
            body_i_shape_collision_detected
            or has_drone_body_contact(current_landing_contacts)
        )
        propeller_i_shape_collision_detected = (
            propeller_i_shape_collision_detected
            or has_drone_propeller_contact(
                current_landing_contacts,
                propeller_link_indices,
            )
        )

        if pos[2] < drop_height - DESCENT_EPSILON:
            has_descended = True

        is_stable = (
            max(abs(v) for v in linear_vel) < LINEAR_VELOCITY_LIMIT
            and max(abs(v) for v in angular_vel) < angular_velocity_limit
        )

        if has_descended and is_stable:
            stable_steps += 1
        else:
            stable_steps = 0

        settled = stable_steps >= settle_steps
        timed_out = l_count >= max_attempt_steps

        if settled or timed_out:
            center_distance = np.hypot(pos[0], pos[1])
            tilt_angle = get_tilt_angle(orientation)
            # Final Z is validated by actual support contacts because platform
            # height changes with base size and angle.
            landing_arm_contacts = get_landing_arm_contacts(
                current_landing_contacts,
                landing_arm_link_indices,
            )
            ground_contacts = get_active_contacts(drone, ground_plane)

            is_successful = (
                settled
                and center_distance < SUCCESS_POSITION_LIMIT
                and tilt_angle < MAX_TILT_ANGLE
                and len(landing_arm_contacts) >= MIN_LANDING_CONTACTS
                and not ground_contacts
                and not propeller_i_shape_collision_detected
            )
            if propeller_i_shape_collision_detected:
                result = UNSUCCESSFUL_RESULT
            elif is_successful and body_i_shape_collision_detected:
                result = COLLISION_RESULT
            else:
                result = SUCCESS_RESULT if is_successful else UNSUCCESSFUL_RESULT
            landing_time = min(l_count * TIME_STEP, MAX_LANDING_ATTEMPT_TIME)

            return landing_time, result


def get_case_csv_file(base_size, angle, drone_arm_length):
    return CSV_OUTPUT_DIR / f"{get_case_stem(base_size, angle, drone_arm_length)}.csv"


def get_axis_positions(base_size):
    start = -(base_size / 2) - LANDING_GRID_RESOLUTION / 2
    end = (base_size / 2) + LANDING_GRID_RESOLUTION / 2
    return np.round(
        np.arange(start, end + FLOAT_TOLERANCE, LANDING_GRID_RESOLUTION), 4
    ).tolist()


def iter_start_positions(base_size):
    axis_positions = get_axis_positions(base_size)
    for y in axis_positions:
        for x in axis_positions:
            yield x, y


def get_attempt_count(base_size):
    return len(get_axis_positions(base_size)) ** 2


def get_case_list():
    return [
        (base_size, angle, drone_arm_length)
        for base_size in BASE_SIZES
        for angle in ANGLES
        for drone_arm_length in DRONE_ARM_LENGTHS
    ]


def get_worker_count(case_count):
    if VISUALIZE:
        return 1
    return min(MAX_PARALLEL_WORKERS, case_count)


def run_case(case):
    get_pybullet()

    base_size, angle, drone_arm_length = case
    connection_mode = p.GUI if VISUALIZE else p.DIRECT
    p.connect(connection_mode)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    configure_physics()

    drone_urdf_path = None
    try:
        ground_plane = p.loadURDF("plane.urdf")
        base_segments = load_base_segments()
        base_segment_ids = set(base_segments)
        drone_urdf_path = get_parameterized_drone_urdf_path(drone_arm_length)
        drone = p.loadURDF(
            str(drone_urdf_path),
            [max(BASE_SIZES), max(BASE_SIZES), 1],
            p.getQuaternionFromEuler([0, 0, 0]),
            flags=getattr(p, "URDF_USE_INERTIA_FROM_FILE", 0),
        )
        landing_arm_link_indices = get_drone_link_indices(drone, DRONE_ARM_LINK_NAMES)
        propeller_link_indices = get_drone_link_indices(
            drone,
            DRONE_PROPELLER_LINK_NAMES,
        )

        radian = np.radians(angle)
        base_height = base_size / (2 * np.tan(radian))
        # min(ANGLES) gives the shallowest platform angle, which produces the
        # tallest edge height.  Using it here guarantees the drone clears every
        # platform configuration in the sweep.
        drop_height = DROP_CLEARANCE + base_size / (2 * np.tan(np.radians(min(ANGLES))))

        position_base_segments(base_segments, base_size, base_height, radian)

        CSV_OUTPUT_DIR.mkdir(exist_ok=True)
        file_name = get_case_csv_file(base_size, angle, drone_arm_length)

        with open(file_name, "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["StartX", "StartY", "Time (s)", "Result"])

            for x, y in iter_start_positions(base_size):
                reset_drone(drone, x, y, drop_height)
                landing_time, result = run_landing_attempt(
                    drone,
                    base_segment_ids,
                    ground_plane,
                    landing_arm_link_indices,
                    propeller_link_indices,
                    x,
                    y,
                    drop_height,
                    drone_arm_length,
                )

                writer.writerow(
                    [round(x, 4), round(y, 4), round(landing_time, 4), result]
                )

        process_csv(file_name)
        summary = get_success_summary(file_name, base_size, angle, drone_arm_length)
        create_plot(file_name, summary)
        return summary

    finally:
        if p.isConnected():
            p.disconnect()
        if drone_urdf_path is not None:
            try:
                drone_urdf_path.unlink(missing_ok=True)
            except OSError:
                pass


def initialize_worker():
    signal.signal(signal.SIGINT, signal.SIG_IGN)


def stop_executor(executor):
    terminate_workers = getattr(executor, "terminate_workers", None)
    if terminate_workers is not None:
        terminate_workers()
        return

    processes = getattr(executor, "_processes", None) or {}
    for process in processes.values():
        if process.is_alive():
            process.terminate()
    executor.shutdown(wait=False, cancel_futures=True)


def run_cases(cases, worker_count):
    if worker_count == 1:
        for case in cases:
            yield run_case(case)
        return

    executor = ProcessPoolExecutor(
        max_workers=worker_count,
        initializer=initialize_worker,
    )
    stopped = False

    try:
        future_to_case = {executor.submit(run_case, case): case for case in cases}
        for future in as_completed(future_to_case):
            yield future.result()

    except KeyboardInterrupt:
        stopped = True
        stop_executor(executor)
        raise

    finally:
        if not stopped:
            executor.shutdown()


def get_plot_file(base_size, angle, drone_arm_length):
    return PLOT_OUTPUT_DIR / f"{get_case_stem(base_size, angle, drone_arm_length)}.png"


def get_combined_plot_file(drone_arm_length):
    return (
        PLOT_OUTPUT_DIR
        / f"all_results_combined_leg_{format_parameter(drone_arm_length)}m.png"
    )


def get_font(size):
    """Try to load a good font, fall back to default."""
    from PIL import ImageFont

    font_candidates = [
        "C:/Windows/Fonts/times.ttf",
        "C:/Windows/Fonts/timesbd.ttf",
        "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/calibri.ttf",
    ]
    for font_path in font_candidates:
        try:
            return ImageFont.truetype(font_path, size)
        except (OSError, IOError):
            continue
    return ImageFont.load_default()


def create_combined_plot(drone_arm_length):
    """Combine all individual heatmap plots into a single composite PNG.

    Layout: rows = base sizes (ascending top to bottom),
            columns = platform angles.
    """
    from PIL import Image, ImageDraw

    n_rows = len(BASE_SIZES)
    n_cols = len(ANGLES)

    # Load one image to get individual plot dimensions.
    sample_path = get_plot_file(BASE_SIZES[0], ANGLES[0], drone_arm_length)
    if not sample_path.exists():
        print(
            "Skipping combined plot: no individual plots found for "
            f"leg {format_mm(drone_arm_length)} mm."
        )
        return

    with Image.open(sample_path) as sample:
        img_w, img_h = sample.size

    # Scale down each individual plot for the composite.
    scale = 0.35
    thumb_w = int(img_w * scale)
    thumb_h = int(img_h * scale)

    # Layout spacing.
    left_margin = 160
    top_margin = 100
    cell_pad_x = 8
    cell_pad_y = 8
    right_margin = 40
    bottom_margin = 60

    canvas_w = left_margin + n_cols * (thumb_w + cell_pad_x) - cell_pad_x + right_margin
    canvas_h = top_margin + n_rows * (thumb_h + cell_pad_y) - cell_pad_y + bottom_margin

    canvas = Image.new("RGB", (canvas_w, canvas_h), (255, 255, 255))
    draw = ImageDraw.Draw(canvas)

    header_font = get_font(36)
    label_font = get_font(28)
    title_font = get_font(48)

    # Main title.
    title_text = (
        f"3DPP Landing Simulation Results \u2014 Leg {format_mm(drone_arm_length)} mm"
    )
    title_bbox = draw.textbbox((0, 0), title_text, font=title_font)
    title_w = title_bbox[2] - title_bbox[0]
    draw.text(
        ((canvas_w - title_w) // 2, 10),
        title_text,
        fill=(30, 30, 30),
        font=title_font,
    )

    # Column headers (angles).
    for col_idx, angle in enumerate(ANGLES):
        col_label = f"{angle}\u00b0"
        col_x = left_margin + col_idx * (thumb_w + cell_pad_x) + thumb_w // 2
        col_bbox = draw.textbbox((0, 0), col_label, font=header_font)
        col_label_w = col_bbox[2] - col_bbox[0]
        draw.text(
            (col_x - col_label_w // 2, top_margin - 44),
            col_label,
            fill=(50, 50, 50),
            font=header_font,
        )

    # Place each plot.
    missing = []
    for row_idx, base_size in enumerate(BASE_SIZES):
        base_mm = base_size * 1000
        row_label = f"{base_mm:g} mm"
        row_y = top_margin + row_idx * (thumb_h + cell_pad_y) + thumb_h // 2
        row_bbox = draw.textbbox((0, 0), row_label, font=label_font)
        row_label_h = row_bbox[3] - row_bbox[1]
        row_label_w = row_bbox[2] - row_bbox[0]
        draw.text(
            (left_margin - row_label_w - 16, row_y - row_label_h // 2),
            row_label,
            fill=(50, 50, 50),
            font=label_font,
        )

        for col_idx, angle in enumerate(ANGLES):
            plot_path = get_plot_file(base_size, angle, drone_arm_length)
            if not plot_path.exists():
                missing.append(plot_path.name)
                continue

            with Image.open(plot_path) as img:
                img_resized = img.resize((thumb_w, thumb_h), Image.LANCZOS)

            paste_x = left_margin + col_idx * (thumb_w + cell_pad_x)
            paste_y = top_margin + row_idx * (thumb_h + cell_pad_y)
            canvas.paste(img_resized, (paste_x, paste_y))

    if missing:
        print(f"Warning: {len(missing)} plot(s) not found.")

    combined_plot_file = get_combined_plot_file(drone_arm_length)
    canvas.save(combined_plot_file, dpi=(300, 300))
    print(f"Combined plot saved to: {combined_plot_file}")


def main():
    cases = get_case_list()
    worker_count = get_worker_count(len(cases))
    total_attempts = sum(get_attempt_count(base_size) for base_size, _, _ in cases)

    print(f"Total simulations: {len(cases)} parameter cases")
    print(f"Total landing attempts: {total_attempts}")
    print(
        f"Parallel workers: {worker_count} "
        f"(MAX_PARALLEL_WORKERS={MAX_PARALLEL_WORKERS})"
    )
    initialize_success_rate_csv()

    progress = ProgressBar(len(cases), "Cases")
    completed = 0

    try:
        progress.update(completed)
        for summary in run_cases(cases, worker_count):
            append_success_summary(summary)
            completed += 1
            progress.update(completed)

    except KeyboardInterrupt:
        progress.close()
        print("Simulation stopped by user.")
        return 130

    progress.close()
    print("Simulation complete.")
    for drone_arm_length in DRONE_ARM_LENGTHS:
        create_combined_plot(drone_arm_length)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
