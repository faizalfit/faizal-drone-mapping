from pathlib import Path
import math

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colormaps
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


PROPERTY_NAME = "signal.faizal_lidar_packet_v2"

FLIGHT_Z = 5.2
MAX_SPEED = 0.5
TURN_TIME = 5.0
MAX_TRACKING_ERROR = 1.5

XY_LIMIT = 5.2
Z_MIN = -0.10
Z_MAX = 3.50
PREVIEW_LIMIT = 120_000

OUTPUT = Path(__file__).resolve().parent / "hasil_mapping"


def smoothstep(u):
    u = np.clip(u, 0.0, 1.0)
    return float(u * u * (3.0 - 2.0 * u))


def main():
    print("Menghubungkan ke CoppeliaSim...", flush=True)
    client = RemoteAPIClient(host="localhost", port=23000)
    sim = client.require("sim")

    if sim.getSimulationState() != sim.simulation_stopped:
        raise RuntimeError("Tekan Stop sebelum menjalankan program.")

    base = sim.getObject("/Quadcopter/base")
    target = sim.getObject("/Quadcopter/target")

    initial = np.array(
        sim.getObjectPosition(target, sim.handle_world), dtype=float
    )
    target_orientation = list(
        sim.getObjectOrientation(target, sim.handle_world)
    )
    commanded = initial.copy()

    batches = []
    trajectory = []
    last_sequence = -1
    last_progress = -1
    point_total = 0
    started = False
    completed = False

    waypoints = [
        [-4.0, -4.0, FLIGHT_Z],
        [-4.0,  4.0, FLIGHT_Z],
        [ 0.0,  4.0, FLIGHT_Z],
        [ 0.0, -4.0, FLIGHT_Z],
        [ 4.0, -4.0, FLIGHT_Z],
        [ 4.0,  4.0, FLIGHT_Z],
    ]

    def step_and_record():
        nonlocal last_sequence, last_progress, point_total

        before = sim.getSimulationTime()
        sim.step()
        state = sim.getSimulationState()

        if state == sim.simulation_paused:
            raise RuntimeError(
                "Simulasi ter-pause. Periksa error Lua di CoppeliaSim."
            )
        if state == sim.simulation_stopped:
            raise RuntimeError("Simulasi dihentikan sebelum misi selesai.")

        t = sim.getSimulationTime()
        if t <= before:
            raise RuntimeError("Waktu simulasi tidak maju.")

        actual = np.array(
            sim.getObjectPosition(base, sim.handle_world), dtype=float
        )
        trajectory.append([t, *actual])

        if np.linalg.norm(actual - commanded) > MAX_TRACKING_ERROR:
            raise RuntimeError(
                "Drone menyimpang lebih dari 1,5 m dari target. "
                "Misi dihentikan; periksa kontrol dan tabrakan."
            )

        raw = sim.getBufferProperty(sim.handle_scene, PROPERTY_NAME, {})

        if raw is None or len(raw) < 12 or len(raw) % 4:
            raise RuntimeError("Paket LiDAR kosong atau tidak valid.")

        values = np.frombuffer(raw, dtype="<f4")
        if not np.isfinite(values[:3]).all():
            raise RuntimeError("Header LiDAR tidak valid.")

        sequence = int(round(float(values[0])))
        count = int(round(float(values[2])))

        if count < 0 or values.size != 3 + 3 * count:
            raise RuntimeError("Jumlah titik tidak cocok dengan ukuran paket.")

        if sequence != last_sequence:
            last_sequence = sequence

            if count:
                points = values[3:].reshape(count, 3)
                valid = np.isfinite(points).all(axis=1)
                valid &= np.abs(points[:, 0]) <= XY_LIMIT
                valid &= np.abs(points[:, 1]) <= XY_LIMIT
                valid &= points[:, 2] >= Z_MIN
                valid &= points[:, 2] <= Z_MAX

                selected = points[valid]
                if len(selected):
                    batches.append(selected.copy())
                    point_total += len(selected)

        progress = int(t // 2)
        if progress != last_progress:
            last_progress = progress
            print(
                f"t={t:6.1f} s | "
                f"XYZ=({actual[0]:.2f}, {actual[1]:.2f}, "
                f"{actual[2]:.2f}) | titik={point_total:,}",
                flush=True,
            )

    def hold(seconds):
        end = sim.getSimulationTime() + seconds
        while sim.getSimulationTime() < end:
            step_and_record()

    def turn_to(yaw):
        start_yaw = target_orientation[2]
        delta = (yaw - start_yaw + math.pi) % (2 * math.pi) - math.pi

        if abs(delta) < 0.01:
            return

        begin = sim.getSimulationTime()

        while True:
            elapsed = sim.getSimulationTime() - begin
            u = min(elapsed / TURN_TIME, 1.0)

            target_orientation[2] = start_yaw + delta * smoothstep(u)
            sim.setObjectOrientation(
                target, sim.handle_world, target_orientation
            )
            step_and_record()

            if u >= 1.0:
                break

        hold(1.0)

    def move_to(destination):
        nonlocal commanded

        start = commanded.copy()
        destination = np.array(destination, dtype=float)
        distance = float(np.linalg.norm(destination - start))

        if distance < 0.001:
            return

        duration = max(2.0, 1.5 * distance / MAX_SPEED)
        begin = sim.getSimulationTime()

        while True:
            elapsed = sim.getSimulationTime() - begin
            u = min(elapsed / duration, 1.0)
            commanded = start + smoothstep(u) * (destination - start)

            sim.setObjectPosition(
                target, sim.handle_world, commanded.tolist()
            )
            step_and_record()

            if u >= 1.0:
                break

        hold(2.0)

    sim.setStepping(True)

    try:
        sim.startSimulation()
        started = True

        print("Hover awal...", flush=True)
        hold(2.0)

        print("Mengatur ketinggian terbang...", flush=True)
        move_to([initial[0], initial[1], FLIGHT_Z])

        for index, destination in enumerate(waypoints, start=1):
            dx = destination[0] - commanded[0]
            dy = destination[1] - commanded[1]

            if math.hypot(dx, dy) > 0.001:
                turn_to(math.atan2(dy, dx))

            print(
                f"Menuju waypoint {index}/{len(waypoints)}: {destination}",
                flush=True,
            )
            move_to(destination)

        hold(2.0)
        completed = True
        print("Lintasan selesai.", flush=True)

    except KeyboardInterrupt:
        print("\nDihentikan pengguna; menyimpan hasil sementara.")

    except Exception as exc:
        print(f"\nMisi dihentikan: {exc}", flush=True)

    finally:
        try:
            if started:
                sim.stopSimulation()
        finally:
            sim.setStepping(False)

    if not batches:
        print("Belum ada titik peta yang dapat disimpan.")
        return

    cloud = np.vstack(batches)
    track = np.asarray(trajectory, dtype=np.float64)

    z = cloud[:, 2].astype(np.float64)
    z_min = float(z.min())
    z_max = float(z.max())
    z_range = z_max - z_min

    if z_range > 0:
        normalized = (z - z_min) / z_range
        color_max = z_max
    else:
        normalized = np.zeros(len(cloud), dtype=np.float64)
        color_max = z_min + 0.01

    rgb = np.rint(
        colormaps["viridis"](normalized)[:, :3] * 255
    ).astype(np.uint8)

    OUTPUT.mkdir(parents=True, exist_ok=True)
    suffix = "selesai" if completed else "parsial"

    cloud_path = OUTPUT / f"point_cloud_3d_{suffix}.npz"
    track_path = OUTPUT / f"trajectory_3d_{suffix}.csv"

    np.savez_compressed(
        cloud_path,
        points=cloud,
        trajectory=track,
        waypoints=np.asarray(waypoints),
        completed=np.array(completed),
        colors_rgb=rgb,
        height_normalized=normalized.astype(np.float32),
        height_color_limits=np.array([z_min, color_max]),
        colormap_name=np.array("viridis"),
    )
    np.savetxt(
        track_path,
        track,
        delimiter=",",
        header="time_s,x_m,y_m,z_m",
        comments="",
    )

    print(f"\nTersimpan: {cloud_path}")
    print(f"Tersimpan: {track_path}")
    print(f"Total sampel peta: {len(cloud):,}")

    if len(cloud) > PREVIEW_LIMIT:
        rng = np.random.default_rng(19)
        preview = cloud[
            rng.choice(len(cloud), size=PREVIEW_LIMIT, replace=False)
        ]
    else:
        preview = cloud

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    sc = ax.scatter(
        preview[:, 0],
        preview[:, 1],
        preview[:, 2],
        c=preview[:, 2],
        cmap="viridis",
        vmin=z_min,
        vmax=color_max,
        s=0.5,
        alpha=0.8,
        label="Point cloud",
    )
    ax.plot(
        track[:, 1],
        track[:, 2],
        track[:, 3],
        color="tab:red",
        linewidth=1.5,
        label="Trajectory aktual",
    )

    fig.colorbar(sc, ax=ax, shrink=0.65, pad=0.08, label="Z dunia (m)")

    ax.set_xlim(-XY_LIMIT, XY_LIMIT)
    ax.set_ylim(-XY_LIMIT, XY_LIMIT)
    ax.set_zlim(Z_MIN, max(FLIGHT_Z + 0.5, track[:, 3].max()))
    ax.set_box_aspect((2 * XY_LIMIT, 2 * XY_LIMIT, FLIGHT_Z + 0.6))

    ax.set_xlabel("X dunia (m)")
    ax.set_ylabel("Y dunia (m)")
    ax.set_zlabel("Z dunia (m)")
    ax.set_title(f"Mapping 3D (Height Color Coded) — hasil {suffix}")
    ax.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()