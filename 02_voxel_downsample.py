from pathlib import Path
import time

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colormaps


FOLDER = Path(__file__).resolve().parent / "hasil_mapping"
INPUT = FOLDER / "point_cloud_3d_selesai.npz"
OUTPUT = FOLDER / "point_cloud_voxel_5cm.npz"

VOXEL_SIZE = 0.05
CHUNK_SIZE = 200_000
TARGET_COUNTS = (12_000, 25_000, 50_000)


def sample_points(points, limit, seed=19):
    if len(points) <= limit:
        return points.copy()
    rng = np.random.default_rng(seed)
    indices = rng.choice(len(points), size=limit, replace=False)
    return points[indices]


def compute_height_colors(points, z_min, color_max):
    z = points[:, 2].astype(np.float64)
    z_range = color_max - z_min
    if z_range > 0:
        normalized = np.clip((z - z_min) / z_range, 0.0, 1.0)
    else:
        normalized = np.zeros(len(points), dtype=np.float64)
    rgb = np.rint(
        colormaps["viridis"](normalized)[:, :3] * 255
    ).astype(np.uint8)
    return rgb, normalized.astype(np.float32)


def main():
    start_time = time.perf_counter()

    if not INPUT.exists():
        raise FileNotFoundError(
            f"File tidak ditemukan: {INPUT}\n"
            "Jika hasilmu parsial, ubah INPUT menjadi "
            "'point_cloud_3d_parsial.npz'."
        )

    print(f"Membuka {INPUT.name}...", flush=True)

    with np.load(INPUT, allow_pickle=False) as data:
        points = data["points"]
        metadata = {
            key: data[key]
            for key in ("trajectory", "waypoints", "completed")
            if key in data.files
        }

    if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
        raise ValueError("Data points harus berukuran N x 3 dan tidak kosong.")

    original_count = len(points)
    print(f"Titik awal       : {original_count:,}")
    print(f"RAM array points : {points.nbytes / 1024**2:.1f} MiB")
    print(f"Ukuran voxel     : {VOXEL_SIZE * 100:.1f} cm", flush=True)

    voxels = {}
    valid_count = 0

    for begin in range(0, original_count, CHUNK_SIZE):
        end = min(begin + CHUNK_SIZE, original_count)

        block = points[begin:end]
        block = block[np.isfinite(block).all(axis=1)]
        valid_count += len(block)

        if len(block):
            keys = np.floor(
                block.astype(np.float64) / VOXEL_SIZE
            ).astype(np.int64)

            unique_keys, inverse, counts = np.unique(
                keys,
                axis=0,
                return_inverse=True,
                return_counts=True,
            )
            inverse = inverse.reshape(-1)

            sums = np.column_stack([
                np.bincount(
                    inverse,
                    weights=block[:, axis],
                    minlength=len(unique_keys),
                )
                for axis in range(3)
            ])

            for key, xyz_sum, count in zip(unique_keys, sums, counts):
                cell = tuple(int(v) for v in key)

                if cell in voxels:
                    entry = voxels[cell]
                    entry[0] += float(xyz_sum[0])
                    entry[1] += float(xyz_sum[1])
                    entry[2] += float(xyz_sum[2])
                    entry[3] += int(count)
                else:
                    voxels[cell] = [
                        float(xyz_sum[0]),
                        float(xyz_sum[1]),
                        float(xyz_sum[2]),
                        int(count),
                    ]

        print(
            f"Progres {100 * end / original_count:5.1f}% | "
            f"voxel terisi: {len(voxels):,}",
            flush=True,
        )

    if not voxels:
        raise ValueError("Tidak ada titik valid untuk diproses.")

    del points

    voxel_indices = np.array(list(voxels.keys()), dtype=np.int64)
    statistics = np.array(list(voxels.values()), dtype=np.float64)
    del voxels

    counts = statistics[:, 3].astype(np.int64)
    reduced = (statistics[:, :3] / counts[:, None]).astype(np.float32)
    del statistics

    if int(counts.sum()) != valid_count:
        raise RuntimeError("Jumlah titik dalam voxel tidak konsisten.")

    if not np.isfinite(reduced).all():
        raise RuntimeError("Hasil voxel mengandung angka tidak valid.")

    z_min = float(reduced[:, 2].min())
    z_max = float(reduced[:, 2].max())
    color_max = z_max if z_max > z_min else z_min + 0.01

    rgb_full, norm_full = compute_height_colors(reduced, z_min, color_max)

    sampled_sets = {}
    for target_n in TARGET_COUNTS:
        subset = sample_points(reduced, target_n, seed=19)
        label_rb = f"{target_n // 1000}rb"
        sampled_sets[label_rb] = subset

        rgb_sub, norm_sub = compute_height_colors(subset, z_min, color_max)
        subset_path = FOLDER / f"point_cloud_{label_rb}.npz"
        np.savez_compressed(
            subset_path,
            points=subset,
            colors_rgb=rgb_sub,
            height_normalized=norm_sub,
            height_color_limits=np.array([z_min, color_max]),
            colormap_name=np.array("viridis"),
            voxel_size=np.array(VOXEL_SIZE),
            source_point_count=np.array(original_count),
            voxel_point_count=np.array(len(reduced)),
            target_point_count=np.array(target_n),
            **metadata,
        )

    np.savez_compressed(
        OUTPUT,
        points=reduced,
        points_12rb=sampled_sets["12rb"],
        points_25rb=sampled_sets["25rb"],
        points_50rb=sampled_sets["50rb"],
        colors_rgb=rgb_full,
        height_normalized=norm_full,
        height_color_limits=np.array([z_min, color_max]),
        colormap_name=np.array("viridis"),
        voxel_indices=voxel_indices,
        points_per_voxel=counts,
        voxel_size=np.array(VOXEL_SIZE),
        grid_origin=np.zeros(3),
        source_point_count=np.array(original_count),
        valid_point_count=np.array(valid_count),
        **metadata,
    )

    reduction = 100 * (1 - len(reduced) / valid_count)

    print("\nSELESAI")
    print(f"Titik awal          : {original_count:,}")
    print(f"Titik setelah voxel : {len(reduced):,} (-{reduction:.2f}%)")
    for label_rb, subset in sampled_sets.items():
        print(f"Sampel {label_rb:4s}         : {len(subset):,} titik -> point_cloud_{label_rb}.npz")
    print(f"Waktu proses        : {time.perf_counter() - start_time:.1f} s")
    print(f"Tersimpan utama     : {OUTPUT}", flush=True)

    lower = reduced.min(axis=0)
    upper = reduced.max(axis=0)
    span = np.maximum(upper - lower, VOXEL_SIZE)
    padding = np.maximum(span * 0.03, VOXEL_SIZE)

    fig = plt.figure(figsize=(16, 5.5))
    for idx, (label_rb, subset) in enumerate(sampled_sets.items(), start=1):
        ax = fig.add_subplot(1, 3, idx, projection="3d")
        sc = ax.scatter(
            subset[:, 0],
            subset[:, 1],
            subset[:, 2],
            c=subset[:, 2],
            cmap="viridis",
            vmin=z_min,
            vmax=color_max,
            s=1,
            depthshade=False,
        )
        ax.set_xlim(lower[0] - padding[0], upper[0] + padding[0])
        ax.set_ylim(lower[1] - padding[1], upper[1] + padding[1])
        ax.set_zlim(lower[2] - padding[2], upper[2] + padding[2])
        ax.set_box_aspect(span + 2 * padding)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_title(f"Downsample {len(subset):,} titik ({label_rb})")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()