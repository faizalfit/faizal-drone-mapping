from pathlib import Path
import argparse
import json
from collections import deque
import itertools
import webbrowser

import numpy as np
from matplotlib import colormaps


def cluster(points, cell=0.10, floor=0.15, boundary=4.85, min_points=60):
    labels = np.full(len(points), -3, dtype=np.int32)
    valid = np.isfinite(points).all(axis=1)
    ground = valid & (points[:, 2] <= floor)
    labels[ground] = -1
    border = valid & ~ground & (
        (np.abs(points[:, 0]) >= boundary) | (np.abs(points[:, 1]) >= boundary)
    )
    labels[border] = -2
    indices = np.flatnonzero(valid & ~ground & ~border)
    if not len(indices):
        return labels, []

    keys, inverse = np.unique(
        np.floor(points[indices].astype(float) / cell).astype(np.int64),
        axis=0,
        return_inverse=True,
    )
    inverse = inverse.reshape(-1)
    lookup = {tuple(k): i for i, k in enumerate(keys)}
    visited = np.full(len(keys), -1, dtype=np.int32)
    offsets = list(itertools.product((-1, 0, 1), repeat=3))
    offsets.remove((0, 0, 0))

    components = 0
    for seed in range(len(keys)):
        if visited[seed] >= 0:
            continue
        visited[seed] = components
        queue = deque([seed])
        while queue:
            i = queue.popleft()
            x, y, z = keys[i]
            for dx, dy, dz in offsets:
                j = lookup.get((x + dx, y + dy, z + dz))
                if j is not None and visited[j] < 0:
                    visited[j] = components
                    queue.append(j)
        components += 1

    membership = visited[inverse]
    sizes = np.bincount(membership, minlength=components)
    accepted = sorted(
        np.flatnonzero(sizes >= min_points),
        key=lambda k: (-sizes[k], int(k)),
    )
    remap = np.full(components, -3, dtype=np.int32)
    for group_id, old in enumerate(accepted):
        remap[old] = group_id
    labels[indices] = remap[membership]

    groups = []
    for i in range(len(accepted)):
        p = points[labels == i]
        lo = p.min(0)
        hi = p.max(0)
        groups.append(
            dict(
                id=i,
                count=len(p),
                lower=lo.tolist(),
                upper=hi.tolist(),
                extent=(hi - lo).tolist(),
                large=bool(max((hi - lo)[:2]) > 4),
            )
        )
    return labels, groups


def sample_indices(n, limit, rng):
    return np.arange(n) if n <= limit else rng.choice(n, limit, replace=False)


def main():
    parser = argparse.ArgumentParser(
        description="Height mapping + geometric groups, one offline HTML."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=Path(__file__).resolve().parent / "hasil_mapping" / "point_cloud_voxel_5cm.npz",
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--cell", type=float, default=0.10)
    parser.add_argument("--floor", type=float, default=0.15)
    parser.add_argument("--boundary", type=float, default=4.85)
    parser.add_argument("--min-points", type=int, default=60)
    parser.add_argument("--no-open", action="store_true")
    args = parser.parse_args()

    if args.cell <= 0 or args.boundary <= 0 or args.min_points < 1:
        parser.error("Parameter tidak valid.")

    out = args.output or args.input.parent
    out.mkdir(parents=True, exist_ok=True)

    with np.load(args.input, allow_pickle=False) as f:
        data = {k: f[k] for k in f.files}

    points = data["points"]
    if (
        points.ndim != 2
        or points.shape[1] != 3
        or not len(points)
        or not np.isfinite(points).all()
    ):
        raise ValueError("points harus N x 3, tidak kosong dan finite.")

    print("Clustering geometris...", flush=True)
    labels, groups = cluster(
        points, args.cell, args.floor, args.boundary, args.min_points
    )

    z = points[:, 2]
    lo = float(z.min())
    hi = float(z.max())
    span = max(hi - lo, 1e-9)
    colors = np.rint(
        colormaps["viridis"]((z - lo) / span)[:, :3] * 255
    ).astype(np.uint8)

    data.update(
        colors_rgb=colors,
        cluster_labels=labels,
        height_color_limits=np.array([lo, hi]),
        colormap_name=np.array("viridis"),
    )
    np.savez_compressed(out / "point_cloud_height_color.npz", **data)

    rng = np.random.default_rng(19)
    idx = sample_indices(len(points), 50000, rng)
    overview = np.column_stack((points[idx], labels[idx])).round(4).tolist()

    for g in groups:
        p = points[labels == g["id"]]
        g["detail"] = p[sample_indices(len(p), 8000, rng)].round(4).tolist()

    track = data.get("trajectory", np.empty((0, 4)))
    track = track[:: max(1, len(track) // 1500), 1:4]
    lut = (
        np.rint(colormaps["viridis"](np.linspace(0, 1, 256))[:, :3] * 255)
        .astype(int)
        .tolist()
    )
    config = dict(
        cell=args.cell,
        floor=args.floor,
        boundary=args.boundary,
        min_points=args.min_points,
    )
    payload = dict(
        points=overview,
        groups=groups,
        track=track.round(4).tolist(),
        lut=lut,
        zmin=lo,
        zmax=hi,
        total=len(points),
        parameters=config,
    )
    summary = dict(
        total=len(points),
        groups=[{k: v for k, v in g.items() if k != "detail"} for g in groups],
        parameters=config,
        ground=int(sum(labels == -1)),
        boundary=int(sum(labels == -2)),
        ungrouped=int(sum(labels == -3)),
    )

    (out / "cluster_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )

    template = Path(__file__).with_name("mapping_viewer_template.html")
    if not template.exists():
        raise FileNotFoundError(
            "Letakkan mapping_viewer_template.html di sebelah script ini."
        )

    html = template.read_text(encoding="utf-8-sig").replace(
        "__DATA__", json.dumps(payload, separators=(",", ":"))
    )
    path = out / "mapping_3d_interaktif.html"
    path.write_text(html, encoding="utf-8")

    print(
        f"Selesai: {len(points):,} titik penuh, {len(groups)} kelompok. "
        "Preview maksimum 50.000 / detail 8.000."
    )
    print(path.resolve())
    if not args.no_open:
        webbrowser.open(path.resolve().as_uri())


if __name__ == "__main__":
    main()