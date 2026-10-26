"""6축 팔 작업 영역(도달 범위) 분석 + 전정→제초 궤적 애니메이션.

구동체 좌표계 (X 전방, Z 상방, 지면 Z=0). J2 축 = (300, 0, 958).
피치 평면(XZ) 순기구학: J2, J3, J5 (J1 요는 평면을 회전, J4/J6 롤은 위치 무관).
툴 팁 = ISO 플랜지 + TOOL_LEN (전동 가위 날 끝 기준 150).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
from scipy.optimize import least_squares

J2_ORIGIN = (300.0, 0.0, 958.0)
L_UPPER = 500.0          # J2 → J3
L_FORE = 422.0           # J3 → J5
L_WRIST = 128.0          # J5 → 플랜지
TOOL_LEN = 150.0         # 플랜지 → 툴 팁
LIMITS = {"J2": (-20.0, 90.0), "J3": (-120.0, 120.0), "J5": (-110.0, 110.0)}
GROUND_CLEAR = 30.0      # 팁 최저 허용 높이 (지면 제초 시)


def fk(j2: float, j3: float, j5: float) -> Dict[str, Tuple[float, float]]:
    """XZ 평면 순기구학 (deg). 각 관절 '전방 상승 +'. 반환: 각 점의 (x, z)."""
    a2 = math.radians(j2)
    a3 = a2 + math.radians(j3)
    a5 = a3 + math.radians(j5)
    x0, _, z0 = J2_ORIGIN
    p3 = (x0 + L_UPPER * math.cos(a2), z0 + L_UPPER * math.sin(a2))
    p5 = (p3[0] + L_FORE * math.cos(a3), p3[1] + L_FORE * math.sin(a3))
    pf = (p5[0] + L_WRIST * math.cos(a5), p5[1] + L_WRIST * math.sin(a5))
    pt = (pf[0] + TOOL_LEN * math.cos(a5), pf[1] + TOOL_LEN * math.sin(a5))
    return {"J3": p3, "J5": p5, "flange": pf, "tip": pt, "tool_angle": (math.degrees(a5), 0.0)}


def workspace(n: int = 60) -> np.ndarray:
    """관절 한계 내 툴 팁 도달점 샘플 (x, z) — 상완·전완이 지면 아래로 가는 자세 제외."""
    pts = []
    for j2 in np.linspace(*LIMITS["J2"], n):
        for j3 in np.linspace(*LIMITS["J3"], n):
            for j5 in np.linspace(*LIMITS["J5"], n // 2):
                f = fk(j2, j3, j5)
                if min(f["J3"][1], f["J5"][1], f["flange"][1], f["tip"][1]) < GROUND_CLEAR:
                    continue
                pts.append(f["tip"])
    return np.array(pts)


def ik(target: Tuple[float, float], tool_deg: float, q0=(30.0, -60.0, 0.0)) -> Tuple[float, float, float]:
    """XZ 평면 IK: 툴 팁 위치 + 툴 피치각 → (J2, J3, J5). 관절 한계 내 최소자승."""
    lo = [LIMITS[k][0] for k in ("J2", "J3", "J5")]
    hi = [LIMITS[k][1] for k in ("J2", "J3", "J5")]

    def res(q):
        f = fk(*q)
        return [f["tip"][0] - target[0], f["tip"][1] - target[1],
                2.0 * (f["tool_angle"][0] - tool_deg)]

    sol = least_squares(res, np.clip(q0, lo, hi), bounds=(lo, hi))
    err = math.hypot(*sol.fun[:2])
    if err > 5.0:
        raise ValueError(f"IK 도달 불가: {target}, 잔차 {err:.1f} mm")
    return tuple(float(v) for v in sol.x)


def reach_summary() -> Dict[str, float]:
    pts = workspace(50)
    xs, zs = pts[:, 0], pts[:, 1]
    out = {"tip_z_max": zs.max(), "tip_x_max": xs.max(), "tip_x_min": xs.min(),
           "tip_z_min": zs.min()}
    out["x_at_z_max"] = xs[zs.argmax()]
    # 특정 높이에서 전방 도달 거리 (0.5 m 간격)
    for zt in (0, 500, 1000, 1500, 2000):
        m = np.abs(zs - zt) < 40
        out[f"x_reach_at_z{zt}"] = xs[m].max() if m.any() else float("nan")
    # 트랙 중심선 기준 측방 도달 (J1 90°) = 전방 도달 − J2 X 오프셋
    out["lateral_reach_at_z1000"] = out["x_reach_at_z1000"] - J2_ORIGIN[0]
    return out


def plot_workspace(out: Path) -> Dict[str, float]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Ellipse, Rectangle

    pts = workspace(50)
    fig, ax = plt.subplots(figsize=(12, 8), dpi=110)
    ax.scatter(pts[:, 0] / 1000, pts[:, 1] / 1000, s=1, c="#2a7fd4", alpha=0.25, label="tool-tip reachable set")
    # 구동체 외형 (프레임 280~720, 휠)
    ax.add_patch(Rectangle((-0.5, 0.28), 1.0, 0.44, fill=False, lw=1.5, color="k"))
    for wx in (-0.45, 0.0, 0.45):
        ax.add_patch(plt.Circle((wx, 0.125), 0.125, fill=False, color="k"))
    ax.plot(J2_ORIGIN[0] / 1000, J2_ORIGIN[2] / 1000, "r+", ms=14, mew=2, label="J2 axis")
    # 참고 나무: 밀식 사과 (주간 높이 ~3.0 m, 수관 폭 1.2 m), 나무 중심 X = 1.3 m
    ax.add_patch(Rectangle((1.25, 0), 0.1, 0.6, color="#8b5a2b"))
    ax.add_patch(Ellipse((1.3, 1.9), 1.2, 2.6, color="#6abf69", alpha=0.35))
    ax.axhline(0, color="k", lw=1)
    s = reach_summary()
    ax.annotate(f"max tip height {s['tip_z_max']/1000:.2f} m @ x={s['x_at_z_max']/1000:.2f} m",
                (s["x_at_z_max"] / 1000, s["tip_z_max"] / 1000), xytext=(0.0, 2.3),
                arrowprops=dict(arrowstyle="->"), fontsize=10)
    ax.set_aspect("equal"); ax.set_xlim(-0.8, 2.2); ax.set_ylim(-0.1, 2.5)
    ax.set_xlabel("X forward (m)"); ax.set_ylabel("Z height (m)")
    ax.set_title("Tool-tip workspace (side view), J2 -20..90 / J3 ±140 / J5 ±110, tool 150 mm")
    ax.grid(alpha=0.3); ax.legend(loc="upper left")
    fig.tight_layout(); fig.savefig(out); plt.close(fig)
    return s


# ---------------------------------------------------------------------------
# 궤적: 운반 자세 → 상단 전정 → 중단 전정 → 지면 제초 → 운반
# ---------------------------------------------------------------------------
def trajectory_keyframes() -> List[Tuple[str, Dict[str, float]]]:
    kf = [("transport", {"J1": 0, "J2": 80, "J3": -120, "J4": 0, "J5": 60, "J6": 0})]
    targets = [
        ("prune_high", (900.0, 1950.0), 45.0, 20.0),    # 수관 상부 가지, 툴 45° 상향, J1 20°
        ("prune_mid", (1250.0, 1300.0), 0.0, -15.0),
        ("weed_ground", (900.0, 100.0), -80.0, 0.0),   # 밑동 둘레 제초, 툴 하향
    ]
    q0 = (30.0, -60.0, 0.0)
    for name, tgt, tool, j1 in targets:
        j2, j3, j5 = ik(tgt, tool, q0)
        q0 = (j2, j3, j5)
        kf.append((name, {"J1": j1, "J2": j2, "J3": j3, "J4": 0, "J5": j5, "J6": 0}))
    kf.append(("transport", dict(kf[0][1])))
    return kf


def interpolate(kf, steps_per_seg: int = 8) -> List[Dict[str, float]]:
    frames = []
    for (_, a), (_, b) in zip(kf, kf[1:]):
        for i in range(steps_per_seg):
            t = i / steps_per_seg
            t = 0.5 - 0.5 * math.cos(math.pi * t)       # ease-in-out
            frames.append({k: a[k] + (b[k] - a[k]) * t for k in a})
    frames.append(dict(kf[-1][1]))
    return frames


def animate(out_gif: Path, out_png_dir: Path, steps: int = 8) -> List[Tuple[str, Dict[str, float]]]:
    import imageio.v2 as imageio
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from orchard_rover.arm_pose_render import render

    kf = trajectory_keyframes()
    frames = interpolate(kf, steps)
    tip_path = []

    def overlay(ax3):
        # 지면, 구동체 프레임 박스, 나무
        X, Y = np.meshgrid([-700, 2200], [-900, 900])
        ax3.plot_surface(X, Y, np.zeros_like(X), color="#d9c9a3", alpha=0.35)
        bx = [-500, 500, 500, -500, -500]; by = [-400, -400, 400, 400, -400]
        for z in (280, 720):
            ax3.plot(bx, by, [z] * 5, color="k", lw=1)
        for x, y in zip(bx[:4], by[:4]):
            ax3.plot([x, x], [y, y], [280, 720], color="k", lw=1)
        # 나무: 줄기 + 수관(점군)
        ax3.plot([1300, 1300], [0, 0], [0, 600], color="#8b5a2b", lw=6)
        th = np.linspace(0, 2 * math.pi, 40); ph = np.linspace(0, math.pi, 20)
        T, P = np.meshgrid(th, ph)
        ax3.plot_surface(1300 + 600 * np.sin(P) * np.cos(T), 600 * np.sin(P) * np.sin(T),
                         1900 + 1300 * np.cos(P), color="#6abf69", alpha=0.15, linewidth=0)
        if tip_path:
            tx, ty, tz = zip(*tip_path)
            ax3.plot(tx, ty, tz, color="#9b4fd4", lw=2)

    lims = ((-700, 2200), (-1450, 1450), (-100, 2800))
    out_png_dir.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(12, 8), dpi=90)
    images = []
    for i, q in enumerate(frames):
        f = fk(q["J2"], q["J3"], q["J5"])
        yaw = math.radians(q["J1"])
        tx, tz = f["tip"]
        tip_path.append(((tx - J2_ORIGIN[0]) * math.cos(yaw) + J2_ORIGIN[0],
                         (tx - J2_ORIGIN[0]) * math.sin(yaw), tz))
        png = out_png_dir / f"frame_{i:03d}.png"
        render(q, png, view=(18, -55), title=f"frame {i}", offset=J2_ORIGIN, overlay=overlay,
               lims=lims, fig=fig)
        images.append(imageio.imread(png))
    plt.close(fig)
    imageio.mimsave(out_gif, images, duration=0.12, loop=0)
    return kf


if __name__ == "__main__":
    out = Path(__file__).resolve().parents[1] / "output" / "arm"
    out.mkdir(parents=True, exist_ok=True)
    s = plot_workspace(out / "workspace_xz.png")
    for k, v in s.items():
        print(f"{k:24s}: {v:8.0f} mm")
    print()
    for name, q in trajectory_keyframes():
        f = fk(q["J2"], q["J3"], q["J5"])
        print(f"{name:12s} J1={q['J1']:+5.0f} J2={q['J2']:+6.1f} J3={q['J3']:+6.1f} J5={q['J5']:+6.1f}"
              f"  tip=({f['tip'][0]:.0f}, {f['tip'][1]:.0f})")
