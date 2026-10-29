"""6축 팔 자세 렌더 — 각 관절을 꺾은 자세로 링크별 색상 + 축 표시 (matplotlib, 컬러 래스터).

체인: base(0) ─J2─ upper(1) ─J3─ fore_root(2) ─J4─ fore_tube(3) ─J5─ wrist(4) ─J6─ flange(5)
각 형상은 이름으로 체인 단계에 배정되고, 하류 단계 전체가 해당 관절 축으로 회전한다.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Tuple

import cadquery as cq
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from orchard_rover.arm_assembly import UPPER_LEN, build_arm
from orchard_rover.wrist_joint import WristParams, wrist_axes

# 체인 단계 판정 (이름 접두/키워드)
STAGE_RULES: List[Tuple[int, Tuple[str, ...]]] = [
    (5, ("iso_flange",)),
    (4, ("wrist_block", "j6_motor", "j6_gear", "wrist_camera")),
    (3, ("forearm_tube", "j5_yoke", "j5_motor", "j5_gear", "j5_bearing")),
    # J3: 출력 플랜지·베어링은 상완(1), 모터·링·캠·디스크는 전완판과 함께 전완(2)
    (2, ("J3_motor", "J3_ring", "J3_cam", "J3_disc", "fore_plate_J3", "fore_offset_bracket",
         "j4_housing", "j4_motor", "j4_gear", "j4_bearing")),
    (1, ("J2_output_flange", "J2_output_bearing", "upper_", "gs_anchor_arm", "J3_")),
    (0, ("J2_",)),
]
STAGE_COLOR = {0: "#8a8a8f", 1: "#d94848", 2: "#f0902a", 3: "#2e9e4f", 4: "#2a7fd4", 5: "#9b4fd4"}
STAGE_NAME = {0: "base / J2 fixed", 1: "upper arm  (moved by J2)", 2: "J3 output + J4 housing  (moved by J3)",
              3: "forearm tube + J5 yoke  (moved by J4 roll)", 4: "wrist block + J6  (moved by J5 pitch)",
              5: "ISO 9409 flange  (moved by J6 roll)"}


def stage_of(name: str) -> int:
    for st, keys in STAGE_RULES:
        if any(name.startswith(k) or k in name for k in keys):
            return st
    return 0


_HOME_CACHE: Dict[int, List[cq.Shape]] = {}


def load_home_staged() -> Dict[int, List[cq.Shape]]:
    """홈 자세 형상을 단계별로 한 번만 생성해 캐시 (build_arm 은 수십 초)."""
    if not _HOME_CACHE:
        asm = build_arm()
        for k in range(6):
            _HOME_CACHE[k] = []
        for ch in asm.children:
            shp = ch.obj.val() if isinstance(ch.obj, cq.Workplane) else ch.obj
            if ch.loc is not None:
                shp = shp.moved(ch.loc)
            _HOME_CACHE[stage_of(ch.name)].append(shp)
    return _HOME_CACHE


def place(shape: cq.Shape, stage: int, q: Dict[str, float]) -> cq.Shape:
    """팔 좌표계 홈 자세 형상을 체인 단계 stage 기준으로 관절각 q (deg) 자세로 옮긴다.

    하류 → 상류 순으로 적용 (각 관절 축은 그 시점의 홈 좌표). J1 요는 모든 단계에 적용.
    """
    ax = wrist_axes(WristParams(x_j3=UPPER_LEN, x_flange=UPPER_LEN + 450 + 100))
    x_j3, x_j5, x_fl = UPPER_LEN, ax["J5_pitch_x"], ax["flange_x"]
    joints = [  # (이 단계 이상 회전, origin, axis, 각)
        (5, (x_fl, 0, 0), (1, 0, 0), q.get("J6", 0)),
        (4, (x_j5, 0, 0), (0, 1, 0), -q.get("J5", 0)),
        (3, (x_j3 + 100, 0, 0), (1, 0, 0), q.get("J4", 0)),
        (2, (x_j3, 0, 0), (0, 1, 0), -q.get("J3", 0)),
        (1, (0, 0, 0), (0, 1, 0), -q.get("J2", 0)),
        (0, (0, 0, 0), (0, 0, 1), q.get("J1", 0)),
    ]
    for min_stage, origin, axis, deg in joints:
        if stage >= min_stage and abs(deg) > 1e-9:
            o = cq.Vector(*origin)
            shape = shape.rotate(o, o + cq.Vector(*axis), deg)
    return shape


def pose_arm(q: Dict[str, float]) -> Dict[int, List[cq.Shape]]:
    """관절각 q (deg) 로 자세를 만든 형상을 단계별로 반환."""
    return {st: [place(s, st, q) for s in shapes] for st, shapes in load_home_staged().items()}


def _tris(shape: cq.Shape, tol: float = 2.0):
    verts, faces = shape.tessellate(tol, 0.3)
    v = [(p.x, p.y, p.z) for p in verts]
    return [[v[i] for i in f] for f in faces]


def render(q: Dict[str, float], out: Path, view=(28, -50), title: str = "",
           offset=(0.0, 0.0, 0.0), overlay=None, lims=None, fig=None) -> None:
    """자세 렌더. offset: 팔 원점(J2 축) 의 월드 좌표. overlay(ax) 로 배경 요소 추가.

    fig 가 주어지면 그 figure 에 그리고 저장(out 이 None 이면 저장 생략).
    """
    staged = pose_arm(q)
    own_fig = fig is None
    fig = fig or plt.figure(figsize=(14, 9), dpi=110)
    fig.clf()
    axp = fig.add_subplot(111, projection="3d")
    if overlay:
        overlay(axp)
    ox, oy, oz = offset
    allpts = []
    for st in range(6):
        polys = []
        for s in staged[st]:
            polys += [[(x + ox, y + oy, z + oz) for x, y, z in tri] for tri in _tris(s)]
        if not polys:
            continue
        pc = Poly3DCollection(polys, facecolor=STAGE_COLOR[st], edgecolor="none", alpha=0.95)
        axp.add_collection3d(pc)
        allpts += [p for tri in polys for p in tri]
        axp.plot([], [], color=STAGE_COLOR[st], lw=8, label=f"{st}: {STAGE_NAME[st]}")
    if lims:
        (x0, x1), (y0, y1), (z0, z1) = lims
    else:
        xs, ys, zs = zip(*allpts)
        c = [(max(a) + min(a)) / 2 for a in (xs, ys, zs)]
        r = max(max(a) - min(a) for a in (xs, ys, zs)) / 2 + 50
        (x0, x1), (y0, y1), (z0, z1) = [(ci - r, ci + r) for ci in c]
    axp.set_xlim(x0, x1); axp.set_ylim(y0, y1); axp.set_zlim(z0, z1)
    axp.set_box_aspect((1, 1, 1))
    axp.view_init(elev=view[0], azim=view[1])
    axp.set_xlabel("X"); axp.set_ylabel("Y"); axp.set_zlabel("Z")
    axp.legend(loc="upper left", fontsize=9)
    qtxt = "  ".join(f"{k}={v:+.0f}°" for k, v in q.items())
    axp.set_title(f"{title}\n{qtxt}", fontsize=11)
    if out is not None:
        fig.savefig(out)
    if own_fig:
        plt.close(fig)


if __name__ == "__main__":
    out = Path(__file__).resolve().parents[1] / "output" / "arm"
    out.mkdir(parents=True, exist_ok=True)
    render({"J1": 0, "J2": 0, "J3": 0, "J4": 0, "J5": 0, "J6": 0}, out / "pose_home.png",
           view=(22, -62), title="Home pose (all joints 0)")
    render({"J1": 0, "J2": 40, "J3": -95, "J4": 60, "J5": 55, "J6": 30}, out / "pose_bent.png",
           view=(22, -62), title="All 6 joints articulated")
    render({"J1": 0, "J2": 70, "J3": -140, "J4": 0, "J5": 70, "J6": 0}, out / "pose_fold.png",
           view=(22, -62), title="Folded (transport) pose")
