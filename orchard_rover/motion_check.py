"""관절 가동 범위 간섭 검토 — 부품 단위로 자세를 만들어 링크 간 간섭 부피를 구한다.

구동체 좌표계. 그룹:
  fixed  구동체 전체 + J1 고정부(베이스 플레이트·크로스멤버·구동부·크로스롤러)
  0~5    팔 체인 단계 (arm_pose_render.stage_of). 0 = J1 회전부(터릿·브라켓·클레비스) + J2 고정부
  spring 가스 스프링 (J1·J2 자세에 따라 양 앵커 사이)
같은 그룹 쌍은 상대 운동이 없으므로 제외하고, 홈 자세에서 이미 맞닿은 쌍(관절 체결·끼워맞춤)도
관절을 따라 함께 도는 짝이므로 제외한다. 그 밖의 쌍은 간섭 부피 > 1 mm³ 이면 충돌.

판정:
  A. 단일 관절 스윕 (나머지 관절 0) — 한계 전 범위에서 충돌 0 이어야 함 (설계 결함)
  B. 조합 자세 — 충돌 자세를 운용 제약(경로 계획에서 회피)으로 기록
"""

from __future__ import annotations

from itertools import combinations
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import cadquery as cq
import numpy as np

from orchard_rover.arm_assembly import build_arm
from orchard_rover.arm_pose_render import place, stage_of
from orchard_rover.base_joint import BaseJointParams, build_j1
from orchard_rover.rocker_bogie_cad import build_assembly as build_rover
from orchard_rover.rocker_bogie_params import SuspensionParams
from orchard_rover.shoulder_joint import ShoulderParams
from orchard_rover.transport_check import J2_ORIGIN, gas_spring_posed

LIMITS = {"J1": (-180.0, 180.0), "J2": (-20.0, 90.0), "J3": (-120.0, 120.0),
          "J4": (-180.0, 180.0), "J5": (-110.0, 110.0), "J6": (-180.0, 180.0)}
J1_ROTATING = ("turret", "shoulder_bracket", "gs_clevis")
SPRING_ANCHORS = ("gs_clevis", "gs_anchor_arm")
MATE_TOL = 0.1          # 홈 자세 접촉 판정 (mm)
COLLIDE_VOL = 1.0       # 충돌 판정 간섭 부피 (mm³)

Part = Tuple[str, object, cq.Shape]   # (이름, 그룹, 홈 형상: 팔은 팔 좌표, 나머지는 구동체 좌표)


def _walk(asm: cq.Assembly, loc: cq.Location, out: Dict[str, cq.Shape]) -> None:
    for ch in asm.children:
        l = loc * ch.loc if ch.loc is not None else loc
        if ch.obj is not None:
            shp = ch.obj.val() if isinstance(ch.obj, cq.Workplane) else ch.obj
            out[ch.name] = shp.moved(l)
        _walk(ch, l, out)


def load_parts() -> List[Part]:
    p = SuspensionParams()
    rover: Dict[str, cq.Shape] = {}
    _walk(build_rover(p, 0.0), cq.Location(), rover)
    j1: Dict[str, cq.Shape] = {}
    _walk(build_j1(BaseJointParams(), p, ShoulderParams())[0], cq.Location(), j1)
    arm: Dict[str, cq.Shape] = {}
    _walk(build_arm(j2_adapter=False), cq.Location(), arm)
    parts: List[Part] = [(f"rover:{n}", "fixed", s) for n, s in rover.items()]
    parts += [(f"J1:{n}", 0 if n in J1_ROTATING else "fixed", s) for n, s in j1.items()]
    parts += [(n, stage_of(n), s) for n, s in arm.items()]
    return parts


def posed(parts: List[Part], q: Dict[str, float]) -> Dict[str, Tuple[object, cq.Shape]]:
    """자세 q 의 부품 형상 (구동체 좌표) + 가스 스프링."""
    org = cq.Vector(*J2_ORIGIN)
    out: Dict[str, Tuple[object, cq.Shape]] = {}
    for name, grp, shp in parts:
        if grp == "fixed":
            out[name] = (grp, shp)
        elif name.startswith("J1:"):           # 구동체 좌표 → J1 축 기준 요
            s = shp.translate(-org)
            out[name] = (grp, place(s, 0, {"J1": q.get("J1", 0)}).translate(org))
        else:
            out[name] = (grp, place(shp, grp, q).translate(org))
    out["gas_spring"] = ("spring", gas_spring_posed(q))
    return out


def _bb_overlap(a: cq.BoundBox, b: cq.BoundBox, tol: float = 0.0) -> bool:
    return not (a.xmin > b.xmax + tol or b.xmin > a.xmax + tol or a.ymin > b.ymax + tol or
                b.ymin > a.ymax + tol or a.zmin > b.zmax + tol or b.zmin > a.zmax + tol)


def mating_pairs(home: Dict[str, Tuple[object, cq.Shape]]) -> set:
    """홈 자세에서 서로 다른 그룹이면서 이미 맞닿은 쌍 (관절 체결부)."""
    bbs = {n: s.BoundingBox() for n, (_, s) in home.items()}
    mates = {frozenset((a, "gas_spring")) for a in home if a.split(":")[-1] in SPRING_ANCHORS}
    for a, b in combinations(home, 2):
        ga, gb = home[a][0], home[b][0]
        if ga == gb or not _bb_overlap(bbs[a], bbs[b], MATE_TOL):
            continue
        if home[a][1].distance(home[b][1]) < MATE_TOL:
            mates.add(frozenset((a, b)))
    return mates


def collisions(shapes: Dict[str, Tuple[object, cq.Shape]], mates: set) -> List[Tuple[float, str, str]]:
    bbs = {n: s.BoundingBox() for n, (_, s) in shapes.items()}
    hits = []
    for a, b in combinations(shapes, 2):
        ga, gb = shapes[a][0], shapes[b][0]
        if ga == gb or frozenset((a, b)) in mates or not _bb_overlap(bbs[a], bbs[b]):
            continue
        v = shapes[a][1].intersect(shapes[b][1]).Volume()
        if v > COLLIDE_VOL:
            hits.append((v, a, b))
    return sorted(hits, reverse=True)


def single_sweeps(step: float = 10.0) -> Iterable[Tuple[str, Dict[str, float]]]:
    for j, (lo, hi) in LIMITS.items():
        for a in np.arange(lo, hi + 1e-9, step):
            yield f"A:{j}", {j: float(a)}


def combo_poses() -> Iterable[Tuple[str, Dict[str, float]]]:
    for j2 in (-20.0, 0.0, 45.0, 90.0):
        for j3 in np.arange(-120.0, 121.0, 15.0):
            yield "B:J2xJ3", {"J2": j2, "J3": float(j3)}
    for j3 in (0.0, -120.0):
        for j4 in (0.0, 90.0, 180.0, -90.0):
            for j5 in np.arange(-110.0, 111.0, 22.0):
                yield "B:J3xJ4xJ5", {"J3": j3, "J4": j4, "J5": float(j5)}
    for j1 in np.arange(-180.0, 181.0, 45.0):
        for j2 in (-20.0, 0.0, 90.0):
            for j3 in (0.0, -120.0):
                yield "B:J1xJ2xJ3", {"J1": float(j1), "J2": j2, "J3": j3}


def run(poses: Iterable[Tuple[str, Dict[str, float]]], parts: List[Part] = None,
        mates: set = None) -> List[Tuple[str, Dict[str, float], List[Tuple[float, str, str]]]]:
    parts = parts if parts is not None else load_parts()
    mates = mates if mates is not None else mating_pairs(posed(parts, {}))
    return [(tag, q, collisions(posed(parts, q), mates)) for tag, q in poses]


def _fmt_q(q: Dict[str, float]) -> str:
    return " ".join(f"{k}={v:+.0f}" for k, v in q.items())


def main(out: Path) -> None:
    out.mkdir(parents=True, exist_ok=True)
    parts = load_parts()
    mates = mating_pairs(posed(parts, {}))
    lines = ["=== 관절 가동 범위 간섭 검토 (부품 단위, 간섭 > 1 mm³) ===",
             f"부품 {len(parts) + 1}, 홈 접촉 제외 쌍 {len(mates)}", ""]
    single = run(single_sweeps(), parts, mates)
    bad = [(t, q, h) for t, q, h in single if h]
    lines.append(f"[A] 단일 관절 스윕 (10° 간격, 나머지 0): {len(single)} 자세, 충돌 {len(bad)}")
    for t, q, h in bad:
        lines.append(f"   {_fmt_q(q):28s} " + ", ".join(f"{a}↔{b} {v:.0f}" for v, a, b in h[:4]))
    combo = run(combo_poses(), parts, mates)
    lines.append("")
    lines.append(f"[B] 조합 자세: {len(combo)} 자세, 충돌 {sum(1 for _, _, h in combo if h)} (운용 제약)")
    for tag in sorted({t for t, _, _ in combo}):
        rows = [(q, h) for t, q, h in combo if t == tag and h]
        lines.append(f" {tag}: 충돌 {len(rows)}/{sum(1 for t, _, _ in combo if t == tag)}")
        for q, h in rows:
            v, a, b = h[0]
            lines.append(f"   {_fmt_q(q):28s} {a}↔{b} {v:.0f}" + (f" 외 {len(h) - 1}" if len(h) > 1 else ""))
    txt = "\n".join(lines)
    (out / "motion_check.txt").write_text(txt, encoding="utf-8")
    print(txt)


if __name__ == "__main__":
    main(Path(__file__).resolve().parents[1] / "output" / "arm")
