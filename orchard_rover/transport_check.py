"""운반(접힘) 자세 간섭 검토 — 팔 링크 vs 구동체/J1/가스 스프링, 팔 링크 상호 간 최소 거리.

구동체 좌표계. 팔 원점(J2 축) = (300, 0, 958).
검사 쌍:
  A. 팔 이동부(상완~플랜지) ↔ 정적부(구동체 프레임·차동 바·J1 플레이트·터릿·브라켓·가스 스프링)
  B. 전완·손목(단계 ≥ 2) ↔ 상완·J2 고정부(단계 ≤ 1)   — 팔 자체 접힘 간섭
합격 기준: 최소 거리 ≥ 15 mm (케이블·커버 여유)
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Tuple

import cadquery as cq

from orchard_rover.arm_pose_render import pose_arm, STAGE_NAME
from orchard_rover.base_joint import BaseJointParams, build_j1
from orchard_rover.rocker_bogie_cad import build_assembly as build_rover
from orchard_rover.rocker_bogie_params import SuspensionParams
from orchard_rover.shoulder_joint import ShoulderParams

J2_ORIGIN = (300.0, 0.0, 958.0)
MIN_CLEAR = 15.0


def _compound(shapes: List[cq.Shape]) -> cq.Compound:
    return cq.Compound.makeCompound(shapes)


def static_shapes() -> Dict[str, cq.Shape]:
    """정적 간섭 대상: 프레임(휠·서스펜션 제외), 차동 바, J1 부품, 가스 스프링 베이스 포스트."""
    p = SuspensionParams()
    rover = build_rover(p, 0.0)
    out: Dict[str, cq.Shape] = {}
    for ch in rover.children:
        if ch.name in ("frame_ref", "diff_post", "diff_bar"):
            shp = ch.obj.val() if isinstance(ch.obj, cq.Workplane) else ch.obj
            if ch.loc is not None:
                shp = shp.moved(ch.loc)
            out[f"rover:{ch.name}"] = shp
    j1, _ = build_j1(BaseJointParams(), p, ShoulderParams())
    for ch in j1.children:
        shp = ch.obj.val() if isinstance(ch.obj, cq.Workplane) else ch.obj
        out[f"J1:{ch.name}"] = shp
    return out


def gas_spring_posed(j2_deg: float) -> cq.Shape:
    """가스 스프링: 베이스 앵커 (J2 축 위 h) → 상완 앵커 (팔 위 r, J2 와 함께 회전). Ø22 실린더."""
    b, p2 = BaseJointParams(), ShoulderParams()
    ox, oy, oz = J2_ORIGIN
    a0 = cq.Vector(ox, oy - 80, oz + p2.gs_h)                 # 브라켓 상단 앵커 (상완 플레이트 외측 Y=−80 근처)
    th = math.radians(j2_deg)
    a1 = cq.Vector(ox + p2.gs_r * math.cos(th), oy - 80, oz + p2.gs_r * math.sin(th))
    v = a1 - a0
    plane = cq.Plane(origin=a0, normal=v.normalized())
    return cq.Workplane(plane).circle(11).extrude(v.Length).val()


def posed_arm_shapes(q: Dict[str, float]) -> Dict[int, cq.Shape]:
    staged = pose_arm(q)
    out = {}
    for st, shapes in staged.items():
        if shapes:
            out[st] = _compound(shapes).translate(cq.Vector(*J2_ORIGIN))
    return out


def check_pose(q: Dict[str, float], statics: Dict[str, cq.Shape]) -> Dict[str, object]:
    arm = posed_arm_shapes(q)
    spring = gas_spring_posed(q["J2"])
    results: List[Tuple[str, str, float]] = []
    moving = {st: s for st, s in arm.items() if st >= 1}
    # A. 이동부 ↔ 정적부
    for st, s in moving.items():
        for name, t in list(statics.items()) + [("gas_spring", spring)]:
            # J2 출력 플랜지/상완(1단계)와 J1 브라켓·스프링은 설계상 인접 → 거리 0 허용 쌍 제외
            if st == 1 and name in ("J1:shoulder_bracket", "gas_spring"):
                continue
            d = s.distance(t)
            results.append((f"arm[{st}]", name, d))
    # B. 팔 자체: 단계 ≥2 ↔ 단계 0·1 (인접 2↔1 은 J3 조인트로 맞닿으므로 제외)
    for st_hi in (3, 4, 5):
        for st_lo in (0, 1):
            if st_lo in arm and st_hi in arm:
                results.append((f"arm[{st_hi}]", f"arm[{st_lo}]", arm[st_hi].distance(arm[st_lo])))
    if 2 in arm and 0 in arm:
        results.append(("arm[2]", "arm[0]", arm[2].distance(arm[0])))
    bb = _compound(list(arm.values())).BoundingBox()
    worst = min(results, key=lambda r: r[2])
    return {"q": q, "results": results, "min_clear": worst[2], "worst_pair": worst[:2],
            "x_max": bb.xmax, "x_min": bb.xmin, "z_max": bb.zmax, "y_half": max(abs(bb.ymin), abs(bb.ymax)),
            "ok": worst[2] >= MIN_CLEAR}


CANDIDATES = {
    "T1_fold_fwd":  {"J1": 0, "J2": 80, "J3": -120, "J4": 0, "J5": 60, "J6": 0},   # 현재 운반 자세
    "T2_fold_tight": {"J1": 0, "J2": 90, "J3": -120, "J4": 0, "J5": 30, "J6": 0},
    "T3_fold_back": {"J1": 180, "J2": 60, "J3": -120, "J4": 0, "J5": 60, "J6": 0},  # 뒤로 접어 차체 위
    "T4_low_fwd":   {"J1": 0, "J2": 45, "J3": -120, "J4": 0, "J5": 75, "J6": 0},
    "T5_side":      {"J1": 90, "J2": 80, "J3": -120, "J4": 0, "J5": 40, "J6": 0},
}


def main(out: Path) -> List[Dict[str, object]]:
    out.mkdir(parents=True, exist_ok=True)
    statics = static_shapes()
    rows = []
    lines = ["=== 운반 자세 간섭 검토 (최소 거리 mm, 합격 ≥ 15) ===",
             f"{'자세':14s} {'J1':>4s} {'J2':>4s} {'J3':>5s} {'J5':>4s} | {'min':>6s}  최악 쌍{'':22s} | {'Xmax':>6s} {'Zmax':>6s} {'Y반폭':>6s}  판정"]
    for name, q in CANDIDATES.items():
        r = check_pose(q, statics)
        rows.append((name, r))
        wp = f"{r['worst_pair'][0]}↔{r['worst_pair'][1]}"
        lines.append(f"{name:14s} {q['J1']:4.0f} {q['J2']:4.0f} {q['J3']:5.0f} {q['J5']:4.0f} | "
                     f"{r['min_clear']:6.1f}  {wp:28s} | {r['x_max']:6.0f} {r['z_max']:6.0f} {r['y_half']:6.0f}  "
                     f"{'OK' if r['ok'] else 'FAIL'}")
    lines.append("")
    for name, r in rows:
        lines.append(f"[{name}] 15 mm 미만 쌍:")
        bad = [x for x in r["results"] if x[2] < MIN_CLEAR]
        lines += [f"   {a} ↔ {b}: {d:.1f}" for a, b, d in sorted(bad, key=lambda x: x[2])] or ["   (없음)"]
    txt = "\n".join(lines)
    (out / "transport_check.txt").write_text(txt, encoding="utf-8")
    print(txt)
    return [r for _, r in rows]


if __name__ == "__main__":
    main(Path(__file__).resolve().parents[1] / "output" / "arm")
