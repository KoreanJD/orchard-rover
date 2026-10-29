"""팔·J1 판재 부품 2D 가공 도면 (DXF) — 판 두께 중앙 단면, 구멍 위치는 조립 좌표 그대로.

로커/보기 플레이트(rocker_bogie_cad)와 같은 방식: 판 법선을 Z 로 눕힌 뒤 두께 중앙에서 자른다.
도면 원점: 측판·전완판은 팔 좌표계(J2 축 원점, X 팔 방향 → 도면 +X, 높이 Z → 도면 +Y),
J1 브라켓은 터릿 상면 중심, 베이스·터릿은 J1 축 중심.

출력: output/dxf/<부품>.dxf, output/dxf/plates.txt (두께·외곽 치수·구멍 수)
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Tuple

import cadquery as cq

from orchard_rover.arm_assembly import PLATE_T, make_forearm, make_upper_arm, upper_plate_y
from orchard_rover.base_joint import (BaseJointParams, make_base_plate, make_bracket_gusset,
                                      make_shoulder_bracket, make_turret)
from orchard_rover.shoulder_joint import ADAPTER_T, ShoulderParams, elbow_params, joint_stack_length

def _flatten(shape: cq.Shape, normal: str) -> Tuple[cq.Shape, float]:
    """판 법선을 +Z 로 눕힌다. 반환: (형상, 원래 법선 좌표 → 눕힌 Z 부호).

    Y 법선: X 축 −90°, (x, y, z) → (x, z, −y)  — 도면 X = X, 도면 Y = 높이 Z
    X 법선: Z 축 −90° 후 X 축 −90°, (x, y, z) → (y, z, x) — 도면 X = Y, 도면 Y = Z
    """
    o = cq.Vector(0, 0, 0)
    if normal == "Y":
        return shape.rotate(o, cq.Vector(1, 0, 0), -90), -1.0
    if normal == "X":
        s = shape.rotate(o, cq.Vector(0, 0, 1), -90)
        return s.rotate(o, cq.Vector(1, 0, 0), -90), 1.0
    return shape, 1.0


def plate_section(shape: cq.Shape, normal: str, mid: float = None,
                  t: float = None) -> Tuple[cq.Workplane, float]:
    """판 형상을 눕혀 두께 중앙 단면을 반환 (단면, 두께).

    mid: 판 중면의 원래 법선 좌표 — 판에 거셋·허브처럼 법선 방향 덧붙임이 있으면 지정.
    """
    flat, sign = _flatten(shape, normal)
    bb = flat.BoundingBox()
    z = (bb.zmin + bb.zmax) / 2 if mid is None else sign * mid
    sec = cq.Workplane("XY").add(flat).section(z)
    return sec, (bb.zmax - bb.zmin) if t is None else t


Plate = Tuple[cq.Shape, str, str, float, float]   # (형상, 판 법선, 메모, 중면 좌표 | None, 두께 | None)


def plate_parts() -> Dict[str, Plate]:
    """판재 부품. 판 이외 덧붙임(거셋·바닥 플랜지·허브)은 중면 좌표를 지정해 단면에서 빠지게 한다."""
    p2, p3, b = ShoulderParams(), elbow_params(), BaseJointParams()
    y_plate = upper_plate_y(p2)
    upper = make_upper_arm(p2, p3, y_plate)
    fore = make_forearm(p3, y_plate + PLATE_T + joint_stack_length(p3))
    al = "Al 6061-T6"
    return {
        "upper_plate_J2": (upper["upper_plate_J2"][0].val(), "Y", f"{al} {PLATE_T:.0f}t", None, None),
        "upper_plate_J3": (upper["upper_plate_J3"][0].val(), "Y", f"{al} {PLATE_T:.0f}t", None, None),
        "fore_plate_J3": (fore["fore_plate_J3"][0].val(), "Y", f"{al} {ADAPTER_T:.0f}t (J3 어댑터 겸용)",
                          None, None),
        "fore_offset_bracket": (fore["fore_offset_bracket"][0].val(), "X", f"{al} {PLATE_T:.0f}t", None, None),
        # 판 중면 Y = −t/2 (거셋·바닥 플랜지는 Y ≥ 0)
        "shoulder_bracket": (make_shoulder_bracket(b, p2).val(), "Y",
                             f"{al} {b.bracket_t:.0f}t (J2 어댑터 겸용, 거셋 ×2·바닥 플랜지 체결)",
                             -b.bracket_t / 2, b.bracket_t),
        "shoulder_bracket_gusset": (make_bracket_gusset(b).val(), "X", f"{al} {b.gusset_t:.0f}t ×2", None, None),
        "j1_base_plate": (make_base_plate(b).val(), "Z", f"{al} {b.plate_t:.0f}t", None, None),
        # 판 중면 Z = t/2 (아래 허브는 베어링 내륜 끼움)
        "j1_turret": (make_turret(b).val(), "Z", f"{al} {b.turret_t:.0f}t + 허브 (선반 가공)",
                      b.turret_t / 2, b.turret_t),
    }


def export(out: Path) -> str:
    out.mkdir(parents=True, exist_ok=True)
    lines = ["=== 판재 부품 DXF (두께 중앙 단면) ===", f"{'부품':26s} {'두께':>6s} {'외곽 W×H':>14s} {'구멍':>4s}  재질"]
    for name, (shape, normal, note, mid, t_plate) in plate_parts().items():
        sec, t = plate_section(shape, normal, mid, t_plate)
        cq.exporters.exportDXF(sec, str(out / f"{name}.dxf"))
        face = sec.val()
        bb = face.BoundingBox()
        n_holes = sum(len(f.innerWires()) for f in sec.faces().vals())
        lines.append(f"{name:26s} {t:6.1f} {bb.xlen:6.0f}×{bb.ylen:<6.0f} {n_holes:4d}  {note}")
    txt = "\n".join(lines)
    (out / "plates.txt").write_text(txt, encoding="utf-8")
    return txt


if __name__ == "__main__":
    print(export(Path(__file__).resolve().parents[1] / "output" / "dxf"))
