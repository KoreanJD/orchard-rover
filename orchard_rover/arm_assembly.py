"""로봇팔 J2–상완–J3–전완 조립.

팔 좌표계: X = 팔 전개 방향, Y = 관절 축, Z = 상방. J2 축 = 원점.
조인트 모듈(shoulder_joint)은 Z 축 적층으로 생성되므로 X 축 회전으로 Y 축에 정렬한다.
  J2: rotate +90° → 출력 플랜지가 -Y, 모터가 +Y (피치 평면 y→z: 가스 스프링 앵커가 위로)
  J3: rotate -90° → 출력 플랜지가 +Y, 모터가 -Y (상완 플레이트 바깥쪽)

상완: HFS5-3030 ×2 (Z 방향 60 간격 사다리) + 양단 12t Al 측판 (-Y 면), 길이 500 (축간)
전완: HFS5-3030 ×1, 길이 450 (축간) — 플레이스홀더
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict

import cadquery as cq

from orchard_rover.shoulder_joint import (ShoulderParams, build, elbow_params, joint_stack_length)
from orchard_rover.wrist_joint import WristParams, build_wrist

UPPER_LEN = 500.0
FORE_LEN = 450.0
RAIL = 30.0
RAIL_GAP = 60.0          # 두 레일 면 사이 간격 (Z)
PLATE_T = 12.0
COL = {
    "rail": cq.Color(0.6, 0.6, 0.63), "plate": cq.Color(0.85, 0.3, 0.3),
    "bracket": cq.Color(0.4, 0.4, 0.45), "base": cq.Color(0.3, 0.3, 0.35),
}


def _rot_x(obj: cq.Workplane, deg: float) -> cq.Workplane:
    return obj.rotate((0, 0, 0), (1, 0, 0), deg)


def _joint_shapes(p: ShoulderParams, rot_deg: float, origin: tuple) -> Dict[str, tuple]:
    """조인트 조립체를 형상 dict 로 평탄화하여 회전·이동."""
    asm, _ = build(p)
    out = {}
    for ch in asm.children:
        obj = cq.Workplane().add(ch.obj) if not isinstance(ch.obj, cq.Workplane) else ch.obj
        if ch.loc is not None:
            obj = cq.Workplane().add(ch.obj.moved(ch.loc)) if hasattr(ch.obj, "moved") else obj
        out[ch.name] = (_rot_x(obj, rot_deg).translate(origin), ch.color)
    return out


def make_upper_arm(p2: ShoulderParams, p3: ShoulderParams, y_plate: float) -> Dict[str, tuple]:
    """상완 링크: 레일 2본 + J2 측판 + J3 측판 + 가스 스프링 앵커 브라켓."""
    shapes: Dict[str, tuple] = {}
    # 측판 Y 위치: J2 출력 플랜지 면 (−Y 쪽)
    z_rail = RAIL_GAP / 2 + RAIL / 2
    rail_len = UPPER_LEN - 120.0          # 양단 측판 영역 60 씩 제외
    for i, z in enumerate((z_rail, -z_rail)):
        rail = (cq.Workplane("XY").box(rail_len, RAIL, RAIL)
                .translate((UPPER_LEN / 2, y_plate + PLATE_T + RAIL / 2, z)))
        shapes[f"upper_rail_{i}"] = (rail, COL["rail"])
    # J2 측판: 플랜지 체결 (PCD 90) + 레일 T-슬롯 체결 홀
    plate_h = RAIL_GAP + 2 * RAIL + 20
    p_j2 = (cq.Workplane("XZ").center(90, 0).rect(240, plate_h).extrude(-PLATE_T)
            .edges("|Y").fillet(10))
    p_j2 = p_j2.union(cq.Workplane("XZ").circle(plate_h / 2).extrude(-PLATE_T))
    p_j2 = p_j2.faces(">Y").workplane().circle(p2.output_bearing[0] / 2 - 5).cutThruAll()
    bolt6 = [(45 * math.cos(math.radians(a)), 45 * math.sin(math.radians(a))) for a in range(0, 360, 60)]
    p_j2 = p_j2.faces(">Y").workplane().pushPoints(bolt6).circle(3.3).cutThruAll()
    rail_bolts = [(x, z) for x in (90, 150, 190) for z in (z_rail, -z_rail)]
    p_j2 = p_j2.faces(">Y").workplane().pushPoints(rail_bolts).circle(3.3).cutThruAll()
    shapes["upper_plate_J2"] = (p_j2.translate((0, y_plate + PLATE_T, 0)), COL["plate"])
    # J3 측판: J3 어댑터 체결 + 레일 체결, 중앙 관통 (출력 플랜지 통과)
    p_j3 = (cq.Workplane("XZ").center(UPPER_LEN - 90, 0).rect(240, plate_h).extrude(-PLATE_T)
            .edges("|Y").fillet(10))
    p_j3 = p_j3.union(cq.Workplane("XZ").center(UPPER_LEN, 0).circle(plate_h / 2).extrude(-PLATE_T))
    p_j3 = (p_j3.faces(">Y").workplane().center(UPPER_LEN, 0)
            .circle(p3.output_bearing[1] / 2 + 1).cutThruAll())
    p_j3 = (p_j3.faces(">Y").workplane().center(UPPER_LEN, 0)
            .pushPoints([(x, z) for x in (-90, -150, -190) for z in (z_rail, -z_rail)])
            .circle(3.3).cutThruAll())
    shapes["upper_plate_J3"] = (p_j3.translate((0, y_plate + PLATE_T, 0)), COL["plate"])
    # 가스 스프링 상완 앵커 (J2 축에서 r=100, 레일 상단)
    anchor = (cq.Workplane("XY").box(30, 12, 40)
              .translate((p2.gs_r, y_plate + PLATE_T + RAIL / 2, z_rail + RAIL / 2 + 20)))
    shapes["gs_anchor_arm"] = (anchor, COL["bracket"])
    return shapes


def make_forearm(y_plate: float) -> Dict[str, tuple]:
    """전완: J3 출력 플랜지 측판 → 6축 손목 체인 (J4 롤 / J5 피치 / J6 롤 / ISO 플랜지).

    손목 체인은 팔 중심선(Y=0) 에 두고, J3 측판에서 Y 오프셋 브라켓으로 연결한다.
    """
    shapes = {}
    plate = (cq.Workplane("XZ").center(UPPER_LEN + 45, 0).rect(130, 90).extrude(-PLATE_T)
             .union(cq.Workplane("XZ").center(UPPER_LEN, 0).circle(55).extrude(-PLATE_T))
             .edges("|Y").fillet(8))
    shapes["fore_plate_J3"] = (plate.translate((0, y_plate + PLATE_T, 0)), COL["plate"])
    # 오프셋 브라켓: 측판 (Y=y_plate) → J4 하우징 플랜지 (Y=0 중심)
    y0, y1 = sorted((y_plate + PLATE_T, 0.0))
    bracket = (cq.Workplane("XY").box(PLATE_T, y1 - y0 + 45, 90)
               .translate((UPPER_LEN + 60 + PLATE_T / 2, (y0 + y1) / 2, 0)))
    shapes["fore_offset_bracket"] = (bracket, COL["plate"])
    for n, (o, c) in build_wrist(WristParams(x_j3=UPPER_LEN, x_flange=UPPER_LEN + FORE_LEN + 100)).items():
        shapes[n] = (o, c)
    return shapes


def build_arm() -> cq.Assembly:
    p2, p3 = ShoulderParams(), elbow_params()
    asm = cq.Assembly(name="arm_J2_J3")

    # J2: 출력이 −Y. 모터 전면 Z=0 → 회전 후 Y=0 평면. 출력 플랜지 상면 Y = −stack
    j2 = _joint_shapes(p2, +90.0, (0, 0, 0))
    y_flange_j2 = -joint_stack_length(p2)
    for n, (o, c) in j2.items():
        asm.add(o, name=f"J2_{n}", color=c)
    # 상완 측판은 J2 플랜지 상면에 밀착 (−Y 쪽으로 두께)
    y_plate = y_flange_j2 - PLATE_T
    for n, (o, c) in make_upper_arm(p2, p3, y_plate).items():
        asm.add(o, name=n, color=c)
    # J3: 어댑터(고정부) 가 상완 J3 측판 −Y 면에 체결, 출력이 +Y 로 측판을 관통
    #     모터 전면(Z=0) 이 −Y 방향 회전 후 Y = y_plate − 10(어댑터) 에 오도록 배치
    j3 = _joint_shapes(p3, -90.0, (UPPER_LEN, y_plate - 10.0 - PLATE_T, 0))
    for n, (o, c) in j3.items():
        asm.add(o, name=f"J3_{n}", color=c)
    # 전완: J3 출력 플랜지 상면 (+Y 쪽)
    y_fore_plate = y_plate - 10.0 - PLATE_T + joint_stack_length(p3)
    for n, (o, c) in make_forearm(y_fore_plate).items():
        asm.add(o, name=n, color=c)
    # 베이스 가스 스프링 앵커 포스트 (J1 측, J2 축 위 h)
    post = cq.Workplane("XY").box(30, 30, p2.gs_h + 20).translate((0, 40, p2.gs_h / 2))
    asm.add(post, name="gs_anchor_base_post", color=COL["base"])
    return asm


def export(out: Path) -> None:
    out.mkdir(parents=True, exist_ok=True)
    asm = build_arm()
    asm.save(str(out / "arm_J2_J3_assembly.step"))
    comp = cq.Workplane().add(asm.toCompound())
    cq.exporters.export(comp, str(out / "arm_iso.svg"),
                        opt={"width": 1400, "height": 900, "projectionDir": (1, -1.2, 0.8),
                             "showHidden": False, "strokeWidth": 0.5})
    cq.exporters.export(comp, str(out / "arm_side.svg"),
                        opt={"width": 1400, "height": 700, "projectionDir": (0, -1, 0),
                             "showHidden": False, "strokeWidth": 0.5})
    try:
        import cairosvg
        from PIL import Image, ImageOps
        for n in ("iso", "side"):
            svg, png = out / f"arm_{n}.svg", out / f"arm_{n}.png"
            cairosvg.svg2png(url=str(svg), write_to=str(png), output_width=1400, background_color="white")
            img = ImageOps.flip(Image.open(png))
            if n == "side":
                img = img.transpose(Image.Transpose.TRANSPOSE)
            img.save(png)
    except ImportError:
        pass


if __name__ == "__main__":
    export(Path(__file__).resolve().parents[1] / "output" / "arm")
