"""로봇팔 J2–상완–J3–전완 조립.

팔 좌표계: X = 팔 전개 방향, Y = 관절 축, Z = 상방. J2 축 = 원점.
조인트 모듈(shoulder_joint)은 Z 축 적층으로 생성되므로 X 축 회전으로 Y 축에 정렬한다.
  J2, J3 모두 rotate +90° → 출력 플랜지가 −Y, 모터가 +Y.
  두 조인트 모두 출력 플랜지가 상완 측판 +Y 면에 체결된다 (같은 방향).
    J2: 링·어댑터·모터 = 베이스(J1 브라켓이 어댑터 겸용), 플랜지 = 상완
    J3: 플랜지 = 상완, 링·모터 = 전완 (전완판이 J3 어댑터 겸용)

상완: HFS6-3030 ×2 (Z 방향 60 간격 사다리, 양단 링 하우징 회피) + 양단 12t Al 측판, 500 (축간)
전완: 전완판(10t, J3 어댑터 자리) + J4 하우징 연결 브라켓 + 6축 손목 체인
가스 스프링 MISUMI FGSS22120C: J1 터릿 클레비스 (J2 축 −35, −150) → 상완 측판 바깥면 스터드 (r280, +10°)
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict

import cadquery as cq

from orchard_rover.shoulder_joint import (ADAPTER_T, ShoulderParams, adapter_radius, build, elbow_params,
                                          gas_spring_arm_anchor, joint_stack_length)
from orchard_rover.wrist_joint import NEMA17, WristParams, build_wrist

UPPER_LEN = 500.0
FORE_LEN = 450.0
RAIL = 30.0
RAIL_GAP = 60.0          # 두 레일 면 사이 간격 (Z)
RAIL_END = 85.0          # 관절 축 → 레일 끝 (J2 링 하우징 r82.5 + 여유)
PLATE_T = 12.0
J2_PLATE_END = 285.0    # J2 측판 끝 X (가스 스프링 스터드 r280 수용, J3 측판 X 290)
GS_PLANE_GAP = 14.0      # 상완 측판 바깥면 → 가스 스프링 중심 평면 (간극 3 + 실린더 r11)
COL = {
    "rail": cq.Color(0.6, 0.6, 0.63), "plate": cq.Color(0.85, 0.3, 0.3),
    "bracket": cq.Color(0.4, 0.4, 0.45), "base": cq.Color(0.3, 0.3, 0.35),
}


def _rot_x(obj: cq.Workplane, deg: float) -> cq.Workplane:
    return obj.rotate((0, 0, 0), (1, 0, 0), deg)


# 조인트 단독 조립(shoulder_assembly.step) 시각화용 임시 부품 — 팔 조립에서는
# 상완 측판·전완판·레일이 대신하므로 제외 (넣으면 정식 부품과 겹침)
JOINT_PLACEHOLDERS = ("upper_arm_plate", "gas_spring")


def _joint_shapes(p: ShoulderParams, rot_deg: float, origin: tuple) -> Dict[str, tuple]:
    """조인트 조립체를 형상 dict 로 평탄화하여 회전·이동 (임시 부품 제외)."""
    asm, _ = build(p)
    out = {}
    for ch in asm.children:
        if ch.name in JOINT_PLACEHOLDERS:
            continue
        obj = cq.Workplane().add(ch.obj) if not isinstance(ch.obj, cq.Workplane) else ch.obj
        if ch.loc is not None:
            obj = cq.Workplane().add(ch.obj.moved(ch.loc)) if hasattr(ch.obj, "moved") else obj
        out[ch.name] = (_rot_x(obj, rot_deg).translate(origin), ch.color)
    return out


def _cut_y(plate: cq.Workplane, pts, d: float) -> cq.Workplane:
    """XZ 좌표(X, Z) 점들에 Y 방향 관통 구멍(지름 d)을 뚫는다.

    faces(">Y").workplane() 은 .center() 로 옮긴 원점을 상속하고 로컬 x 가 −X 로 잡혀
    구멍이 판 밖으로 나갔었음 → XZ 평면 커터로 월드 좌표를 직접 지정한다.
    """
    cutter = cq.Workplane("XZ").pushPoints(pts).circle(d / 2).extrude(100, both=True)
    return plate.cut(cutter)


def _bolt6(cx: float, pcd: float):
    return [(cx + pcd / 2 * math.cos(math.radians(a)), pcd / 2 * math.sin(math.radians(a)))
            for a in range(0, 360, 60)]


def upper_plate_y(p2: ShoulderParams) -> float:
    """상완 측판 바깥면(−Y) Y — J2 출력 플랜지 상면에서 판 두께만큼 바깥."""
    return -joint_stack_length(p2) - PLATE_T


def gas_spring_y(p2: ShoulderParams) -> float:
    """가스 스프링 중심 평면 Y (팔 좌표계) — 상완 측판 바깥."""
    return upper_plate_y(p2) - GS_PLANE_GAP


def make_upper_arm(p2: ShoulderParams, p3: ShoulderParams, y_plate: float) -> Dict[str, tuple]:
    """상완 링크: 레일 2본 + J2 측판 + J3 측판 + 가스 스프링 상완 앵커 스터드.

    J2·J3 출력 플랜지가 각각 측판 +Y 면에 체결 (PCD 볼트 ×6, 중앙 저널 여유 홀).
    """
    shapes: Dict[str, tuple] = {}
    z_rail = RAIL_GAP / 2 + RAIL / 2
    rail_len = UPPER_LEN - 2 * RAIL_END   # 양단 링 하우징 회피
    for i, z in enumerate((z_rail, -z_rail)):
        rail = (cq.Workplane("XY").box(rail_len, RAIL, RAIL)
                .translate((UPPER_LEN / 2, y_plate + PLATE_T + RAIL / 2, z)))
        shapes[f"upper_rail_{i}"] = (rail, COL["rail"])
    # J2 측판: 플랜지 체결 (PCD 90) + 레일 T-슬롯 체결 홀, 가스 스프링 스터드 자리까지 X 285 로 연장
    # (J3 측판 시작 X 290 과 5 간격)
    plate_h = RAIL_GAP + 2 * RAIL + 20
    x0, x1 = -30.0, J2_PLATE_END
    p_j2 = (cq.Workplane("XZ").center((x0 + x1) / 2, 0).rect(x1 - x0, plate_h).extrude(-PLATE_T)
            .edges("|Y").fillet(10))
    p_j2 = p_j2.union(cq.Workplane("XZ").circle(plate_h / 2).extrude(-PLATE_T))
    p_j2 = _cut_y(p_j2, [(0, 0)], p2.output_bearing[0] - 10)
    p_j2 = _cut_y(p_j2, _bolt6(0.0, p2.flange_bolt_pcd), 6.6)
    rail_bolts = [(x, z) for x in (100, 150, 190) for z in (z_rail, -z_rail)]
    p_j2 = _cut_y(p_j2, rail_bolts + [(240.0, z_rail), (240.0, -z_rail)], 6.6)
    # XZ 압출(-PLATE_T)은 +Y 방향 → Y ∈ [y_plate, y_plate + PLATE_T]
    shapes["upper_plate_J2"] = (p_j2.translate((0, y_plate, 0)), COL["plate"])
    # J3 측판: J3 출력 플랜지 체결 (J2 측판과 같은 방식) + 레일 체결
    p_j3 = (cq.Workplane("XZ").center(UPPER_LEN - 90, 0).rect(240, plate_h).extrude(-PLATE_T)
            .edges("|Y").fillet(10))
    p_j3 = p_j3.union(cq.Workplane("XZ").center(UPPER_LEN, 0).circle(plate_h / 2).extrude(-PLATE_T))
    p_j3 = _cut_y(p_j3, [(UPPER_LEN, 0)], p3.output_bearing[0] - 10)
    p_j3 = _cut_y(p_j3, _bolt6(UPPER_LEN, p3.flange_bolt_pcd), 6.6)
    p_j3 = _cut_y(p_j3, [(UPPER_LEN - x, z) for x, z in rail_bolts], 6.6)
    shapes["upper_plate_J3"] = (p_j3.translate((0, y_plate, 0)), COL["plate"])
    # 가스 스프링 상완 앵커: 측판 바깥면 M8 스터드 (측판 탭, 스페이서 + 핀) — 팔 수평일 때 위치
    ax, az = gas_spring_arm_anchor(p2, 0.0)
    y_eye = y_plate - GS_PLANE_GAP
    spacer = (cq.Workplane("XZ").center(ax, az).circle(8).extrude(y_plate - (y_eye + 6))
              .translate((0, y_plate, 0)))                      # XZ 압출(+) = −Y 방향
    pin = (cq.Workplane("XZ").center(ax, az).circle(4).extrude(14)
           .translate((0, y_eye + 6, 0)))
    shapes["gs_anchor_arm"] = (spacer.union(pin), COL["bracket"])
    return shapes


def make_forearm(p3: ShoulderParams, y_face: float) -> Dict[str, tuple]:
    """전완: 전완판(J3 어댑터 겸용) → 연결 브라켓 → 6축 손목 체인 (J4 롤 / J5 피치 / J6 롤 / ISO).

    전완판은 J3 모터 전면(Y=y_face)과 링 하우징 사이 어댑터 자리 [y_face − ADAPTER_T, y_face].
    J4 하우징 플랜지는 YZ 평면 연결 브라켓에 체결되고, J4 모터는 브라켓·전완판 창으로 뒤로 빠진다.
    """
    shapes = {}
    w = WristParams(x_j3=UPPER_LEN, x_flange=UPPER_LEN + FORE_LEN + 100)
    x_h = UPPER_LEN + w.j4_offset          # J4 하우징 후단
    x_br = x_h - PLATE_T                   # 브라켓 후단 = 전완판 끝
    win = NEMA17 / 2 + 2                   # J4 모터 통과 창 반폭
    x_win = x_h + 10 - 48 - 2              # J4 모터 후단 − 여유
    r_ad = adapter_radius(p3)
    plate = (cq.Workplane("XZ").center(UPPER_LEN, 0).circle(r_ad).extrude(-ADAPTER_T)
             .union(cq.Workplane("XZ").center((UPPER_LEN + x_br) / 2, 0).rect(x_br - UPPER_LEN, 90)
                    .extrude(-ADAPTER_T)))
    plate = plate.intersect(cq.Workplane("XY").box(x_br - (UPPER_LEN - r_ad) + 2, 50, 2 * r_ad + 2)
                            .translate(((x_br + UPPER_LEN - r_ad - 2) / 2, ADAPTER_T / 2, 0)))
    plate = _cut_y(plate, [(UPPER_LEN, 0)], 40.0)                    # NEMA23 보스 Ø38.1
    plate = _cut_y(plate, [(UPPER_LEN + sx * 23.57, sz * 23.57) for sx in (1, -1) for sz in (1, -1)], 5.2)
    plate = plate.cut(cq.Workplane("XY").box(x_br - x_win + 2, 50, 2 * win)
                      .translate(((x_win + x_br + 2) / 2, ADAPTER_T / 2, 0)))
    shapes["fore_plate_J3"] = (plate.translate((0, y_face - ADAPTER_T, 0)), COL["plate"])
    # 연결 브라켓 (YZ 판): 전완판 바깥면 → J4 하우징 플랜지 (+Y 45 까지), J4 모터 창
    y0, y1 = y_face - ADAPTER_T, 45.0
    bracket = (cq.Workplane("XY").box(PLATE_T, y1 - y0, 90)
               .translate((x_br + PLATE_T / 2, (y0 + y1) / 2, 0))
               .cut(cq.Workplane("XY").box(PLATE_T + 2, 2 * win, 2 * win).translate((x_br + PLATE_T / 2, 0, 0))))
    shapes["fore_offset_bracket"] = (bracket, COL["plate"])
    for n, (o, c) in build_wrist(w).items():
        shapes[n] = (o, c)
    return shapes


def build_arm(j2_adapter: bool = True) -> cq.Assembly:
    """팔 조립. j2_adapter=False 면 J2 어댑터를 빼고 (J1 브라켓이 대신) 조립한다."""
    p2, p3 = ShoulderParams(), elbow_params()
    asm = cq.Assembly(name="arm_J2_J3")

    # J2: 출력이 −Y. 모터 전면 Z=0 → 회전 후 Y=0 평면. 출력 플랜지 상면 Y = −stack
    for n, (o, c) in _joint_shapes(p2, +90.0, (0, 0, 0)).items():
        if n == "adapter" and not j2_adapter:
            continue
        asm.add(o, name=f"J2_{n}", color=c)
    # 상완 측판은 J2 플랜지 상면에 밀착 (−Y 쪽으로 두께)
    y_plate = upper_plate_y(p2)
    for n, (o, c) in make_upper_arm(p2, p3, y_plate).items():
        asm.add(o, name=n, color=c)
    # J3: J2 와 같은 방향 — 출력 플랜지 상면이 상완 J3 측판 +Y 면에 밀착
    y_j3_face = y_plate + PLATE_T + joint_stack_length(p3)     # J3 모터 전면
    for n, (o, c) in _joint_shapes(p3, +90.0, (UPPER_LEN, y_j3_face, 0)).items():
        if n == "adapter":                                      # 전완판이 대신
            continue
        asm.add(o, name=f"J3_{n}", color=c)
    for n, (o, c) in make_forearm(p3, y_j3_face).items():
        asm.add(o, name=n, color=c)
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
