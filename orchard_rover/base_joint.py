"""J1 베이스 요 조인트 + 구동체 마운트, 전체 로봇(구동체 + 팔) 조립.

구동체 좌표계 사용 (X 전방, Y 좌측, Z 상방, 지면 Z=0).
J1 축 = Z, 위치 X=+300 (차동 바 스윙 영역 X=−150±120 회피), 상단 레일 위.

적층 (아래→위):
  상단 레일 상면 (Z=720)
  ├─ 베이스 플레이트 12t (레일 2본 + 크로스멤버 2본에 M8 T-너트)
  │    └─ 아래로: NEMA23 + 1:15 유성 감속기 (프레임 내부로 매달림)
  ├─ 크로스롤러 베어링 (ID 80 / OD 120 / W 16), 외륜 고정
  └─ 터릿 플레이트 12t (내륜 회전) ┬ J2 브라켓 (수직 10t, J2 어댑터 겸용) → J2 축 Z=950
                                     └ 가스 스프링 클레비스 (J2 축 아래 180, 상완 측판 바깥 평면)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Tuple

import cadquery as cq

from orchard_rover.arm_assembly import _cut_y, build_arm, gas_spring_y
from orchard_rover.rocker_bogie_cad import build_assembly as build_rover
from orchard_rover.rocker_bogie_params import SuspensionParams
from orchard_rover.shoulder_joint import (ADAPTER_T, ShoulderParams, adapter_radius, gas_spring_arm_anchor,
                                          make_gas_spring)


@dataclass
class BaseJointParams:
    x_mount: float = 300.0
    plate_t: float = 12.0
    plate_x: float = 200.0
    plate_y: float = 820.0          # 좌우 상단 레일 (Y ±380 중심) 을 덮음
    crossmember_dx: float = 60.0    # 크로스멤버 중심 X 오프셋 (±)
    bearing: Tuple[float, float, float] = (80.0, 120.0, 16.0)   # 크로스롤러 ID/OD/W
    turret_size: float = 180.0
    turret_t: float = 12.0
    bracket_t: float = ADAPTER_T    # J2 사이클로이드 어댑터 자리를 겸함
    bracket_w: float = 180.0        # J2 링 하우징 Ø165 체결
    gusset_t: float = 12.0
    gusset_depth: float = 60.0      # 판 +Y 면에서의 깊이 (바닥)
    gusset_h: float = 240.0         # 높이 (터릿 상면에서) — J2 축(200) 위까지, 상단 R90 윤곽 안 (거셋 X ±78~90)
    shoulder_rise: float = 200.0    # 터릿 상면 → J2 축
    gearbox_d: float = 57.0
    gearbox_len: float = 60.0       # 1:15 유성 (NEMA23 플랜지형)
    motor_len: float = 76.0
    clevis_gap: float = 12.5        # 가스 스프링 아이 폭 12 + 여유
    clevis_t: float = 6.0


def z_rail_top(p: SuspensionParams) -> float:
    return p.z_frame_top_rail + p.frame_profile / 2


def base_plate_bolts(b: BaseJointParams):
    """베이스 플레이트 M5 체결점 (플레이트 중심 기준 XY).

    좌우 상단 레일(중심 Y ±380)은 안쪽 홈 Y ±370, 크로스멤버(중심 X ±60)는 바깥 홈 X ±70.
    """
    slot = SuspensionParams().hfs5_slot_offset
    rail_y = 380.0 - slot
    cm_x = b.crossmember_dx + slot
    pts = [(x, y) for x in (-70.0, 0.0, 70.0) for y in (rail_y, -rail_y)]
    pts += [(x, y) for x in (-cm_x, cm_x) for y in (-250.0, -120.0, 120.0, 250.0)]
    return pts


def make_base_plate(b: BaseJointParams) -> cq.Workplane:
    """베이스 플레이트: 중앙 감속기 출력 관통, 베어링 외륜 자리, 레일·크로스멤버 체결 홀.

    HFS5-4040 상면 홈 2줄(중심 ±10)에 맞춘 M5 (Ø5.5) — 시리즈 5 T너트(HNTT5)는 M5 까지.
    """
    pl = (cq.Workplane("XY").rect(b.plate_x, b.plate_y).extrude(b.plate_t).edges("|Z").fillet(10))
    pl = pl.faces(">Z").workplane().circle(b.gearbox_d / 2 + 1).cutThruAll()
    # 베어링 외륜 받침 턱 (OD + 0.02 포켓, 깊이 4)
    pl = pl.faces(">Z").workplane().circle(b.bearing[1] / 2 + 0.01).cutBlind(-4)
    pl = pl.faces(">Z").workplane().pushPoints(base_plate_bolts(b)).circle(2.75).cutThruAll()
    # 베어링 외륜 체결 M5 ×8 @PCD (OD−10)
    pcd = (b.bearing[1] - 10) / 2
    pl = pl.faces(">Z").workplane().pushPoints(
        [(pcd * math.cos(a), pcd * math.sin(a)) for a in [k * math.pi / 4 for k in range(8)]]
    ).circle(2.2).cutThruAll()
    return pl


def make_turret(b: BaseJointParams) -> cq.Workplane:
    """터릿 플레이트: 베어링 내륜 체결, 중앙 허브(감속기 출력 Ø14 키), J2 브라켓 체결."""
    t = (cq.Workplane("XY").rect(b.turret_size, b.turret_size).extrude(b.turret_t)
         .edges("|Z").fillet(15))
    hub = cq.Workplane("XY").circle(b.bearing[0] / 2 - 0.01).extrude(b.bearing[2] + 2).translate((0, 0, -b.bearing[2] - 2))
    t = t.union(hub)
    t = t.faces(">Z").workplane().circle(7.05).cutThruAll()     # 감속기 출력축 Ø14
    pcd = (b.bearing[0] + 10) / 2
    t = t.faces(">Z").workplane().pushPoints(
        [(pcd * math.cos(a), pcd * math.sin(a)) for a in [k * math.pi / 4 + math.pi / 8 for k in range(8)]]
    ).circle(2.2).cutThruAll()
    return t


def make_bracket_gusset(b: BaseJointParams) -> cq.Workplane:
    """J2 브라켓 거셋 1장: YZ 평면 삼각 (깊이 Y gusset_depth × 높이 Z gusset_h), 두께 X 0→gusset_t."""
    return (cq.Workplane("YZ").polyline([(0, 0), (b.gusset_depth, 0), (0, b.gusset_h)]).close()
            .extrude(b.gusset_t))                                                 # YZ 압출 = +X


def make_shoulder_bracket(b: BaseJointParams, p2: ShoulderParams) -> cq.Workplane:
    """J2 브라켓 겸 사이클로이드 어댑터: 수직 판 (XZ 평면, Y ∈ [−bracket_t, 0]), 거셋, 바닥 플랜지.

    터릿 상면 Z=0 기준. J2 축은 (0, 0, shoulder_rise). 판이 J2 어댑터 자리를 차지하여
    +Y 면에 NEMA23 (보스 Ø38.1, M5 ×4), −Y 면에 링 하우징이 체결된다.
    """
    zc = b.shoulder_rise
    assert b.bracket_w / 2 >= adapter_radius(p2), "브라켓이 J2 링 하우징을 덮지 못함"
    pl = (cq.Workplane("XZ").center(0, zc / 2).rect(b.bracket_w, zc).extrude(-b.bracket_t)
          .union(cq.Workplane("XZ").center(0, zc).circle(b.bracket_w / 2).extrude(-b.bracket_t))
          .translate((0, -b.bracket_t, 0)))
    pl = _cut_y(pl, [(0, zc)], 40.0)                                           # 모터 보스 Ø38.1
    pl = _cut_y(pl, [(sx * 23.57, zc + sz * 23.57) for sx in (1, -1) for sz in (1, -1)], 5.2)
    # 거셋 (양측, 수직 삼각) — 판 +Y 면(모터 측), 면외 강성 보강
    g = make_bracket_gusset(b)
    gL = g.translate((b.bracket_w / 2 - b.gusset_t, 0, 0))
    gR = g.translate((-b.bracket_w / 2, 0, 0))
    # 바닥 플랜지 (터릿 체결 M6 ×4), Y ∈ [0, 60]
    foot = cq.Workplane("XY").rect(b.bracket_w, 60).extrude(b.gusset_t).translate((0, 30, 0))
    foot = foot.faces(">Z").workplane().pushPoints([(-60, 10), (60, 10), (-60, -10), (60, -10)]).circle(3.3).cutThruAll()
    return pl.union(gL).union(gR).union(foot)


def make_gs_clevis(b: BaseJointParams, pin_z: float) -> cq.Workplane:
    """가스 스프링 베이스 클레비스: 받침판 6t + 치크 2장 (핀 Y 방향, M8) — 터릿 상면 Z=0, 핀 높이 pin_z."""
    half = b.clevis_gap / 2 + b.clevis_t
    base = cq.Workplane("XY").box(40, 2 * half, 6, centered=(True, True, False))
    cl = base
    for s in (1, -1):
        y_in = s * b.clevis_gap / 2
        cheek = (cq.Workplane("XZ").center(0, pin_z / 2).rect(24, pin_z).extrude(-b.clevis_t)
                 .union(cq.Workplane("XZ").center(0, pin_z).circle(12).extrude(-b.clevis_t)))
        cl = cl.union(cheek.translate((0, y_in if s > 0 else y_in - b.clevis_t, 0)))
    cl = _cut_y(cl, [(0, pin_z)], 8.2)
    pin = cq.Workplane("XZ").center(0, pin_z).circle(4).extrude(half + 2, both=True)
    return cl.union(pin)


def make_j1_drive(b: BaseJointParams) -> cq.Workplane:
    """NEMA23 + 플랜지형 유성 감속기 (Z 아래로 매달림, 출력축 위로)."""
    gb = cq.Workplane("XY").circle(b.gearbox_d / 2).extrude(-b.gearbox_len)
    motor = (cq.Workplane("XY").rect(57, 57).extrude(-b.motor_len).edges("|Z").chamfer(5)
             .translate((0, 0, -b.gearbox_len)))
    # 출력축 상단 = 터릿 상면 (build_j1 의 z_turret + turret_t 와 동일 적층)
    shaft = cq.Workplane("XY").circle(7).extrude(b.plate_t - 4 + b.bearing[2] + 2 + b.turret_t)
    return gb.union(motor).union(shaft)


def build_j1(b: BaseJointParams, p: SuspensionParams, p2: ShoulderParams) -> Tuple[cq.Assembly, float]:
    """J1 조립체 (구동체 좌표). 반환: (조립체, J2 축 Z)."""
    z0 = z_rail_top(p)
    x = b.x_mount
    asm = cq.Assembly(name="J1_base")
    asm.add(make_base_plate(b).translate((x, 0, z0)), name="base_plate", color=cq.Color(0.85, 0.3, 0.3))
    # 크로스멤버 2본 (HFS5-4040)
    for dx in (-b.crossmember_dx, b.crossmember_dx):
        cm = cq.Workplane("XY").box(40, 720, 40).translate((x + dx, 0, p.z_frame_top_rail))
        asm.add(cm, name=f"crossmember_{'f' if dx > 0 else 'r'}", color=cq.Color(0.55, 0.55, 0.58))
    asm.add(make_j1_drive(b).translate((x, 0, z0)), name="j1_drive", color=cq.Color(0.2, 0.2, 0.25))
    ib, ob, wb = b.bearing
    brg = cq.Workplane("XY").circle(ob / 2).circle(ib / 2).extrude(wb).translate((x, 0, z0 + b.plate_t - 4))
    asm.add(brg, name="cross_roller", color=cq.Color(0.8, 0.8, 0.8))
    z_turret = z0 + b.plate_t - 4 + wb + 2
    asm.add(make_turret(b).translate((x, 0, z_turret)), name="turret", color=cq.Color(0.85, 0.72, 0.1))
    z_top = z_turret + b.turret_t
    asm.add(make_shoulder_bracket(b, p2).translate((x, 0, z_top)), name="shoulder_bracket",
            color=cq.Color(0.85, 0.3, 0.3))
    # 가스 스프링 베이스 클레비스: 팔 좌표 gs_base (J2 축 기준), 팔 가스 스프링 평면
    bx, bz = p2.gs_base
    asm.add(make_gs_clevis(b, b.shoulder_rise + bz).translate((x + bx, gas_spring_y(p2), z_top)),
            name="gs_clevis", color=cq.Color(0.4, 0.4, 0.45))
    return asm, z_top + b.shoulder_rise


def gas_spring_home(p2: ShoulderParams, origin: Tuple[float, float, float]) -> cq.Workplane:
    """수평 자세 가스 스프링: 베이스 앵커 → 상완 앵커, 팔 좌표 origin(J2 축) 기준."""
    (bx, bz), (ax, az) = p2.gs_base, gas_spring_arm_anchor(p2, 0.0)
    L = math.hypot(ax - bx, az - bz)
    elev = math.degrees(math.atan2(az - bz, ax - bx))
    # +Y 축 회전은 +X 를 −Z 로 보내므로 들어 올리려면 음의 각
    ox, oy, oz = origin
    return (make_gas_spring(L, p2.gs_spec).rotate((0, 0, 0), (0, 1, 0), -elev)
            .translate((ox + bx, oy + gas_spring_y(p2), oz + bz)))


def build_full_robot(theta_left: float = 0.0) -> Tuple[cq.Assembly, Dict[str, float]]:
    """구동체 + J1 + 팔 전체 조립."""
    p = SuspensionParams()
    b = BaseJointParams()
    p2 = ShoulderParams()
    rover = build_rover(p, theta_left)
    j1, z_j2 = build_j1(b, p, p2)
    arm = build_arm(j2_adapter=False)      # J1 브라켓이 J2 어댑터 자리를 대신
    full = cq.Assembly(name="orchard_robot")
    full.add(rover, name="rover")
    full.add(j1, name="J1")
    # 팔 좌표계 원점(J2 축) → 구동체 (x_mount, 0, z_j2)
    full.add(arm, name="arm", loc=cq.Location(cq.Vector(b.x_mount, 0, z_j2)))
    full.add(gas_spring_home(p2, (b.x_mount, 0.0, z_j2)), name="gas_spring", color=cq.Color(0.2, 0.7, 0.3))
    info = {"z_j2_axis": z_j2, "z_rail_top": z_rail_top(p), "x_mount": b.x_mount,
            "tip_x_home": b.x_mount + 950.0, "tip_z_home": z_j2}
    return full, info


def export(out: Path) -> Dict[str, float]:
    out.mkdir(parents=True, exist_ok=True)
    full, info = build_full_robot(0.0)
    full.save(str(out / "orchard_robot_full.step"))
    comp = cq.Workplane().add(full.toCompound())
    views = {"iso": (1, -1.2, 0.8), "side": (0, -1, 0), "front": (1, 0, 0)}
    for n, d in views.items():
        cq.exporters.export(comp, str(out / f"robot_{n}.svg"),
                            opt={"width": 1600, "height": 1000, "projectionDir": d,
                                 "showHidden": False, "strokeWidth": 0.5})
    try:
        import cairosvg
        from PIL import Image, ImageOps
        for n in views:
            svg, png = out / f"robot_{n}.svg", out / f"robot_{n}.png"
            cairosvg.svg2png(url=str(svg), write_to=str(png), output_width=1600, background_color="white")
            img = ImageOps.flip(Image.open(png))
            if n == "side":
                img = img.transpose(Image.Transpose.TRANSPOSE)
            elif n == "front":
                img = img.transpose(Image.Transpose.TRANSVERSE)
            img.save(png)
    except ImportError:
        pass
    return info


if __name__ == "__main__":
    for k, v in export(Path(__file__).resolve().parents[1] / "output" / "robot").items():
        print(f"{k:14s}: {v:8.1f}")
