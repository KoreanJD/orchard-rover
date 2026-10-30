"""Rocker-Bogie 서스펜션 CadQuery 모델 생성기.

생성물 (output/ 디렉터리):
    parts/*.step        : 개별 부품 STEP
    rocker_bogie_assembly.step / .stl : 전체 조립체 (프레임 참조 포함)
    dxf/rocker_plate.dxf, dxf/bogie_plate.dxf : 워터젯/레이저 가공용 2D 외형
    rocker_bogie_iso.svg / .png, _side.svg : 검토용 투영 뷰

좌표계: X 전방, Y 좌측, Z 상방 (지면 Z=0). 모든 치수 mm.

사용법:
    python rocker_bogie_cad.py [--pose THETA_LEFT_DEG]
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Dict, Tuple

import cadquery as cq

from orchard_rover.rocker_bogie_params import BEARINGS, WHEELS, SuspensionParams

Point2 = Tuple[float, float]

# 조립체 색상 (검토 편의)
COLORS: Dict[str, cq.Color] = {
    "frame": cq.Color(0.55, 0.55, 0.58),
    "rocker": cq.Color(0.85, 0.25, 0.25),
    "bogie": cq.Color(0.95, 0.55, 0.15),
    "wheel": cq.Color(0.15, 0.15, 0.15),
    "pivot": cq.Color(0.85, 0.72, 0.1),
    "diff": cq.Color(0.1, 0.5, 0.85),
    "link": cq.Color(0.2, 0.75, 0.3),
}


# ---------------------------------------------------------------------------
# 2D 플레이트 유틸리티 (XZ 평면 작업, 두께는 -Y 방향으로 압출)
# ---------------------------------------------------------------------------
def _bar(wp: cq.Workplane, a: Point2, b: Point2, w: float) -> cq.Workplane:
    """두 점 a→b 를 잇는 폭 w 의 둥근 막대(슬롯) 2D 스케치를 추가한다."""
    mx, mz = 0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    ang = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
    # slot2D 의 length 는 양 끝 반원을 포함한 전체 길이
    return wp.center(mx, mz).slot2D(length + w, w, ang).center(-mx, -mz)


def _plate_from_bars(segments, w: float, t: float, bosses) -> cq.Workplane:
    """여러 막대 세그먼트와 원형 보스를 합쳐 두께 t 의 플레이트를 만든다.

    Args:
        segments: [(a, b), ...] 측면도 좌표 쌍.
        w: 막대 폭.
        t: 플레이트 두께 (Y 방향, 0 → -t 로 압출).
        bosses: [(center, diameter), ...] 조인트 보강 원판.
    """
    solid = None
    for a, b in segments:
        part = _bar(cq.Workplane("XZ"), a, b, w).extrude(t)
        solid = part if solid is None else solid.union(part)
    for c, d in bosses:
        part = cq.Workplane("XZ").center(*c).circle(d / 2).extrude(t)
        solid = solid.union(part)
    return solid.clean()


def _hole(solid: cq.Workplane, c: Point2, d: float, depth: float = 500.0) -> cq.Workplane:
    """XZ 평면상 점 c 에 Y 방향 관통 구멍(지름 d)을 뚫는다."""
    cutter = (cq.Workplane("XZ").center(*c).circle(d / 2)
              .extrude(depth, both=True))
    return solid.cut(cutter)


def _place_y(solid: cq.Workplane, y_in: float, t: float) -> cq.Workplane:
    """XZ 에서 -Y 로 압출된 솔리드를 Y 구간 [y_in, y_in + t] 로 이동한다."""
    return solid.translate((0, y_in + t, 0))


# ---------------------------------------------------------------------------
# 개별 부품
# ---------------------------------------------------------------------------
def make_rocker_plate(p: SuspensionParams) -> cq.Workplane:
    """로커 플레이트 (Al 6061-T6, 두께 plate_t).

    P(차체 피벗) 에서 F(전방 휠), B(보기 피벗), U(차동 링크) 로 뻗는 3-암 플레이트.
    P 에는 6004ZZ ×2 베어링 포켓 보스, F 에는 허브모터 M12 축 홀, B 에는 Ø12 축 압입 홀.
    """
    pts = p.side_points()
    w, t = p.arm_w, p.plate_t
    brg = BEARINGS[p.rocker_journal_d]
    segs = [(pts["P"], pts["F"]), (pts["P"], pts["B"]), (pts["P"], pts["U"])]
    bosses = [(pts["P"], brg["od"] + 20), (pts["F"], w), (pts["B"], w), (pts["U"], w)]
    plate = _plate_from_bars(segs, w, t, bosses)

    # 베어링 하우징 보스: 플레이트 외측(+Y 방향은 배치 후)으로 2×베어링 폭 확보
    boss_len = 2 * brg["width"] + 6 - t
    boss = (cq.Workplane("XZ").center(*pts["P"]).circle(brg["od"] / 2 + 8)
            .extrude(-boss_len))                     # +Y 쪽으로 돌출
    plate = plate.union(boss)
    plate = _hole(plate, pts["P"], brg["od"] - 0.01)   # 베어링 압입 포켓 (H7)
    plate = _hole(plate, pts["F"], p.front_axle_d + 0.05)   # 허브모터 축 (드롭아웃은 모터 사양 확정 후)
    plate = _hole(plate, pts["B"], p.bogie_shaft_d - 0.02)  # 보기 축 압입
    plate = _hole(plate, pts["U"], 8.2)                 # 로드엔드 M8 볼트
    plate = _hole(plate, fork_standoff_point(p), 10.2)  # 외측 포크 스탠드오프 M10
    # 경량화 홀 (전방 암)
    px, pz = pts["P"]
    fx, fz = pts["F"]
    for s in (0.3, 0.5, 0.7):
        c = (px + s * (fx - px), pz + s * (fz - pz))
        plate = _hole(plate, c, w * 0.45)
    return plate


def make_bogie_plate(p: SuspensionParams) -> cq.Workplane:
    """보기 플레이트 — B(피벗) 에서 M(중간 휠), R(후방 휠) 로 뻗는 V자 플레이트."""
    pts = p.side_points()
    w, t = p.arm_w, p.plate_t
    brg = BEARINGS[p.bogie_shaft_d]
    segs = [(pts["B"], pts["M"]), (pts["B"], pts["R"])]
    bosses = [(pts["B"], brg["od"] + 16), (pts["M"], w), (pts["R"], w)]
    plate = _plate_from_bars(segs, w, t, bosses)
    boss_len = 2 * brg["width"] + 4 - t
    boss = (cq.Workplane("XZ").center(*pts["B"]).circle(brg["od"] / 2 + 6)
            .extrude(-boss_len))
    plate = plate.union(boss)
    plate = _hole(plate, pts["B"], brg["od"] - 0.01)
    plate = _hole(plate, pts["M"], p.axle_d + 0.05)
    plate = _hole(plate, pts["R"], p.axle_d + 0.05)
    return plate


def make_pivot_block(p: SuspensionParams) -> cq.Workplane:
    """로커 피벗 서포트 — MISUMI SHF20 플랜지형 샤프트 홀더 (주조품, 단순화 형상).

    L60 × t8 플랜지 + 보스 (축방향 전체 T20), 볼트 홀 Ø7 ×2 (간격 48, X 방향, 축 중심 높이)
    → HFS5-4040 측면 하단 레일 외측면의 아래 홈(Z 300)에 M5 + 와셔 + HNTT5-5 ×2 로 체결.
    Ø20 축은 M5 클램프 스크류로 고정 (축 고정, 로커가 베어링으로 회전).
    """
    t, boss, w, pitch = p.shf_flange_t, p.shf_boss_len, p.shf_flange_w, p.shf_hole_pitch
    h, r_hub = p.shf_flange_h, p.shf_boss_d / 2
    flange = (cq.Workplane("XZ").rect(w, h).extrude(-t)       # Y 0→+t
              .edges("|Y").fillet(8))
    hub = cq.Workplane("XZ").circle(r_hub).extrude(-(t + boss))
    blk = flange.union(hub)
    blk = blk.cut(cq.Workplane("XZ").circle(p.rocker_shaft_d / 2 + 0.01).extrude(t + boss + 2, both=True))
    blk = blk.cut(cq.Workplane("XZ").pushPoints([(-pitch / 2, 0), (pitch / 2, 0)])
                  .circle(p.shf_hole_d / 2).extrude(t + boss + 2, both=True))
    # 클램프 슬릿 (하단, 폭 2) — 축 구멍부터 보스 밖까지
    slit = (cq.Workplane("XZ").center(0, -(p.rocker_shaft_d / 2 + r_hub) / 2)
            .rect(2, r_hub - p.rocker_shaft_d / 2 + 2).extrude(-(t + boss)))
    return blk.cut(slit)


def make_shaft_collar(p: SuspensionParams) -> cq.Workplane:
    """세트 칼라 PSCS20-12 (슬릿, Ø20 축 빠짐 방지)."""
    return (cq.Workplane("XZ").circle(p.collar_od / 2).circle(p.rocker_shaft_d / 2 + 0.01)
            .extrude(-p.collar_w))


def make_rocker_shaft(p: SuspensionParams) -> cq.Workplane:
    """로커 피벗 축 — SHF20 클램프 | 6004ZZ ×2 저널 | PSCS20 칼라 (저널경 = 축경이면 직선축).

    저널이 굵으면 단붙이: 베어링은 바깥 끝으로 넘겨 끼우고 안쪽 단차가 내측 베어링 내륜을 받친다.
    """
    brg = BEARINGS[p.rocker_journal_d]
    l_in = p.pivot_block_t + p.block_gap
    l_brg = 2 * brg["width"] + 6
    length = l_in + l_brg + p.collar_w + 2
    shaft = cq.Workplane("XZ").circle(p.rocker_shaft_d / 2).extrude(-length)
    journal = (cq.Workplane("XZ").circle(p.rocker_journal_d / 2).extrude(-l_brg)
               .translate((0, l_in, 0)))
    return shaft.union(journal)


def make_bogie_shaft(p: SuspensionParams) -> cq.Workplane:
    """보기 피벗 축 SFJ12 — 로커 플레이트 압입 + 보기 베어링 보스."""
    brg = BEARINGS[p.bogie_shaft_d]
    length = p.plate_t + p.plate_gap + 2 * brg["width"] + 4 + 10
    return cq.Workplane("XZ").circle(p.bogie_shaft_d / 2).extrude(-length)


def make_bearing_pair(bore: float, gap: float, plane: str = "XZ") -> cq.Workplane:
    """깊은홈 볼베어링 2개 (내경 bore, 사이 간격 gap) — 축 방향 0 → 2W+gap.

    외경은 하우징 압입 포켓(OD − 0.01)과 같게 두어 모델상 접촉으로 표현한다.
    plane "XZ" 는 +Y 방향, "XY" 는 +Z 방향으로 쌓는다.
    """
    brg = BEARINGS[bore]
    out = None
    for k in range(2):
        ring = cq.Workplane(plane).circle((brg["od"] - 0.01) / 2).circle(bore / 2)
        ring = ring.extrude(-brg["width"] if plane == "XZ" else brg["width"])
        off = k * (brg["width"] + gap)
        ring = ring.translate((0, off, 0) if plane == "XZ" else (0, 0, off))
        out = ring if out is None else out.union(ring)
    return out


def make_axle(p: SuspensionParams, overhang: float) -> cq.Workplane:
    """휠 스터브 액슬 SFJ16 — 플레이트 내측면에서 휠 외측면까지."""
    length = overhang + WHEELS[p.wheel_d]["width"] / 2 + 8
    return cq.Workplane("XZ").circle(p.axle_d / 2).extrude(-length)


def make_front_axle(p: SuspensionParams) -> cq.Workplane:
    """전방 허브모터 고정축 M12 — 로커 플레이트 내측면에서 외측 포크 외측면까지 (양쪽 지지)."""
    d = p.derived
    length = d["y_fork_in"] + p.plate_t - d["y_rocker_in"]
    return cq.Workplane("XZ").circle(p.front_axle_d / 2).extrude(-length)


def make_front_fork(p: SuspensionParams) -> cq.Workplane:
    """전방 외측 포크 — 축 F 에서 로커 피벗 방향으로 fork_len 만큼, 로커 전방 암과 평행한 짧은 암.

    F 에 허브모터 축 홀, 끝(S)에 스탠드오프 볼트 M10 홀. XZ 압출(+plate_t)은 −Y 방향 (_place_y 로 배치).
    """
    pts = p.side_points()
    s = fork_standoff_point(p)
    w = p.arm_w
    fork = _plate_from_bars([(pts["F"], s)], w, p.plate_t, [(pts["F"], w), (s, w)])
    fork = _hole(fork, pts["F"], p.front_axle_d + 0.05)
    return _hole(fork, s, 10.2)


def fork_standoff_point(p: SuspensionParams) -> Point2:
    """외측 포크 ↔ 로커 플레이트 스탠드오프 위치 (XZ): F 에서 P 방향 fork_len."""
    pts = p.side_points()
    (fx, fz), (px, pz) = pts["F"], pts["P"]
    L = math.hypot(px - fx, pz - fz)
    return (fx + (px - fx) * p.fork_len / L, fz + (pz - fz) * p.fork_len / L)


def make_standoff(p: SuspensionParams) -> cq.Workplane:
    """포크 스탠드오프 Ø20 (M10 관통 볼트) — 로커 플레이트 외측면 → 포크 내측면, +Y 방향."""
    d = p.derived
    length = d["y_fork_in"] - (d["y_rocker_in"] + p.plate_t)
    return (cq.Workplane("XZ").circle(p.standoff_d / 2).circle(5.1).extrude(-length))


def make_wheel(p: SuspensionParams) -> cq.Workplane:
    """UWRF 우레탄 휠 (단순화: 원통 + 허브 + 보어)."""
    spec = WHEELS[p.wheel_d]
    r, wdt = p.wheel_d / 2, spec["width"]
    wheel = (cq.Workplane("XZ").circle(r).extrude(-wdt)
             .faces(">Y").workplane().circle(r - 12).cutBlind(-5)
             .faces("<Y").workplane().circle(r - 12).cutBlind(-5))
    hub = cq.Workplane("XZ").circle(r - 25).extrude(-wdt)
    wheel = wheel.union(hub)
    bore = cq.Workplane("XZ").circle(spec["bore"] / 2 + 0.05).extrude(wdt + 2, both=True)
    return wheel.cut(bore)


def make_hub_motor_wheel(p: SuspensionParams) -> cq.Workplane:
    """전방 구동륜 — 10인치 타이어 + BLDC 허브 모터(스테이터 Ø160×60, 단순화)."""
    spec = WHEELS[p.wheel_d]
    wdt = spec["width"]
    wheel = make_wheel(p)
    motor = (cq.Workplane("XZ").circle(80).extrude(-60).translate((0, (wdt - 60) / 2, 0))
             .faces(">Y").workplane().circle(70).cutBlind(-3))
    # 스테이터 보어 = 고정축 M12 (+0.1)
    bore = cq.Workplane("XZ").circle(p.front_axle_d / 2 + 0.05).extrude(wdt + 2, both=True)
    return wheel.union(motor).cut(bore)


def make_diff_bar(p: SuspensionParams) -> cq.Workplane:
    """차동 바 — 중앙 Z축 피벗(6001ZZ ×2), 양단 M8 로드엔드 홀."""
    half = p.derived["diff_bar_half_len"]
    brg = BEARINGS[12]
    bar = (cq.Workplane("XY").slot2D(2 * half + p.diff_bar_w, p.diff_bar_w, 90)
           .extrude(p.diff_bar_t))
    hub = cq.Workplane("XY").circle(brg["od"] / 2 + 8).extrude(2 * brg["width"] + 4)
    bar = bar.union(hub)
    bar = bar.faces(">Z").workplane().circle(brg["od"] / 2 - 0.005).cutThruAll()
    bar = (bar.faces(">Z").workplane().pushPoints([(0, half), (0, -half)])
           .circle(4.1).cutThruAll())
    return bar.translate((0, 0, -p.diff_bar_t / 2))


def make_diff_post(p: SuspensionParams) -> cq.Workplane:
    """차동 바 피벗 포스트 — 상부 크로스멤버(HFS5-4040, Y 방향) 위 M5 ×4, Ø12 수직 핀.

    체결 홀은 크로스멤버 상면 홈 2줄(X ±10)에 Y ±22 (시리즈 5 T너트는 M5 까지).
    """
    height = p.z_diff_bar - p.diff_bar_t / 2 - (p.z_frame_top_rail + p.frame_profile / 2)
    post = (cq.Workplane("XY").rect(80, 60).extrude(10)            # 베이스 플레이트
            .faces(">Z").workplane().circle(16).extrude(height - 10 - 2))
    slot = p.hfs5_slot_offset
    post = post.cut(cq.Workplane("XY").pushPoints([(sx * slot, sy * 22.0) for sx in (1, -1) for sy in (1, -1)])
                    .circle(2.75).extrude(12).translate((0, 0, -1)))
    pin = cq.Workplane("XY").circle(6).extrude(height + 2 * BEARINGS[12]["width"] + 8)
    return post.union(pin)


def make_link_rod(p: SuspensionParams) -> cq.Workplane:
    """차동 링크 로드 — Ø8 로드 + 양단 로드엔드(PHS8 상당, 단순화 구체)."""
    length = p.diff_link_len
    rod = (cq.Workplane("YZ").circle(p.link_rod_d / 2)
           .extrude(length - 24).translate((12, 0, 0)))
    ends = None
    for x in (0, length):
        e = cq.Workplane("XY").sphere(9).translate((x, 0, 0))
        ends = e if ends is None else ends.union(e)
    return rod.union(ends)


def make_frame_reference(p: SuspensionParams) -> cq.Workplane:
    """프레임 참조 — HFS5-4040 하부 사각 프레임 + 상부 사각 + 4 기둥 + 차동 크로스멤버.

    서스펜션 연결점 확인용이며 실제 프레임 상세(브라켓 등)는 별도 설계.
    """
    pr = p.frame_profile
    hw = p.frame_half_w - pr / 2            # 레일 중심 Y
    hl = p.frame_len / 2 - pr / 2           # 레일 중심 X
    zl, zt = p.z_frame_low_rail, p.z_frame_top_rail

    def rail_x(y: float, z: float, length: float) -> cq.Workplane:
        return cq.Workplane("XY").box(length, pr, pr).translate((0, y, z))

    def rail_y(x: float, z: float, length: float) -> cq.Workplane:
        return cq.Workplane("XY").box(pr, length, pr).translate((x, 0, z))

    frame = None
    for z in (zl, zt):
        for y in (hw, -hw):
            r = rail_x(y, z, p.frame_len)
            frame = r if frame is None else frame.union(r)
        for x in (hl, -hl):
            frame = frame.union(rail_y(x, z, 2 * hw - pr))
    for x in (hl, -hl):
        for y in (hw, -hw):
            post = cq.Workplane("XY").box(pr, pr, zt - zl - pr).translate(
                (x, y, 0.5 * (zl + zt)))
            frame = frame.union(post)
    # 차동 포스트용 상부 크로스멤버
    frame = frame.union(rail_y(p.derived["x_diff_pivot"], zt, 2 * hw - pr))
    return frame.clean()


# ---------------------------------------------------------------------------
# 조립
# ---------------------------------------------------------------------------
def _rot_y(obj: cq.Workplane, cx: float, cz: float, theta: float) -> cq.Workplane:
    """점 (cx, *, cz) 를 지나는 Y축 기준으로 theta(rad, 전방 상승 +) 회전한다.

    우수 좌표계에서 +Y 축 회전은 전방(+X)을 아래로 내리므로 부호를 반전한다.
    """
    if abs(theta) < 1e-12:
        return obj
    return obj.rotate((cx, 0, cz), (cx, 1, cz), -math.degrees(theta))


def _side(p: SuspensionParams, side: float, theta: float,
          parts: Dict[str, cq.Workplane]) -> cq.Assembly:
    """한쪽(side=+1 좌, -1 우) 서스펜션 서브어셈블리를 로커 각 theta 로 배치한다.

    좌측 기준으로 월드 좌표에 형상을 배치·회전한 뒤, 우측은 XZ 평면 미러.
    평지 자세를 가정하여 보기는 로커 회전을 상쇄(지면 수평 유지)한다.
    """
    d = p.derived
    pts = p.side_points()
    px, pz = pts["P"]
    bx, bz = pts["B"]
    wdt = WHEELS[p.wheel_d]["width"]
    tag = "L" if side > 0 else "R"

    shapes: Dict[str, Tuple[cq.Workplane, cq.Color]] = {}

    sx, sz = fork_standoff_point(p)

    # --- 로커 그룹 (P 기준 회전) ---
    rocker_grp: Dict[str, Tuple[cq.Workplane, cq.Color]] = {
        "rocker_plate": (_place_y(parts["rocker_plate"], d["y_rocker_in"], p.plate_t), COLORS["rocker"]),
        "axle_F": (parts["axle_front"].translate((pts["F"][0], d["y_rocker_in"], pts["F"][1])), COLORS["pivot"]),
        "wheel_F": (parts["wheel_drive"].translate((pts["F"][0], d["y_wheel_in"], pts["F"][1])), COLORS["wheel"]),
        # 외측 포크 + 스탠드오프: 허브모터 축 양쪽 지지
        "fork_F": (_place_y(parts["front_fork"], d["y_fork_in"], p.plate_t), COLORS["rocker"]),
        "standoff_F": (parts["standoff"].translate((sx, d["y_rocker_in"] + p.plate_t, sz)), COLORS["pivot"]),
        "bogie_shaft": (parts["bogie_shaft"].translate((bx, d["y_rocker_in"], bz)), COLORS["pivot"]),
    }
    # --- 보기 그룹 (B 기준, 로커 회전 상쇄) ---
    bogie_grp: Dict[str, Tuple[cq.Workplane, cq.Color]] = {
        "bogie_plate": (_place_y(parts["bogie_plate"], d["y_bogie_in"], p.plate_t), COLORS["bogie"]),
        # 보기 보스 6001ZZ ×2 (간격 4) — 보기 축 위
        "bogie_bearing": (parts["bogie_bearing"].translate((bx, d["y_bogie_in"], bz)), COLORS["pivot"]),
    }
    for key in ("M", "R"):
        bogie_grp[f"axle_{key}"] = (
            parts["axle_mb"].translate((pts[key][0], d["y_bogie_in"], pts[key][1])), COLORS["pivot"])
        bogie_grp[f"wheel_{key}"] = (
            parts["wheel"].translate((pts[key][0], d["y_wheel_in"], pts[key][1])), COLORS["wheel"])

    for name, (obj, col) in bogie_grp.items():
        obj = _rot_y(obj, bx, bz, -theta)          # 보기: 로커 회전 상쇄
        rocker_grp[name] = (obj, col)
    for name, (obj, col) in rocker_grp.items():
        shapes[name] = (_rot_y(obj, px, pz, theta), col)

    # --- 프레임 고정부 ---
    shapes["pivot_block"] = (parts["pivot_block"].translate((px, p.frame_half_w, pz)), COLORS["frame"])
    shapes["rocker_shaft"] = (parts["rocker_shaft"].translate((px, p.frame_half_w, pz)), COLORS["pivot"])
    y_collar = d["y_rocker_in"] + p.plate_t + (2 * BEARINGS[p.rocker_journal_d]["width"] + 6 - p.plate_t) + 1
    shapes["shaft_collar"] = (parts["shaft_collar"].translate((px, y_collar, pz)), COLORS["pivot"])
    # 로커 보스 6004ZZ ×2 (간격 6) — 축 저널 위 (회전 대칭이라 로커 각과 무관)
    shapes["rocker_bearing"] = (parts["rocker_bearing"].translate((px, d["y_rocker_in"], pz)), COLORS["pivot"])

    sub = cq.Assembly(name=f"suspension_{tag}")
    for name, (obj, col) in shapes.items():
        if side < 0:
            obj = obj.mirror("XZ")
        sub.add(obj, name=f"{name}_{tag}", color=col)
    return sub


def build_assembly(p: SuspensionParams, theta_left: float = 0.0) -> cq.Assembly:
    """전체 서스펜션 조립체를 생성한다.

    Args:
        p: 설계 파라미터.
        theta_left: 좌측 로커 각도(rad). 우측 각도와 차동 바 각도는 기구학으로 결정.
    """
    phi, theta_right = (0.0, 0.0) if abs(theta_left) < 1e-9 else p.differential_response(theta_left)
    d = p.derived
    brg16, brg12 = BEARINGS[p.rocker_journal_d], BEARINGS[p.bogie_shaft_d]

    parts: Dict[str, cq.Workplane] = {
        "rocker_plate": make_rocker_plate(p),
        "bogie_plate": make_bogie_plate(p),
        "pivot_block": make_pivot_block(p),
        "shaft_collar": make_shaft_collar(p),
        "wheel": make_wheel(p),
        "wheel_drive": make_hub_motor_wheel(p),
        "diff_bar": make_diff_bar(p),
        "diff_post": make_diff_post(p),
        "link_rod": make_link_rod(p),
        "frame": make_frame_reference(p),
        "rocker_bearing": make_bearing_pair(p.rocker_journal_d, 6.0),
        "bogie_bearing": make_bearing_pair(p.bogie_shaft_d, 4.0),
        "front_fork": make_front_fork(p),
        "standoff": make_standoff(p),
    }
    parts["rocker_shaft"] = make_rocker_shaft(p)
    parts["rocker_shaft_len"] = p.pivot_block_t + 2 * brg16["width"] + 6 + 12
    parts["bogie_shaft"] = make_bogie_shaft(p)
    parts["bogie_shaft_len"] = p.plate_t + p.plate_gap + 2 * brg12["width"] + 4 + 10
    ov_f = d["y_wheel_c"] - d["y_rocker_in"]
    ov_mb = d["y_wheel_c"] - d["y_bogie_in"]
    parts["axle_front"] = make_front_axle(p)
    parts["axle_mb"] = make_axle(p, ov_mb)
    parts["axle_mb_len"] = ov_mb + WHEELS[p.wheel_d]["width"] / 2 + 8

    asm = cq.Assembly(name="rocker_bogie_suspension")
    asm.add(parts["frame"], name="frame_ref", color=COLORS["frame"])
    asm.add(_side(p, +1.0, theta_left, parts), name="left")
    asm.add(_side(p, -1.0, theta_right, parts), name="right")

    # 차동 바 + 포스트 + 링크
    xd = d["x_diff_pivot"]
    asm.add(parts["diff_post"].translate((xd, 0, p.z_frame_top_rail + p.frame_profile / 2)),
            name="diff_post", color=COLORS["frame"])
    asm.add(parts["diff_bar"], name="diff_bar", color=COLORS["diff"],
            loc=cq.Location(cq.Vector(xd, 0, p.z_diff_bar), cq.Vector(0, 0, 1), math.degrees(phi)))
    # 차동 바 허브 6001ZZ ×2 (간격 4) — 포스트 Ø12 핀 위, 허브 하단(바 하면)부터
    asm.add(make_bearing_pair(12, 4.0, "XY").translate((xd, 0, p.z_diff_bar - p.diff_bar_t / 2)),
            name="diff_bearing", color=COLORS["pivot"])
    for side, th in ((+1.0, theta_left), (-1.0, theta_right)):
        ux, uz = p.upright_top(th)
        uy = side * d["y_rocker_c"]
        half = d["diff_bar_half_len"]
        bx = xd - side * half * math.sin(phi)
        by = side * half * math.cos(phi)
        bz = p.z_diff_bar
        # 로드: 바 끝단 → 상방 암 끝 방향으로 정렬
        v = cq.Vector(ux - bx, uy - by, uz - bz)
        yaw = math.degrees(math.atan2(v.y, v.x))
        pitch = -math.degrees(math.atan2(v.z, math.hypot(v.x, v.y)))
        loc = (cq.Location(cq.Vector(bx, by, bz))
               * cq.Location(cq.Vector(), cq.Vector(0, 0, 1), yaw)
               * cq.Location(cq.Vector(), cq.Vector(0, 1, 0), pitch))
        asm.add(parts["link_rod"], name=f"link_{'L' if side > 0 else 'R'}",
                color=COLORS["link"], loc=loc)
    return asm


# ---------------------------------------------------------------------------
# 내보내기
# ---------------------------------------------------------------------------
def export_all(p: SuspensionParams, out: Path, theta_left_deg: float = 0.0) -> Dict[str, float]:
    """STEP/STL/DXF/SVG 를 out 디렉터리에 내보내고 부품 질량(kg)을 반환한다."""
    out.mkdir(parents=True, exist_ok=True)
    (out / "parts").mkdir(exist_ok=True)
    (out / "dxf").mkdir(exist_ok=True)

    singles = {
        "rocker_plate": make_rocker_plate(p),
        "bogie_plate": make_bogie_plate(p),
        "pivot_block": make_pivot_block(p),
        "shaft_collar": make_shaft_collar(p),
        "diff_bar": make_diff_bar(p),
        "diff_post": make_diff_post(p),
        "link_rod": make_link_rod(p),
        "front_fork": make_front_fork(p),
        "wheel_ref": make_wheel(p),
        "wheel_drive_ref": make_hub_motor_wheel(p),
    }
    masses: Dict[str, float] = {}
    for name, obj in singles.items():
        cq.exporters.export(obj, str(out / "parts" / f"{name}.step"))
        masses[name] = obj.val().Volume() / 1000.0 * p.density_g_cm3 / 1000.0

    # 2D 가공 도면: XZ 평면 플레이트 외형 → XY 로 회전 후 DXF
    for name in ("rocker_plate", "bogie_plate", "front_fork"):
        # X축 기준 -90° 회전: (x, y, z) → (x, z, -y) — 높이 Z 가 도면 +Y 로, 두께가 +Z 로
        flat = singles[name].rotate((0, 0, 0), (1, 0, 0), -90)
        # section() 은 작업평면 법선 기준이므로 XY 작업평면에 다시 올린다
        section = cq.Workplane("XY").add(flat.val()).section(p.plate_t / 2)   # 보스 제외, 홀 포함
        cq.exporters.exportDXF(section, str(out / "dxf" / f"{name}.dxf"))

    asm = build_assembly(p, math.radians(theta_left_deg))
    asm.save(str(out / "rocker_bogie_assembly.step"))
    compound = asm.toCompound()
    cq.exporters.export(cq.Workplane().add(compound), str(out / "rocker_bogie_assembly.stl"),
                        tolerance=0.5, angularTolerance=0.3)

    # CadQuery SVG 투영은 화면 상하가 반전되어 출력됨 → render_png 에서 보정
    view_shape = cq.Workplane().add(compound)
    views = {
        "iso": (1.0, -1.2, 0.8),
        "side": (0.0, -1.0, 0.0),
        "front": (1.0, 0.0, 0.0),
        "top": (0.0, 0.0, 1.0),
    }
    for vname, direction in views.items():
        cq.exporters.export(
            view_shape, str(out / f"rocker_bogie_{vname}.svg"),
            opt={"width": 1400, "height": 900, "projectionDir": direction,
                 "showHidden": False, "strokeWidth": 0.6, "marginLeft": 40, "marginTop": 40})
    return masses


def render_png(out: Path) -> None:
    """SVG 뷰를 PNG 로 변환한다 (cairosvg 필요)."""
    import cairosvg  # 지연 import: 선택 의존성

    from PIL import Image, ImageOps

    # 기준 형상으로 검증한 보정 (exporter 출력은 상하 반전, 축 정렬 뷰는 축이 뒤바뀜)
    #   iso  : 상하 반전만
    #   side : 상하 반전 후 전치   → Z 상방, +X(전방) 우측
    #   front: 상하 반전 후 역전치 → Z 상방, +Y(좌측) 우측 (전방에서 본 모습)
    #   top  : 원본 유지           → +X 우측, +Y 상방
    for svg in out.glob("rocker_bogie_*.svg"):
        png = svg.with_suffix(".png")
        cairosvg.svg2png(url=str(svg), write_to=str(png),
                         output_width=1400, background_color="white")
        img = Image.open(png)
        if svg.stem.endswith("_top"):
            pass
        elif svg.stem.endswith("_side"):
            img = ImageOps.flip(img).transpose(Image.Transpose.TRANSPOSE)
        elif svg.stem.endswith("_front"):
            img = ImageOps.flip(img).transpose(Image.Transpose.TRANSVERSE)
        else:
            img = ImageOps.flip(img)
        img.save(png)


def main() -> None:
    """CLI 진입점."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pose", type=float, default=0.0, help="좌측 로커 각도(deg)")
    ap.add_argument("--out", type=Path, default=Path(__file__).resolve().parents[1] / "output")
    args = ap.parse_args()
    p = SuspensionParams()
    masses = export_all(p, args.out, args.pose)
    for k, v in masses.items():
        print(f"{k:14s}: {v:6.3f} kg")
    try:
        render_png(args.out)
    except ImportError:
        print("cairosvg 미설치: PNG 변환 생략")


if __name__ == "__main__":
    main()
