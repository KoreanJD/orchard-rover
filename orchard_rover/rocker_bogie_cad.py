"""Rocker-Bogie 서스펜션 CadQuery 모델 생성기.

생성물 (output/ 디렉터리):
    parts/*.step        : 개별 부품 STEP
    rocker_bogie_assembly.step / .stl : 전체 조립체 (프레임 참조 포함)
    dxf/rocker_plate.dxf, dxf/bogie_plate.dxf, dxf/diff_bar.dxf : 워터젯/레이저 가공용 2D 외형
        (베어링 보스 없음 — MISUMI BGRBB 홀더를 판 구멍에 끼우고 볼트 체결)
    rocker_bogie_iso.svg / .png, _side.svg : 검토용 투영 뷰

좌표계: X 전방, Y 좌측, Z 상방 (지면 Z=0). 모든 치수 mm.

사용법:
    python rocker_bogie_cad.py [--pose THETA_LEFT_DEG]
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import cadquery as cq

from orchard_rover.rocker_bogie_params import BEARING_HOLDERS, BEARINGS, WHEELS, SuspensionParams

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


def holder_bolt_points(hold: Dict[str, float]) -> List[Point2]:
    """BGRBB 플랜지 볼트 4 홀 (P.C.D., 45° 배치 — 막대 방향과 어긋나게) 의 홀더 중심 기준 좌표."""
    r = hold["pcd"] / 2
    return [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for a in (45, 135, 225, 315)]


def _holder_seat(plate: cq.Workplane, c: Point2, hold: Dict[str, float]) -> cq.Workplane:
    """베어링 홀더 자리: 몸통 끼움 구멍 D (H7) + 플랜지 볼트 탭 4 (판 쪽 나사)."""
    plate = _hole(plate, c, hold["D"])
    for dx, dz in holder_bolt_points(hold):
        plate = _hole(plate, (c[0] + dx, c[1] + dz), hold["tap"])
    return plate


# ---------------------------------------------------------------------------
# 개별 부품
# ---------------------------------------------------------------------------
def make_rocker_plate(p: SuspensionParams) -> cq.Workplane:
    """로커 플레이트 (Al 6061-T6, 두께 plate_t).

    P(차체 피벗) 에서 F(전방 휠), B(보기 피벗), U(차동 링크) 로 뻗는 3-암 플레이트.
    P 에는 베어링 홀더(BGRBB6004DD) 자리, F 에는 허브모터 M12 축 홀, B 에는 Ø12 축 압입 홀.
    단일 두께 판 (워터젯 + 탭) — 베어링 보스는 상용 홀더를 볼트로 체결.
    """
    pts = p.side_points()
    w, t = p.arm_w, p.plate_t
    hold = BEARING_HOLDERS[p.rocker_journal_d]
    segs = [(pts["P"], pts["F"]), (pts["P"], pts["B"]), (pts["P"], pts["U"])]
    bosses = [(pts["P"], hold["H"] + 6), (pts["F"], w), (pts["B"], w), (pts["U"], w)]
    plate = _plate_from_bars(segs, w, t, bosses)
    plate = _holder_seat(plate, pts["P"], hold)
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
    hold = BEARING_HOLDERS[p.bogie_shaft_d]
    segs = [(pts["B"], pts["M"]), (pts["B"], pts["R"])]
    bosses = [(pts["B"], hold["H"] + 6), (pts["M"], w), (pts["R"], w)]
    plate = _plate_from_bars(segs, w, t, bosses)
    plate = _holder_seat(plate, pts["B"], hold)          # BGRBB6001DD (플랜지는 판 바깥면)
    for key in ("M", "R"):
        x, z = pts[key]
        plate = _hole(plate, (x, z), p.axle_d + 0.05)              # SFJ20 (SHF20 홀더가 클램프)
        for dz in (-p.shf_hole_pitch / 2, p.shf_hole_pitch / 2):   # 홀더 볼트 M6 탭 (드릴 Ø5.0, 세로 줄)
            plate = _hole(plate, (x, z + dz), 5.0)
    return plate


def make_pivot_block(p: SuspensionParams, bore: float = None) -> cq.Workplane:
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
    bore = p.rocker_shaft_d if bore is None else bore
    blk = blk.cut(cq.Workplane("XZ").circle(bore / 2 + 0.01).extrude(t + boss + 2, both=True))
    blk = blk.cut(cq.Workplane("XZ").pushPoints([(-pitch / 2, 0), (pitch / 2, 0)])
                  .circle(p.shf_hole_d / 2).extrude(t + boss + 2, both=True))
    # 클램프 슬릿 (하단, 폭 2) — 축 구멍부터 보스 밖까지
    slit = (cq.Workplane("XZ").center(0, -(bore / 2 + r_hub) / 2)
            .rect(2, r_hub - bore / 2 + 2).extrude(-(t + boss)))
    return blk.cut(slit)


def make_axle_holder(p: SuspensionParams) -> cq.Workplane:
    """아이들러 축 홀더 — SHF20 을 Y축 기준 90° 돌려 볼트 줄을 세로(Z ±24)로.

    휠 하중 모멘트(X축 둘레)를 볼트 인장으로 받는다. 보기 판 바깥면에 M6 ×2 (판 12t 에 탭).
    클램프 슬릿은 ±X 쪽 → 판과 휠 사이 틈으로 육각 렌치 접근.
    """
    return make_pivot_block(p, p.axle_d).rotate((0, 0, 0), (0, 1, 0), 90)


def make_spacer(p: SuspensionParams, length: float, bore: float = None, od: float = 28.0) -> cq.Workplane:
    """축 스페이서/와셔 (기본 Ø20/Ø28) — 베어링 내륜만 받친다 (+Y 방향 0 → length)."""
    bore = p.axle_d if bore is None else bore
    return cq.Workplane("XZ").circle(od / 2).circle(bore / 2 + 0.01).extrude(-length)


def make_bearing_holder(bore: int) -> cq.Workplane:
    """MISUMI BGRBB 베어링 홀더 (라운드 플랜지) — 플랜지 Y 0→T, 몸통 T→L (+Y).

    베어링 자리(OD − 0.01) 관통, 플랜지 볼트 4 홀은 플랜지 바깥면(Y 0)에서 카운터보어.
    베어링은 make_bearing_pair 로 따로 (양 끝에서 l 안쪽).
    """
    hold = BEARING_HOLDERS[bore]
    flange = cq.Workplane("XZ").circle(hold["H"] / 2).extrude(-hold["T"])
    body = cq.Workplane("XZ").circle(hold["D"] / 2 - 0.02).extrude(-hold["L"])
    h = flange.union(body).cut(
        cq.Workplane("XZ").circle((BEARINGS[bore]["od"] - 0.01) / 2).extrude(hold["L"] + 2, both=True))
    pts = holder_bolt_points(hold)
    h = h.cut(cq.Workplane("XZ").pushPoints(pts).circle(hold["hole"] / 2).extrude(hold["T"] + 2, both=True))
    return h.cut(cq.Workplane("XZ").pushPoints(pts).circle(hold["cb"] / 2).extrude(-hold["cb_h"]))


def holder_bearing_gap(bore: int) -> float:
    """홀더 안 베어링 2개 사이 간격 (양 끝에서 l 안쪽에 하나씩)."""
    hold = BEARING_HOLDERS[bore]
    return hold["L"] - 2 * hold["l"] - 2 * BEARINGS[bore]["width"]


def make_shaft_collar(p: SuspensionParams, bore: float = None, od: float = None,
                      width: float = None) -> cq.Workplane:
    """세트 칼라 (슬릿, 축 빠짐 방지) — 기본 PSCS20-12, 보기 축은 PSCS12-10 (+Y 방향 0 → width)."""
    bore = p.rocker_shaft_d if bore is None else bore   # 아이들러 축(SFJ20)에도 같은 칼라
    od = p.collar_od if od is None else od
    width = p.collar_w if width is None else width
    return cq.Workplane("XZ").circle(od / 2).circle(bore / 2 + 0.01).extrude(-width)


def rocker_shaft_length(p: SuspensionParams) -> float:
    """로커 피벗 축 길이: 레일 면 → SHF20 | 간극 | BGRBB6004 홀더 L | 1 | PSCS20 칼라 | 2."""
    return p.pivot_block_t + p.block_gap + BEARING_HOLDERS[p.rocker_journal_d]["L"] + 1 + p.collar_w + 2


def make_rocker_shaft(p: SuspensionParams) -> cq.Workplane:
    """로커 피벗 축 — SHF20 클램프 | 홀더(6004 ×2) 저널 | PSCS20 칼라 (저널경 = 축경이면 직선축).

    저널이 굵으면 단붙이: 베어링은 바깥 끝으로 넘겨 끼우고 안쪽 단차가 내측 베어링 내륜을 받친다.
    """
    l_in = p.pivot_block_t + p.block_gap
    l_brg = BEARING_HOLDERS[p.rocker_journal_d]["L"]
    length = rocker_shaft_length(p)
    shaft = cq.Workplane("XZ").circle(p.rocker_shaft_d / 2).extrude(-length)
    journal = (cq.Workplane("XZ").circle(p.rocker_journal_d / 2).extrude(-l_brg)
               .translate((0, l_in, 0)))
    return shaft.union(journal)


def bogie_shaft_length(p: SuspensionParams) -> float:
    """보기 축 길이: 로커 판 내측면 → 로커 판 | 간격 | 보기 판 | 홀더 플랜지 T | 1 | PSCS12-10 칼라 | 2."""
    return 2 * p.plate_t + p.plate_gap + BEARING_HOLDERS[p.bogie_shaft_d]["T"] + 1 + p.bogie_collar_w + 2


def make_bogie_shaft(p: SuspensionParams) -> cq.Workplane:
    """보기 피벗 축 SFJ12 — 로커 플레이트 압입 + 보기 베어링 홀더 (BGRBB6001DD) + 바깥 끝 PSCS12-10."""
    return cq.Workplane("XZ").circle(p.bogie_shaft_d / 2).extrude(-bogie_shaft_length(p))


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


def idler_axle_length(p: SuspensionParams) -> float:
    """SFJ20 아이들러 축 길이: 보기 판 내측면 → 판·홀더·스페이서·휠·와셔·PSCS20 칼라 + 2."""
    d = p.derived
    return (d["y_wheel_in"] - d["y_bogie_in"] + WHEELS[p.wheel_d]["width"]
            + p.idler_washer_t + p.collar_w + 2)


def make_axle(p: SuspensionParams) -> cq.Workplane:
    """아이들러 스터브 액슬 SFJ20 — 보기 판 내측면에서 칼라 바깥까지 (+Y)."""
    return cq.Workplane("XZ").circle(p.axle_d / 2).extrude(-idler_axle_length(p))


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


def _tire(p: SuspensionParams) -> cq.Workplane:
    """타이어 + 허브 (단순화: 원통 + 측면 리세스, 보어 없음) — +Y 방향 0 → 폭."""
    r, wdt = p.wheel_d / 2, WHEELS[p.wheel_d]["width"]
    wheel = (cq.Workplane("XZ").circle(r).extrude(-wdt)
             .faces(">Y").workplane().circle(r - 12).cutBlind(-5)
             .faces("<Y").workplane().circle(r - 12).cutBlind(-5))
    return wheel.union(cq.Workplane("XZ").circle(r - 25).extrude(-wdt))


def make_wheel(p: SuspensionParams) -> cq.Workplane:
    """아이들러 휠 — 허브 양끝 베어링 포켓 (내경 20 베어링 ×2, 외경 압입), 가운데 축 관통."""
    wdt = WHEELS[p.wheel_d]["width"]
    brg = BEARINGS[p.axle_d]
    wheel = _tire(p).cut(cq.Workplane("XZ").circle(p.axle_d / 2 + 1.0).extrude(wdt + 2, both=True))
    for y0 in (0.0, wdt - brg["width"]):
        pocket = cq.Workplane("XZ").circle((brg["od"] - 0.01) / 2).extrude(-brg["width"]).translate((0, y0, 0))
        wheel = wheel.cut(pocket)
    return wheel


def idler_bearing_gap(p: SuspensionParams) -> float:
    """아이들러 허브 베어링 2개 사이 간격 (허브 양끝에 하나씩)."""
    return WHEELS[p.wheel_d]["width"] - 2 * BEARINGS[p.axle_d]["width"]


def make_hub_motor_wheel(p: SuspensionParams) -> cq.Workplane:
    """전방 구동륜 — 10인치 타이어 + BLDC 허브 모터(스테이터 Ø160×60, 단순화)."""
    spec = WHEELS[p.wheel_d]
    wdt = spec["width"]
    wheel = _tire(p)
    motor = (cq.Workplane("XZ").circle(80).extrude(-60).translate((0, (wdt - 60) / 2, 0))
             .faces(">Y").workplane().circle(70).cutBlind(-3))
    # 스테이터 보어 = 고정축 M12 (+0.1)
    bore = cq.Workplane("XZ").circle(p.front_axle_d / 2 + 0.05).extrude(wdt + 2, both=True)
    return wheel.union(motor).cut(bore)


def make_diff_bar(p: SuspensionParams) -> cq.Workplane:
    """차동 바 — 단일 두께 판 (워터젯 + 탭): 중앙 패드에 베어링 홀더(BGRBB6001DD) 자리, 양단 M8 로드엔드 홀.

    홀더 플랜지는 바 윗면, 몸통이 바를 관통해 아래로 (L − T − t) 돌출.
    """
    half = p.derived["diff_bar_half_len"]
    hold = BEARING_HOLDERS[12]
    bar = (cq.Workplane("XY").slot2D(2 * half + p.diff_bar_w, p.diff_bar_w, 90)
           .extrude(p.diff_bar_t))
    bar = bar.union(cq.Workplane("XY").circle((hold["H"] + 6) / 2).extrude(p.diff_bar_t))
    bar = bar.cut(cq.Workplane("XY").circle(hold["D"] / 2).extrude(p.diff_bar_t + 2, both=True))
    bar = bar.cut(cq.Workplane("XY").pushPoints(holder_bolt_points(hold)).circle(hold["tap"] / 2)
                  .extrude(p.diff_bar_t + 2, both=True))
    bar = bar.cut(cq.Workplane("XY").pushPoints([(0, half), (0, -half)]).circle(4.1)
                  .extrude(p.diff_bar_t + 2, both=True))
    return bar.translate((0, 0, -p.diff_bar_t / 2))


def diff_holder_bottom_z(p: SuspensionParams) -> float:
    """차동 바 홀더 몸통 아래 끝 Z (플랜지가 바 윗면에 앉음)."""
    hold = BEARING_HOLDERS[12]
    return p.z_diff_bar + p.diff_bar_t / 2 + hold["T"] - hold["L"]


def make_diff_post(p: SuspensionParams) -> cq.Workplane:
    """차동 바 피벗 포스트 — 상부 크로스멤버(HFS5-4040, Y 방향) 위 M5 ×4, Ø12 수직 핀.

    체결 홀은 크로스멤버 상면 홈 2줄(X ±10)에 Y ±22 (시리즈 5 T너트는 M5 까지).
    기둥 끝은 홀더 몸통 아래 2 (내륜 스페이서 자리), 핀은 홀더 윗면 위 4 (멈춤링 자리).
    """
    base = p.z_frame_top_rail + p.frame_profile / 2
    col = diff_holder_bottom_z(p) - 2 - base
    post = (cq.Workplane("XY").rect(80, 60).extrude(10)            # 베이스 플레이트
            .faces(">Z").workplane().circle(16).extrude(col - 10))
    slot = p.hfs5_slot_offset
    post = post.cut(cq.Workplane("XY").pushPoints([(sx * slot, sy * 22.0) for sx in (1, -1) for sy in (1, -1)])
                    .circle(2.75).extrude(12).translate((0, 0, -1)))
    pin_top = p.z_diff_bar + p.diff_bar_t / 2 + BEARING_HOLDERS[12]["T"] + 4
    pin = cq.Workplane("XY").circle(6).extrude(pin_top - base)
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


@dataclass(frozen=True)
class FrameMember:
    """HFS5-4040 부재 하나: 축 방향, 중심, 길이, 끝단 맞대기 수 (M5 탭 가공)."""

    name: str
    axis: str                                   # "X" | "Y" | "Z"
    center: Tuple[float, float, float]
    length: float
    butt_ends: int


HFS5_4040 = {"kg_per_m": 1.28, "area": 472.0, "I": 8.89e4}   # MISUMI 카탈로그


def hfs5_4040_section() -> cq.Workplane:
    """HFS5-4040 단면 (XY, 중심 원점) — 외곽 40, 면마다 홈 2줄 (중심 ±10, 개구 6), 코너 구멍 4-Ø4.2.

    외곽·홈 위치·개구·코너 구멍은 카탈로그 외형도, T홈 내부·중앙 공동은 근사.
    (모델 단면적 ≈ 570 mm², 카탈로그 472 — 질량은 카탈로그 값으로 계산)
    """
    slot = [(-3, 0), (3, 0), (3, 1.8), (5, 1.8), (5, 4.3), (2, 6.8), (-2, 6.8), (-5, 4.3), (-5, 1.8), (-3, 1.8)]
    sec = cq.Workplane("XY").rect(40, 40).extrude(1)
    cutters = []
    for k in range(4):                                       # 면 4개: 회전 0/90/180/270
        c, s = math.cos(k * math.pi / 2), math.sin(k * math.pi / 2)
        for off in (-10.0, 10.0):
            pts = []
            for u, v in slot:                                # 면 (y = 20) 기준 → 회전
                x, y = off + u, 20.0 - v
                pts.append((c * x - s * y, s * x + c * y))
            cutters.append(cq.Workplane("XY").polyline(pts).close().extrude(1))
    octagon = [(13, 5), (5, 13), (-5, 13), (-13, 5), (-13, -5), (-5, -13), (5, -13), (13, -5)]
    cutters.append(cq.Workplane("XY").polyline(octagon).close().extrude(1))
    cutters.append(cq.Workplane("XY").pushPoints([(sx * 10, sy * 10) for sx in (1, -1) for sy in (1, -1)])
                   .circle(2.1).extrude(1))
    for ct in cutters:
        sec = sec.cut(ct)
    return sec


def make_profile(length: float, axis: str) -> cq.Workplane:
    """길이 length 의 HFS5-4040 부재, 축 방향 axis, 중심 원점."""
    section = hfs5_4040_section().faces("<Z").val()
    bar = cq.Workplane("XY").add(cq.Solid.extrudeLinear(section, cq.Vector(0, 0, length)))
    bar = bar.translate((0, 0, -length / 2))
    if axis == "X":
        return bar.rotate((0, 0, 0), (0, 1, 0), 90)
    if axis == "Y":
        return bar.rotate((0, 0, 0), (1, 0, 0), 90)
    return bar


def frame_members(p: SuspensionParams) -> List[FrameMember]:
    """구동체 프레임 HFS5-4040 부재 목록.

    좌우 레일(X 1000)이 관통, 앞뒤 가로대·차동 크로스멤버(Y)는 레일 안쪽 면에 맞대기,
    기둥(Z)은 하단 레일 위·상단 레일 아래에 맞대기. 맞대기 끝단은 M5 탭 (코너 구멍 Ø4.2),
    상대 관통 부재에 렌치 홀 → 캡 스크류로 체결 (브라켓 없음: 레일 상·측면은 부품 체결면).
    """
    pr = p.frame_profile
    hw = p.frame_half_w - pr / 2            # 레일 중심 Y
    hl = p.frame_len / 2 - pr / 2           # 가로대·기둥 중심 X
    zl, zt = p.z_frame_low_rail, p.z_frame_top_rail
    inner = 2 * hw - pr                     # 레일 안쪽 면 사이
    out: List[FrameMember] = []
    for lvl, z in (("low", zl), ("top", zt)):
        for side, y in (("L", hw), ("R", -hw)):
            out.append(FrameMember(f"rail_{lvl}_{side}", "X", (0.0, y, z), p.frame_len, 0))
        for end, x in (("F", hl), ("B", -hl)):
            out.append(FrameMember(f"cross_{lvl}_{end}", "Y", (x, 0.0, z), inner, 2))
    for end, x in (("F", hl), ("B", -hl)):
        for side, y in (("L", hw), ("R", -hw)):
            out.append(FrameMember(f"post_{end}{side}", "Z", (x, y, 0.5 * (zl + zt)), zt - zl - pr, 2))
    out.append(FrameMember("cross_diff", "Y", (p.derived["x_diff_pivot"], 0.0, zt), inner, 2))
    return out


def make_frame(p: SuspensionParams) -> Dict[str, cq.Workplane]:
    """프레임 부재별 형상 (구동체 좌표)."""
    return {f"frame_{m.name}": make_profile(m.length, m.axis).translate(m.center) for m in frame_members(p)}


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
    # 로커 베어링 홀더 BGRBB6004DD: 플랜지가 로커 판 안쪽 면, 몸통이 판을 관통해 바깥으로
    hr, hb = BEARING_HOLDERS[p.rocker_journal_d], BEARING_HOLDERS[p.bogie_shaft_d]
    y_rh = d["y_rocker_in"] - hr["T"]                       # 로커 홀더 플랜지 안쪽 끝
    rocker_grp["rocker_holder"] = (parts["rocker_holder"].translate((px, y_rh, pz)), COLORS["frame"])
    # 보기 베어링 홀더 BGRBB6001DD: 플랜지가 보기 판 바깥면, 몸통이 판을 관통해 안쪽으로
    y_bogie_out = d["y_bogie_in"] + p.plate_t
    y_bh = y_bogie_out + hb["T"] - hb["L"]                  # 보기 홀더 몸통 안쪽 끝
    # 로커 판 바깥면 → 보기 안쪽 베어링 내륜 스페이서 (로커와 함께 정지)
    rocker_grp["bogie_spacer"] = (
        parts["bogie_spacer"].translate((bx, d["y_rocker_in"] + p.plate_t, bz)), COLORS["pivot"])
    # 바깥 베어링 내륜 → 홀더 플랜지 면 + 1 스페이서 | PSCS12-10 칼라: 보기 그룹이 축에서 빠지지 않게
    rocker_grp["bogie_spacer_out"] = (
        parts["bogie_spacer_out"].translate((bx, y_bogie_out + hb["T"] - hb["l"], bz)), COLORS["pivot"])
    rocker_grp["bogie_collar"] = (
        parts["bogie_collar"].translate((bx, y_bogie_out + hb["T"] + 1, bz)), COLORS["pivot"])
    # --- 보기 그룹 (B 기준, 로커 회전 상쇄) ---
    bogie_grp: Dict[str, Tuple[cq.Workplane, cq.Color]] = {
        "bogie_plate": (_place_y(parts["bogie_plate"], d["y_bogie_in"], p.plate_t), COLORS["bogie"]),
        "bogie_holder": (parts["bogie_holder"].translate((bx, y_bogie_out + hb["T"], bz)), COLORS["frame"]),
        # 홀더 안 6001 ×2 (양 끝에서 l 안쪽) — 보기 축 위
        "bogie_bearing": (parts["bogie_bearing"].translate((bx, y_bh + hb["l"], bz)), COLORS["pivot"]),
    }
    y_wheel_out = d["y_wheel_in"] + wdt
    for key in ("M", "R"):
        x, z = pts[key]
        # SFJ20 축 | SHF20 홀더(판 바깥면) | 스페이서 | 휠(베어링 ×2) | 와셔 | PSCS20-12 칼라
        bogie_grp[f"axle_{key}"] = (parts["axle_mb"].translate((x, d["y_bogie_in"], z)), COLORS["pivot"])
        bogie_grp[f"axle_holder_{key}"] = (parts["axle_holder"].translate((x, y_bogie_out, z)), COLORS["frame"])
        bogie_grp[f"idler_spacer_{key}"] = (
            parts["idler_spacer"].translate((x, y_bogie_out + p.pivot_block_t, z)), COLORS["pivot"])
        bogie_grp[f"wheel_{key}"] = (parts["wheel"].translate((x, d["y_wheel_in"], z)), COLORS["wheel"])
        bogie_grp[f"wheel_bearing_{key}"] = (
            parts["idler_bearing"].translate((x, d["y_wheel_in"], z)), COLORS["pivot"])
        bogie_grp[f"idler_washer_{key}"] = (parts["idler_washer"].translate((x, y_wheel_out, z)), COLORS["pivot"])
        bogie_grp[f"idler_collar_{key}"] = (
            parts["shaft_collar"].translate((x, y_wheel_out + p.idler_washer_t, z)), COLORS["pivot"])

    for name, (obj, col) in bogie_grp.items():
        obj = _rot_y(obj, bx, bz, -theta)          # 보기: 로커 회전 상쇄
        rocker_grp[name] = (obj, col)
    for name, (obj, col) in rocker_grp.items():
        shapes[name] = (_rot_y(obj, px, pz, theta), col)

    # --- 프레임 고정부 ---
    shapes["pivot_block"] = (parts["pivot_block"].translate((px, p.frame_half_w, pz)), COLORS["frame"])
    shapes["rocker_shaft"] = (parts["rocker_shaft"].translate((px, p.frame_half_w, pz)), COLORS["pivot"])
    # 축 위 정지 부품 (회전 대칭이라 로커 각과 무관): SHF20 | 스페이서 | 홀더 안 6004 ×2 | 스페이서 | 칼라
    shapes["rocker_spacer_in"] = (
        parts["rocker_spacer_in"].translate((px, p.frame_half_w + p.pivot_block_t, pz)), COLORS["pivot"])
    shapes["rocker_bearing"] = (parts["rocker_bearing"].translate((px, y_rh + hr["l"], pz)), COLORS["pivot"])
    shapes["rocker_spacer_out"] = (
        parts["rocker_spacer_out"].translate((px, y_rh + hr["L"] - hr["l"], pz)), COLORS["pivot"])
    shapes["shaft_collar"] = (parts["shaft_collar"].translate((px, y_rh + hr["L"] + 1, pz)), COLORS["pivot"])

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
    hr, hb = BEARING_HOLDERS[p.rocker_journal_d], BEARING_HOLDERS[p.bogie_shaft_d]

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
        "rocker_holder": make_bearing_holder(p.rocker_journal_d),
        "rocker_bearing": make_bearing_pair(p.rocker_journal_d, holder_bearing_gap(p.rocker_journal_d)),
        "rocker_spacer_in": make_spacer(p, p.block_gap + hr["l"], p.rocker_journal_d),
        "rocker_spacer_out": make_spacer(p, hr["l"] + 1, p.rocker_journal_d),
        # 보기 홀더는 플랜지를 판 바깥면에 두므로 Y 반전 (플랜지 −T→0, 몸통 −L→−T)
        "bogie_holder": make_bearing_holder(p.bogie_shaft_d).mirror("XZ"),
        "bogie_bearing": make_bearing_pair(p.bogie_shaft_d, holder_bearing_gap(p.bogie_shaft_d)),
        "bogie_spacer": make_spacer(p, p.plate_gap - (hb["L"] - hb["T"] - p.plate_t) + hb["l"],
                                    p.bogie_shaft_d, 16.0),
        "bogie_spacer_out": make_spacer(p, hb["l"] + 1, p.bogie_shaft_d, 16.0),
        "bogie_collar": make_shaft_collar(p, p.bogie_shaft_d, p.bogie_collar_od, p.bogie_collar_w),
        "front_fork": make_front_fork(p),
        "standoff": make_standoff(p),
        "axle_holder": make_axle_holder(p),
        "idler_spacer": make_spacer(p, p.wheel_gap - p.pivot_block_t),
        "idler_washer": make_spacer(p, p.idler_washer_t),
        "idler_bearing": make_bearing_pair(p.axle_d, idler_bearing_gap(p)),
    }
    parts["rocker_shaft"] = make_rocker_shaft(p)
    parts["rocker_shaft_len"] = rocker_shaft_length(p)
    parts["bogie_shaft"] = make_bogie_shaft(p)
    parts["bogie_shaft_len"] = bogie_shaft_length(p)
    parts["axle_front"] = make_front_axle(p)
    parts["axle_mb"] = make_axle(p)
    parts["axle_mb_len"] = idler_axle_length(p)

    asm = cq.Assembly(name="rocker_bogie_suspension")
    for name, obj in make_frame(p).items():
        asm.add(obj, name=name, color=COLORS["frame"])
    asm.add(_side(p, +1.0, theta_left, parts), name="left")
    asm.add(_side(p, -1.0, theta_right, parts), name="right")

    # 차동 바 + 포스트 + 링크
    xd = d["x_diff_pivot"]
    asm.add(parts["diff_post"].translate((xd, 0, p.z_frame_top_rail + p.frame_profile / 2)),
            name="diff_post", color=COLORS["frame"])
    bar_loc = cq.Location(cq.Vector(xd, 0, p.z_diff_bar), cq.Vector(0, 0, 1), math.degrees(phi))
    asm.add(parts["diff_bar"], name="diff_bar", color=COLORS["diff"], loc=bar_loc)
    # 차동 바 홀더 BGRBB6001DD: 플랜지가 바 윗면 (+Y 축 → −Z 로 돌려 몸통이 아래로)
    holder = (make_bearing_holder(12).rotate((0, 0, 0), (1, 0, 0), -90)
              .translate((0, 0, p.diff_bar_t / 2 + hb["T"])))
    asm.add(holder, name="diff_holder", color=COLORS["frame"], loc=bar_loc)
    # 홀더 안 6001 ×2 — 포스트 Ø12 핀 위, 기둥 끝 ↔ 아래 베어링 내륜 스페이서
    z_hb = diff_holder_bottom_z(p)
    asm.add(make_bearing_pair(12, holder_bearing_gap(12), "XY").translate((xd, 0, z_hb + hb["l"])),
            name="diff_bearing", color=COLORS["pivot"])
    asm.add(make_spacer(p, 2 + hb["l"], 12, 16.0).rotate((0, 0, 0), (1, 0, 0), 90)
            .translate((xd, 0, z_hb - 2)), name="diff_spacer", color=COLORS["pivot"])
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
        section = cq.Workplane("XY").add(flat.val()).section(p.plate_t / 2)   # 단일 두께 판, 홀 포함
        cq.exporters.exportDXF(section, str(out / "dxf" / f"{name}.dxf"))
    # 차동 바는 XY 평면 판 (두께 중심 Z 0)
    cq.exporters.exportDXF(singles["diff_bar"].section(0.0), str(out / "dxf" / "diff_bar.dxf"))

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
