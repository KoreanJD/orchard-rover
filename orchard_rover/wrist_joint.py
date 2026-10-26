"""6축 손목 모듈 — J4 전완 롤(X) / J5 손목 피치(Y) / J6 툴 롤(X) + ISO 9409-1-50-4-M6 플랜지.

팔 좌표계 (X 전개 방향, Y 관절 피치 축, Z 상방). J3 축 X=500 기준으로 체인:
  J3 출력 측판 → J4 하우징(고정) | J4 출력 → 전완 튜브(회전) → J5 요크 → J5 출력 → 손목 블록
  → J6 모터 → ISO 플랜지 (X = 1050)
NEMA17 (42×42×48) + 플랜지형 유성감속기(Ø42) 구매품, 하우징/요크/블록은 Al 가공.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Tuple

import cadquery as cq

NEMA17 = 42.3
COL = {
    "motor": cq.Color(0.2, 0.2, 0.25), "gear": cq.Color(0.35, 0.35, 0.4),
    "housing": cq.Color(0.6, 0.6, 0.63), "tube": cq.Color(0.7, 0.7, 0.72),
    "rotor": cq.Color(0.85, 0.3, 0.3), "flange": cq.Color(0.2, 0.5, 0.85),
}


@dataclass
class WristParams:
    x_j3: float = 500.0
    x_flange: float = 1050.0
    j4_housing_len: float = 80.0     # 전완 루트, 고정부 (NEMA17 + 유성 1:36 수납)
    j4_housing_d: float = 64.0
    tube_d: float = 50.0
    tube_t: float = 2.0
    j5_yoke_len: float = 60.0
    wrist_block: Tuple[float, float, float] = (50.0, 60.0, 60.0)   # X, Y, Z
    j6_len: float = 60.0             # NEMA17 + 1:20 유성 (블록 안쪽 일부 수납)
    flange_d: float = 63.0
    flange_t: float = 8.0
    gear_len: float = 50.0


def iso9409_flange(d: float = 63.0, t: float = 8.0) -> cq.Workplane:
    """ISO 9409-1-50-4-M6: PCD 50 × M6 ×4, Ø6 H7 위치핀 1, 중앙 Ø31.5 H7 보스 홈."""
    fl = cq.Workplane("YZ").circle(d / 2).extrude(t)
    pts = [(25 * math.cos(math.radians(a)), 25 * math.sin(math.radians(a))) for a in (45, 135, 225, 315)]
    fl = fl.faces(">X").workplane().pushPoints(pts).circle(2.5).cutThruAll()        # M6 탭 (하경)
    fl = fl.faces(">X").workplane().center(0, 25).circle(3.0).cutBlind(-6)          # 위치핀 Ø6
    fl = fl.faces(">X").workplane().circle(31.5 / 2).cutBlind(-3)                   # 센터링 리세스
    return fl


def _nema17_x(length: float) -> cq.Workplane:
    """NEMA17 더미, X 축 방향, 전면 X=0."""
    return cq.Workplane("YZ").rect(NEMA17, NEMA17).extrude(-length).edges("|X").chamfer(4)


def _planetary_x(length: float, d: float = 42.0) -> cq.Workplane:
    """플랜지형 유성감속기 더미, 입력 X=0 → 출력 X=length, 출력축 Ø8."""
    g = cq.Workplane("YZ").circle(d / 2).extrude(length)
    return g.union(cq.Workplane("YZ").circle(4).extrude(length + 15))


def build_wrist(w: WristParams) -> Dict[str, Tuple[cq.Workplane, cq.Color]]:
    """손목 체인 형상 dict (팔 좌표계)."""
    s: Dict[str, Tuple[cq.Workplane, cq.Color]] = {}
    x = w.x_j3 + 60.0   # J3 출력 측판 뒤 (측판 폭 고려)

    # ---- J4: 전완 롤 (고정 하우징 + 회전 튜브) ----
    hous = (cq.Workplane("YZ").circle(w.j4_housing_d / 2).extrude(w.j4_housing_len)
            .faces(">X").workplane().circle(w.tube_d / 2 + 1).cutBlind(-10)
            .translate((x, 0, 0)))
    # J3 측판 연결 플랜지 (하우징 후단, 측판 볼트 M6 ×4)
    hous = hous.union(cq.Workplane("YZ").rect(90, 90).extrude(8).edges("|X").fillet(8).translate((x, 0, 0)))
    s["j4_housing"] = (hous, COL["housing"])
    s["j4_motor"] = (_nema17_x(48).translate((x + 10, 0, 0)), COL["motor"])          # 하우징 내부
    s["j4_gear"] = (_planetary_x(w.gear_len).translate((x + 10, 0, 0)), COL["gear"])
    x_tube0 = x + w.j4_housing_len
    # 롤 출력 허브 + 전완 튜브 (회전부)
    x_tube1 = w.x_flange - w.flange_t - w.j6_len - w.wrist_block[0] - w.j5_yoke_len
    tube = (cq.Workplane("YZ").circle(w.tube_d / 2).circle(w.tube_d / 2 - w.tube_t)
            .extrude(x_tube1 - x_tube0).translate((x_tube0, 0, 0)))
    hub = cq.Workplane("YZ").circle(w.tube_d / 2).extrude(12).translate((x_tube0 - 2, 0, 0))
    s["forearm_tube"] = (tube.union(hub), COL["tube"])
    # 롤 베어링 (6810 류, 50×65×7) 하우징 전단
    s["j4_bearing"] = (cq.Workplane("YZ").circle(32.5).circle(25).extrude(7).translate((x_tube0 - 9, 0, 0)),
                       cq.Color(0.8, 0.8, 0.8))

    # ---- J5: 손목 피치 (요크 + 횡방향 NEMA17) ----
    yoke_x0 = x_tube1
    xc5 = yoke_x0 + w.j5_yoke_len - 10          # J5 축 X
    yoke = (cq.Workplane("YZ").circle(w.tube_d / 2).extrude(14).translate((yoke_x0, 0, 0)))
    arm_w = 12.0
    for sgn in (+1, -1):
        cheek = (cq.Workplane("XZ").center(xc5 - 15, 0).rect(w.j5_yoke_len + 10, 60).extrude(-arm_w)
                 .edges("|Y").fillet(8)
                 .translate((0, sgn * (w.wrist_block[1] / 2 + 2) + (arm_w if sgn > 0 else 0), 0)))
        yoke = yoke.union(cheek)
    s["j5_yoke"] = (yoke, COL["housing"])
    # J5 모터: 요크 한쪽 바깥에 Y 방향, 감속기 출력이 축 핀
    m5 = (cq.Workplane("XZ").rect(NEMA17, NEMA17).extrude(-48)          # Y 0→+48
          .translate((xc5, w.wrist_block[1] / 2 + 2 + arm_w + w.gear_len, 0)))
    g5 = (cq.Workplane("XZ").circle(21).extrude(-w.gear_len)
          .translate((xc5, w.wrist_block[1] / 2 + 2 + arm_w, 0)))
    s["j5_motor"] = (m5, COL["motor"])
    s["j5_gear"] = (g5, COL["gear"])
    # 반대편 지지 베어링 (6001)
    s["j5_bearing"] = (cq.Workplane("XZ").circle(14).circle(6).extrude(8)
                       .translate((xc5, -(w.wrist_block[1] / 2 + 2), 0)), cq.Color(0.8, 0.8, 0.8))

    # ---- 손목 블록 (J5 회전부) + J6 롤 ----
    bx, by, bz = w.wrist_block
    blk = (cq.Workplane("XY").box(bx, by, bz).edges("|Y").fillet(6)
           .translate((xc5 + bx / 2 - 15, 0, 0)))
    s["wrist_block"] = (blk, COL["rotor"])
    x6 = xc5 + bx - 15
    s["j6_motor"] = (_nema17_x(40).translate((x6 - 5, 0, 0)), COL["motor"])      # 블록 안쪽 일부
    x_fl = w.x_flange - w.flange_t
    s["j6_gear"] = (_planetary_x(x_fl - x6 - 15, 42).translate((x6, 0, 0)), COL["gear"])   # 출력축이 플랜지 허브로
    s["iso_flange"] = (iso9409_flange(w.flange_d, w.flange_t).translate((x_fl, 0, 0)), COL["flange"])
    # 손목 카메라 마운트 (블록 상단)
    s["wrist_camera"] = (cq.Workplane("XY").box(30, 30, 20).translate((xc5 + 10, 0, bz / 2 + 10)),
                         cq.Color(0.1, 0.1, 0.1))
    return s


def wrist_axes(w: WristParams) -> Dict[str, float]:
    x = w.x_j3 + 60.0
    x_tube1 = w.x_flange - w.flange_t - w.j6_len - w.wrist_block[0] - w.j5_yoke_len
    return {"J4_roll_axis": "X (전완 중심)", "J5_pitch_x": x_tube1 + w.j5_yoke_len - 10,
            "J6_roll_axis": "X", "flange_x": w.x_flange,
            "forearm_len_j3_to_j5": x_tube1 + w.j5_yoke_len - 10 - w.x_j3}


if __name__ == "__main__":
    w = WristParams()
    print(wrist_axes(w))
    shapes = build_wrist(w)
    comp = cq.Compound.makeCompound([o.val() for o, _ in shapes.values()])
    print("solids", len(comp.Solids()))
