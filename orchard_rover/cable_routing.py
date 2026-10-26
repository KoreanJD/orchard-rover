"""6축 팔 케이블 라우팅 — 전장 구성, 트렁크 케이블 산정, 관절별 여유 길이, CAD 경로 생성.

전제 (제어 아키텍처): 각 관절 모터 후면에 CAN 폐루프 스테퍼 드라이버 일체형 → 팔을 따라
올라가는 케이블은 **36 V 전원 + CAN 버스 + 비상정지/24 V** 트렁크 1본과 **툴 케이블 + 카메라** 뿐.
관절마다 드라이버를 두므로 모터 상선(4선 대전류)이 팔을 타지 않는다.

경로 원칙
  - 상완: HFS5-3030 사다리의 레일 사이 60 mm 공간을 케이블 채널로 사용 (분할 주름관 Ø20)
  - 피치 관절(J2, J3, J5): 축에서 오프셋된 클램프 2점 사이에 루프 — 각도 범위에서 클램프 간 거리 변화량 + 굽힘 반경 여유
  - 롤 관절(J1, J4, J6): 중공축 없음 → 케이블 꼬임 허용 구간(길이 L 당 ±180° 는 고유연 로봇 케이블 기준 L ≥ 150~200 mm)
  - 툴: 손목 블록(J5 회전부)에 M12 커넥터 고정, 툴 피그테일 250 mm 로 J6 ±180° 흡수

좌표계: 팔 좌표 (X 전개, Y 관절축, Z 상방), J2 축 원점.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Tuple

import cadquery as cq

from orchard_rover.arm_assembly import UPPER_LEN

Vec = Tuple[float, float, float]


# ---------------------------------------------------------------------------
# 1. 케이블 구성 및 번들 산정
# ---------------------------------------------------------------------------
@dataclass
class Conductor:
    name: str
    cores: int
    mm2: float
    od: float          # 단심/페어 완성 외경 (mm)
    note: str = ""


TRUNK: List[Conductor] = [
    Conductor("36V 전원 (관절 6축 + 브레이크)", 2, 2.5, 4.0, "피크 400 W / 11 A, 고유연 PUR"),
    Conductor("CAN 버스", 2, 0.25, 4.5, "STP 1페어 + 실드, 1 Mbps, 종단 120 Ω 손목 끝"),
    Conductor("비상정지 / 24 V 보조", 3, 0.5, 4.2, "E-stop 루프 2 + 24 V 1"),
]
TOOL: List[Conductor] = [
    Conductor("툴 36V 전원", 2, 2.5, 4.0, "10 A (예초기 BLDC 기준)"),
    Conductor("툴 CAN + I/O", 5, 0.25, 5.0, "CAN 페어 + 24 V I/O 2 + GND"),
]
CAMERA: List[Conductor] = [
    Conductor("손목 카메라 USB 3.0", 8, 0.14, 5.5, "고유연 USB3, 또는 GMSL 동축 Ø4"),
]


def bundle_od(conds: List[Conductor]) -> float:
    """원형 패킹 근사: 번들 외경 ≈ 1.15 · sqrt(Σ od²) (3~8 가닥 범위 경험식) + 주름관 여유."""
    return 1.15 * math.sqrt(sum(c.od ** 2 for c in conds))


# ---------------------------------------------------------------------------
# 2. 관절별 클램프 위치와 여유 길이
# ---------------------------------------------------------------------------
@dataclass
class JointRoute:
    """관절 하나를 가로지르는 구간: 상류 클램프(고정측) → 하류 클램프(회전측)."""
    name: str
    axis_origin: Vec
    axis_dir: Vec
    clamp_fixed: Vec          # 고정측 링크 좌표(월드, 홈 자세)
    clamp_moving: Vec         # 회전측 링크 좌표(월드, 홈 자세)
    range_deg: Tuple[float, float]
    kind: str = "pitch"       # pitch | roll | wrap
    bundle: float = 15.0      # 번들 외경
    r_wrap: float = 0.0       # wrap: 하우징 외주 감기 반경 (하우징 OD/2 + 번들/2)
    wrap_home_deg: float = 0.0  # wrap: 홈 자세에서 감긴 각도 (클램프 사이 호)

    def moving_point(self, deg: float) -> Vec:
        """회전측 클램프를 관절각 deg 만큼 회전시킨 위치."""
        o = cq.Vector(*self.axis_origin)
        p = cq.Vector(*self.clamp_moving)
        return tuple((p - o).rotate(cq.Vector(*self.axis_dir), deg) + o) if False else \
            _rotate_point(self.clamp_moving, self.axis_origin, self.axis_dir, deg)

    def analyse(self) -> Dict[str, float]:
        """각도 범위에서 클램프 간 직선 거리 최소/최대와 필요 케이블 길이."""
        lo, hi = self.range_deg
        d = [math.dist(self.clamp_fixed, self.moving_point(a)) for a in _frange(lo, hi, 2.0)]
        d_min, d_max = min(d), max(d)
        r_bend = 7.5 * self.bundle            # 동적 굽힘 반경 ≥ 7.5×OD (고유연 케이블)
        if self.kind == "wrap":
            # 하우징에 감김: 감긴 각도 = 홈 각도 + 관절각. 범위 전체에서 0° < 감김 < 330° 이어야 함
            span = hi - lo
            wrap_min = self.wrap_home_deg + lo
            wrap_max = self.wrap_home_deg + hi
            need = self.r_wrap * math.radians(wrap_max) + 2 * 40.0      # 최대 감김 호 + 접선 2×40
            slack_at_min = self.r_wrap * math.radians(span)              # 최소 감김 시 풀려 나오는 길이
            d_min, d_max = wrap_min, wrap_max                             # 보고용: 감김 각도(°)
            if wrap_min < 15 or wrap_max > 330:
                raise ValueError(f"{self.name}: 감김 각도 범위 {wrap_min:.0f}~{wrap_max:.0f}° 부적합")
        elif self.kind == "pitch":
            # 최대 거리 + 양단 굽힘 반원 여유 → 최소 거리에서 남는 길이가 루프로 처짐
            need = d_max + math.pi * r_bend * 0.5
            slack_at_min = need - d_min
        else:
            # 롤: 꼬임 흡수 직선 구간 — ±180° 당 ≥ 200 mm, 범위에 비례
            twist = max(abs(lo), abs(hi))
            need = max(d_max, 200.0 * twist / 180.0)
            slack_at_min = need - d_min
        return {"d_min": d_min, "d_max": d_max, "bend_radius": r_bend,
                "cable_len": need, "slack_at_min": slack_at_min}


def _rotate_point(p: Vec, o: Vec, axis: Vec, deg: float) -> Vec:
    v = cq.Vector(*p) - cq.Vector(*o)
    ax = cq.Vector(*axis).normalized()
    a = math.radians(deg)
    # Rodrigues
    term1 = v * math.cos(a)
    term2 = ax.cross(v) * math.sin(a)
    term3 = ax * (ax.dot(v)) * (1 - math.cos(a))
    r = term1 + term2 + term3 + cq.Vector(*o)
    return (r.x, r.y, r.z)


def _frange(a: float, b: float, step: float):
    n = int((b - a) / step) + 1
    return [a + i * step for i in range(n)] + [b]


def define_routes(x_j3: float = UPPER_LEN, x_j5: float = 922.0, x_fl: float = 1050.0) -> List[JointRoute]:
    """홈 자세(팔 수평 전개) 기준 클램프 좌표. Y 는 팔 중심선 기준 (+ 좌측)."""
    trunk_od = bundle_od(TRUNK + TOOL + CAMERA)          # 손목까지 전부 올라가는 번들
    y_ch = 0.0        # 상완 채널은 두 레일 사이 (Y = 플레이트 측 +?); 단순화: 팔 중심
    return [
        # J1: 터릿 아래 베이스 플레이트 → 터릿 상단. 요 ±180°, 꼬임 구간 = 터릿 포스트 옆 수직 루프
        JointRoute("J1_yaw", (0, 0, -230), (0, 0, 1), (120, -60, -250), (90, -40, -40),
                   (-180, 180), "roll", trunk_od),
        # J2: 브라켓(고정) 상단 → 상완 채널 입구. 클램프를 축 후방·위쪽에 두어 범위 내 거리 변화 최소화
        # J2: 출력 플랜지 동축 케이블 드럼 Ø60 에 감기. 홈 150° → −20°: 130°, +90°: 240°
        JointRoute("J2_shoulder", (0, 0, 0), (0, 1, 0), (-70, 6, 120), (90, -34, 55),
                   (-20, 90), "wrap", trunk_od, r_wrap=30 + trunk_od / 2, wrap_home_deg=150),
        # J3: 상완 채널 출구 → J4 하우징 상부
        # J3: 동축 드럼 Ø60. 범위 ±120° (케이블 때문에 ±140 → ±120 로 제한) → 홈 150° → 30°~270°
        JointRoute("J3_elbow", (x_j3, 0, 0), (0, 1, 0), (x_j3 - 90, -34, 55), (x_j3 + 90, 0, 45),
                   (-120, 120), "wrap", trunk_od, r_wrap=30 + trunk_od / 2, wrap_home_deg=150),
        # J4: 하우징(고정) → 전완 튜브 외면 클램프 (롤 ±180)
        JointRoute("J4_roll", (x_j3 + 100, 0, 0), (1, 0, 0), (x_j3 + 110, 0, 40), (x_j3 + 330, 0, 32),
                   (-180, 180), "roll", bundle_od(TRUNK[1:] + TOOL + CAMERA)),
        # J5: 요크 상단 → 손목 블록 상단(카메라 옆)
        # J5: 요크 축 허브 Ø60 외주. ±110° → 홈 150° → 40°~260°
        JointRoute("J5_pitch", (x_j5, 0, 0), (0, 1, 0), (x_j5 - 60, -20, 36), (x_j5 + 25, -20, 45),
                   (-110, 110), "wrap", bundle_od(TOOL + CAMERA), r_wrap=25 + bundle_od(TOOL + CAMERA) / 2,
                   wrap_home_deg=150),
        # J6: 손목 블록 M12 커넥터 → 툴 피그테일 (툴 측 고정점은 플랜지 밖 60)
        JointRoute("J6_tool_roll", (x_fl, 0, 0), (1, 0, 0), (x_fl - 45, 0, 42), (x_fl + 60, 0, 20),
                   (-180, 180), "roll", bundle_od(TOOL)),
    ]


# ---------------------------------------------------------------------------
# 3. CAD: 홈 자세 케이블 경로 스윕 + 클램프
# ---------------------------------------------------------------------------
def cable_sweep(points: List[Vec], od: float) -> cq.Workplane:
    path = cq.Workplane("XY").spline([cq.Vector(*p) for p in points])
    p0, p1 = cq.Vector(*points[0]), cq.Vector(*points[1])
    tangent = (p1 - p0).normalized()
    plane = cq.Plane(origin=p0, normal=tangent)
    return cq.Workplane(plane).circle(od / 2).sweep(path, transition="round")


# 드럼 Y 위치 (팔 적층 기준 빈 공간): J2 = 상완 플레이트 외측(−Y), J3 = 전완 플레이트 +Y 면, J5 = 요크 −Y 외측
DRUM_Y = {"J2": -80.0, "J3": -12.0, "J5": -55.0}


def _wrap_arc(rt: JointRoute, y_side: float, start_deg: float, n: int = 12) -> List[Vec]:
    """관절 축 둘레 XZ 평면 호 (y_side 에서), start_deg 에서 wrap_home_deg 만큼 반시계(앞쪽 위→뒤) 감김."""
    ox, _, oz = rt.axis_origin
    pts = []
    for i in range(n + 1):
        a = math.radians(start_deg - rt.wrap_home_deg * i / n)
        pts.append((ox + rt.r_wrap * math.cos(a), y_side, oz + rt.r_wrap * math.sin(a)))
    return pts


def home_path_points(routes: List[JointRoute]) -> Dict[str, List[Vec]]:
    """홈 자세에서 관절별 루프/감김을 포함한 경로 점열 (상완 채널은 직선)."""
    r = {rt.name: rt for rt in routes}
    x_j3 = UPPER_LEN
    pts: Dict[str, List[Vec]] = {}
    # J1 수직 루프 (터릿 옆)
    pts["J1"] = [r["J1_yaw"].clamp_fixed, (140, -70, -200), (150, -60, -120), r["J1_yaw"].clamp_moving]
    # J2 감김: 하우징 측면(Y=+95, 모터 측) 에서 상후방 120° → 시계방향으로 150° 감아 전방 하단 −30° 로
    rt = r["J2_shoulder"]
    arc = _wrap_arc(rt, DRUM_Y["J2"], 120.0)
    pts["J2"] = [rt.clamp_fixed, (-60, -40, 110)] + arc + [(55, -60, 35), rt.clamp_moving]
    # 상완 채널 직선 (레일 사이, Y ≈ −34)
    pts["upper_channel"] = [rt.clamp_moving, (x_j3 - 90, -34, 55)]
    rt = r["J3_elbow"]
    arc = _wrap_arc(rt, DRUM_Y["J3"], 130.0)
    pts["J3"] = [rt.clamp_fixed, (x_j3 - 70, -25, 70)] + arc + [(x_j3 + 70, -8, 60), rt.clamp_moving]
    a, b = r["J4_roll"].clamp_fixed, r["J4_roll"].clamp_moving
    pts["J4_forearm"] = [a, (x_j3 + 200, 10, 34), b]
    rt = r["J5_pitch"]
    arc = _wrap_arc(rt, DRUM_Y["J5"], 130.0)
    pts["J5"] = [rt.clamp_fixed, (922 - 50, -35, 40)] + arc + [(922 + 15, -30, 50), rt.clamp_moving]
    a, b = r["J6_tool_roll"].clamp_fixed, r["J6_tool_roll"].clamp_moving
    pts["J6_pigtail"] = [a, (1050 - 10, 25, 70), (1050 + 30, 20, 60), b]
    return pts


def build_cable_shapes(routes: List[JointRoute]) -> Dict[str, Tuple[cq.Workplane, cq.Color]]:
    shapes: Dict[str, Tuple[cq.Workplane, cq.Color]] = {}
    od_map = {rt.name.split("_")[0]: rt.bundle for rt in routes}
    col = cq.Color(0.95, 0.6, 0.1)
    for name, pts in home_path_points(routes).items():
        od = od_map.get(name.split("_")[0], 12.0)
        try:
            shapes[f"cable_{name}"] = (cable_sweep(pts, od), col)
        except Exception as exc:  # 스윕 실패 시 폴리라인 점열만 남김
            print(f"[warn] sweep failed for {name}: {exc}")
    # 케이블 드럼 (wrap 관절, 관절 축 동축, 폭 = 번들 + 6)
    for rt in routes:
        if rt.kind == "wrap":
            y_side = DRUM_Y[rt.name.split("_")[0]]
            drum_r = rt.r_wrap - rt.bundle / 2
            drum = (cq.Workplane("XZ").circle(drum_r + 6).circle(drum_r).extrude(-(rt.bundle + 6))
                    .translate((rt.axis_origin[0], y_side - (rt.bundle + 6) / 2, rt.axis_origin[2])))
            flange = (cq.Workplane("XZ").circle(drum_r + 6).extrude(-3)
                      .translate((rt.axis_origin[0], y_side - (rt.bundle + 6) / 2 - 3, rt.axis_origin[2])))
            shapes[f"drum_{rt.name}"] = (drum.union(flange), cq.Color(0.5, 0.5, 0.55))
    # 클램프: 각 클램프 점에 새들 블록
    for rt in routes:
        for tag, p in (("fixed", rt.clamp_fixed), ("moving", rt.clamp_moving)):
            blk = cq.Workplane("XY").box(16, rt.bundle + 8, 10).translate(p)
            shapes[f"clamp_{rt.name}_{tag}"] = (blk, cq.Color(0.3, 0.3, 0.35))
    return shapes


# ---------------------------------------------------------------------------
# 4. 보고서
# ---------------------------------------------------------------------------
def report(routes: List[JointRoute]) -> str:
    lines = ["=== 케이블 구성 ==="]
    for grp, conds in (("트렁크 (베이스→손목)", TRUNK), ("툴", TOOL), ("카메라", CAMERA)):
        lines.append(f"[{grp}]  번들 OD ≈ {bundle_od(conds):.1f} mm")
        for c in conds:
            lines.append(f"  - {c.name}: {c.cores}C × {c.mm2} mm², Ø{c.od}  {c.note}")
    lines.append(f"상완 채널 통과 번들(전부): OD ≈ {bundle_od(TRUNK + TOOL + CAMERA):.1f} mm → 분할 주름관 Ø20~25")
    lines.append(f"전완 (CAN·E-stop·툴·카메라): OD ≈ {bundle_od(TRUNK[1:] + TOOL + CAMERA):.1f} mm")
    lines.append(f"손목 이후 (툴·카메라): OD ≈ {bundle_od(TOOL + CAMERA):.1f} mm")
    lines += ["", "=== 관절별 여유 길이 (mm) ===",
              f"{'관절':14s} {'종류':5s} {'범위':>12s} {'min':>7s} {'max':>7s} {'Rbend':>6s} {'케이블':>7s} {'여유/풀림':>10s}",
              "  (wrap 은 min/max = 감김 각도 °, 나머지는 클램프 간 거리 mm)"]
    total = 0.0
    for rt in routes:
        a = rt.analyse()
        total += a["cable_len"]
        lines.append(f"{rt.name:14s} {rt.kind:5s} {rt.range_deg[0]:+5.0f}~{rt.range_deg[1]:+4.0f}° "
                     f"{a['d_min']:7.0f} {a['d_max']:7.0f} {a['bend_radius']:6.0f} "
                     f"{a['cable_len']:7.0f} {a['slack_at_min']:10.0f}")
    straight = (UPPER_LEN - 180) + (922 - UPPER_LEN - 330 + 230)   # 상완 채널 + 전완 직선
    lines.append(f"\n직선 구간 합 ≈ {straight:.0f}, 관절 루프 합 ≈ {total:.0f} → 트렁크 총장 ≈ {straight + total + 300:.0f} (베이스 여유 300 포함)")
    return "\n".join(lines)


if __name__ == "__main__":
    routes = define_routes()
    print(report(routes))
