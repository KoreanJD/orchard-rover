"""Rocker-Bogie 서스펜션 설계 파라미터 및 정역학/기구학 계산 모듈.

좌표계 (CadQuery 표준, 단위 mm):
    X : 전방(+)
    Y : 좌측(+)  — 우측 서스펜션은 Y 부호 반전(미러)
    Z : 상방(+), 지면 Z = 0

구조 (NASA MER/MSL 방식, 한쪽 측면 기준):
    차체 측면 피벗 P ─ 로커(Rocker)
        ├─ 전방 끝   : 전방 휠 F
        ├─ 후방 끝   : 보기 피벗 B
        └─ 상방 끝   : 차동 링크 연결점 U
    보기 피벗 B ─ 보기(Bogie)
        ├─ 전방 끝   : 중간 휠 M
        └─ 후방 끝   : 후방 휠 R
    차체 상부 중앙 ─ 차동 바(Differential Bar, Z축 회전)
        └─ 양단 ─ 링크 로드 ─ 좌/우 로커 U점

차체에 연결되는 지점은 좌/우 로커 피벗 2개 + 차동 바 피벗 1개 = 총 3개.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Tuple

Point2 = Tuple[float, float]


# ---------------------------------------------------------------------------
# MISUMI 표준 부품 테이블 (misumi_parts.md 기준)
# ---------------------------------------------------------------------------
BEARINGS: Dict[int, Dict[str, float]] = {
    # 축경 : {OD, 폭, 정격 하중(N)}
    10: {"od": 26.0, "width": 8.0, "load": 2360.0, "name": "6000ZZ"},
    12: {"od": 28.0, "width": 8.0, "load": 2800.0, "name": "6001ZZ"},
    16: {"od": 35.0, "width": 10.0, "load": 4750.0, "name": "6003ZZ"},
    20: {"od": 42.0, "width": 12.0, "load": 6120.0, "name": "6004ZZ"},
}

WHEELS: Dict[int, Dict[str, float]] = {
    # 직경 : {폭, 보어(액슬), 허용 하중(N), 종류}
    # UWRF 우레탄 (MISUMI) — 실내/평지용, 과수원에는 소형
    100: {"width": 30.0, "bore": 12.0, "load": 500.0, "type": "UWRF100"},
    125: {"width": 35.0, "bore": 16.0, "load": 700.0, "type": "UWRF125"},
    150: {"width": 40.0, "bore": 16.0, "load": 900.0, "type": "UWRF150"},
    # 공압 타이어 / 허브 모터 (범용 규격) — 과수원 요철·뿌리 대응
    #   250: 10인치 3.00-4 타이어 — 전방: BLDC 허브모터(36V 350W, M12 축),
    #        중간/후방: 동일 타이어 아이들러(6003ZZ 베어링 허브)
    #   330: 13인치 4.10/3.50-6 — 더 큰 장애물, 프레임 높이 상향 필요
    250: {"width": 80.0, "bore": 16.0, "load": 1500.0, "type": "10in 3.00-4 / hub motor"},
    330: {"width": 90.0, "bore": 16.0, "load": 2000.0, "type": "13in 4.10/3.50-6"},
}


@dataclass
class SuspensionParams:
    """Rocker-Bogie 서스펜션의 모든 설계 치수(mm) 및 하중 조건.

    기본값은 1000×800 프레임(HFS5-4040) 과수원 제초 로봇 기준.

    Attributes:
        robot_mass_kg: 서스펜션이 지지하는 전체 질량(kg).
        dynamic_factor: 충격/동하중 계수(정하중 대비 배율).
        wheel_d: 휠 직경(UWRF 시리즈: 100/125/150).
        x_front, x_mid, x_rear: 휠 접지점 X 좌표.
        z_rocker_pivot: 차체 측 로커 피벗 높이.
        z_bogie_pivot: 보기 피벗 높이(휠 축보다 높게).
        z_diff_bar: 차동 바 높이(프레임 상단 위).
        frame_half_w: 프레임 외측면까지의 Y 거리.
        plate_t: 로커/보기 플레이트 두께(Al 6061-T6).
        arm_w: 로커/보기 암 폭(측면도 상 높이).
        diff_bar_half_len: 차동 바 중심→링크 연결점 거리(Y).
        rocker_shaft_d: 로커 피벗 축경.
        bogie_shaft_d: 보기 피벗 축경.
        axle_d: 휠 스터브 액슬 축경.
    """

    # ---- 하중 조건 ----
    robot_mass_kg: float = 60.0
    dynamic_factor: float = 3.0

    # ---- 휠 ----
    wheel_d: int = 250

    # ---- 측면도 좌표 (XZ) ----
    x_front: float = 450.0
    x_mid: float = 0.0
    x_rear: float = -450.0
    z_rocker_pivot: float = 300.0
    z_bogie_pivot: float = 200.0
    z_diff_bar: float = 750.0

    # ---- 프레임 / 횡방향 (Y) ----
    frame_half_w: float = 400.0   # 프레임 외측면
    frame_len: float = 1000.0
    frame_profile: float = 40.0   # HFS5-4040
    z_frame_low_rail: float = 300.0
    z_frame_top_rail: float = 700.0
    # 로커 피벗 지지: MISUMI SHF16 플랜지형 샤프트 서포트 (표준품, 2홀 M5 @40, 레일 한 슬롯 체결)
    shf_flange_t: float = 8.0     # SHF16 플랜지 두께 T
    shf_boss_len: float = 16.0    # SHF16 클램프 보스 길이 L
    shf_flange_w: float = 50.0    # SHF16 플랜지 폭 W
    shf_hole_pitch: float = 40.0  # SHF16 볼트 홀 간격 B
    block_gap: float = 2.0        # 서포트-로커 플레이트 간 스러스트 와셔 간극
    plate_gap: float = 6.0        # 로커-보기 플레이트 간 와셔 간격
    wheel_gap: float = 10.0       # 보기 플레이트-휠 내측면 간격

    # ---- 단면 ----
    plate_t: float = 12.0
    arm_w: float = 60.0

    # ---- 차동 ----
    # 링크 로드는 X(전후) 방향으로 배치: 바 끝단의 원호 운동에서 Y 변위는
    # 2차 미소량이 되어 로드엔드(볼조인트)로 흡수된다.
    # 바 반길이 = 로커 플레이트 중심 Y (유도), 바 피벗 X = 로커 피벗 X - 링크 길이
    diff_link_len: float = 150.0
    diff_bar_w: float = 30.0
    diff_bar_t: float = 12.0
    link_rod_d: float = 8.0

    # ---- 축 ----
    rocker_shaft_d: int = 16
    bogie_shaft_d: int = 12
    axle_d: int = 16

    # ---- 재료 (Al 6061-T6) ----
    yield_mpa: float = 276.0
    density_g_cm3: float = 2.70

    derived: Dict[str, float] = field(default_factory=dict, init=False)

    # ------------------------------------------------------------------
    def __post_init__(self) -> None:
        """유도 치수를 계산한다."""
        self.derived = {}
        self.derived["x_bogie_pivot"] = 0.5 * (self.x_mid + self.x_rear)
        # 3륜 등하중 조건: x_p = (x_f + 2·x_b) / 3
        self.derived["x_rocker_pivot_equal_load"] = (
            self.x_front + 2.0 * self.derived["x_bogie_pivot"]
        ) / 3.0
        self.derived["wheel_r"] = self.wheel_d / 2.0
        self.derived["z_axle"] = self.wheel_d / 2.0

        # Y 적층 (좌측, 안쪽→바깥쪽)
        y0 = self.frame_half_w
        y_block_out = y0 + self.pivot_block_t
        y_rocker_in = y_block_out + self.block_gap
        y_rocker_out = y_rocker_in + self.plate_t
        y_bogie_in = y_rocker_out + self.plate_gap
        y_bogie_out = y_bogie_in + self.plate_t
        y_wheel_in = y_bogie_out + self.wheel_gap
        y_wheel_c = y_wheel_in + WHEELS[self.wheel_d]["width"] / 2.0
        self.derived.update(
            y_rocker_c=0.5 * (y_rocker_in + y_rocker_out),
            y_bogie_c=0.5 * (y_bogie_in + y_bogie_out),
            y_wheel_c=y_wheel_c,
            y_rocker_in=y_rocker_in,
            y_bogie_in=y_bogie_in,
            y_wheel_in=y_wheel_in,
            track_width=2.0 * y_wheel_c,
        )
        self.derived["diff_bar_half_len"] = self.derived["y_rocker_c"]
        self.derived["x_diff_pivot"] = (
            self.derived["x_rocker_pivot_equal_load"] - self.diff_link_len
        )

    # ------------------------------------------------------------------
    # 기하
    # ------------------------------------------------------------------
    @property
    def pivot_block_t(self) -> float:
        """피벗 서포트(SHF16)가 레일 면에서 돌출하는 길이 = 플랜지 + 보스."""
        return self.shf_flange_t + self.shf_boss_len

    @property
    def x_rocker_pivot(self) -> float:
        """로커 차체 피벗 X — 등하중 위치를 채택."""
        return self.derived["x_rocker_pivot_equal_load"]

    @property
    def x_bogie_pivot(self) -> float:
        """보기 피벗 X — 중간/후방 휠 중점."""
        return self.derived["x_bogie_pivot"]

    def side_points(self) -> Dict[str, Point2]:
        """측면도(XZ) 상의 주요 조인트 좌표를 반환한다."""
        z_axle = self.derived["z_axle"]
        return {
            "P": (self.x_rocker_pivot, self.z_rocker_pivot),
            "F": (self.x_front, z_axle),
            "M": (self.x_mid, z_axle),
            "R": (self.x_rear, z_axle),
            "B": (self.x_bogie_pivot, self.z_bogie_pivot),
            "U": (self.x_rocker_pivot, self.z_diff_bar),
        }

    def link_lengths(self) -> Dict[str, float]:
        """각 링크의 중심간 길이를 반환한다."""
        p = self.side_points()

        def dist(a: Point2, b: Point2) -> float:
            return math.hypot(a[0] - b[0], a[1] - b[1])

        return {
            "rocker_front": dist(p["P"], p["F"]),
            "rocker_rear": dist(p["P"], p["B"]),
            "rocker_upright": dist(p["P"], p["U"]),
            "bogie_front": dist(p["B"], p["M"]),
            "bogie_rear": dist(p["B"], p["R"]),
            "diff_link": self.diff_link_len,
            "diff_bar": 2.0 * self.derived["diff_bar_half_len"],
            "wheelbase": self.x_front - self.x_rear,
        }

    # ------------------------------------------------------------------
    # 정역학
    # ------------------------------------------------------------------
    def static_wheel_loads(self) -> Dict[str, float]:
        """평지 정지 상태의 휠별 수직 하중(N)을 계산한다.

        가정: 마찰 없는 핀 조인트, 수평력 0 → 피벗 높이는 결과에 무관.
        좌우 대칭이므로 차동 바에는 하중이 걸리지 않는다.

        Returns:
            {"front", "mid", "rear", "per_side"} 하중(N).
        """
        w_side = self.robot_mass_kg * 9.81 / 2.0
        x_f, x_m, x_r = self.x_front, self.x_mid, self.x_rear
        x_b, x_p = self.x_bogie_pivot, self.x_rocker_pivot

        # 보기: N_m (x_m - x_b) = N_r (x_b - x_r)
        k = (x_b - x_r) / (x_m - x_b)           # N_m / N_r
        # 로커: N_f (x_f - x_p) = (N_m + N_r)(x_p - x_b)
        # N_f + N_m + N_r = w_side
        n_r = w_side / (1.0 + k + (1.0 + k) * (x_p - x_b) / (x_f - x_p))
        n_m = k * n_r
        n_f = w_side - n_m - n_r
        return {"front": n_f, "mid": n_m, "rear": n_r, "per_side": w_side}

    def safety_factors(self) -> Dict[str, float]:
        """베어링·휠·플레이트의 안전율을 계산한다 (동하중 계수 적용)."""
        loads = self.static_wheel_loads()
        n_max = max(loads["front"], loads["mid"], loads["rear"])
        n_dyn = n_max * self.dynamic_factor
        side_dyn = loads["per_side"] * self.dynamic_factor
        bogie_dyn = (loads["mid"] + loads["rear"]) * self.dynamic_factor

        sf: Dict[str, float] = {}
        # 베어링은 쌍으로 사용 → 하중 1/2
        sf["rocker_bearing"] = BEARINGS[self.rocker_shaft_d]["load"] / (side_dyn / 2)
        sf["bogie_bearing"] = BEARINGS[self.bogie_shaft_d]["load"] / (bogie_dyn / 2)
        sf["wheel"] = WHEELS[self.wheel_d]["load"] / n_dyn

        # 로커 전방 암: 캔틸레버 굽힘 (면내)
        lens = self.link_lengths()
        m_in = n_dyn * lens["rocker_front"]                # N·mm
        i_in = self.plate_t * self.arm_w ** 3 / 12.0
        sigma_in = m_in * (self.arm_w / 2) / i_in
        sf["rocker_bending_inplane"] = self.yield_mpa / sigma_in

        # 면외(횡력 0.5·N) 굽힘 — 플레이트 두께 방향이 약축
        f_lat = 0.5 * n_dyn
        m_out = f_lat * lens["rocker_front"]
        i_out = self.arm_w * self.plate_t ** 3 / 12.0
        sigma_out = m_out * (self.plate_t / 2) / i_out
        sf["rocker_bending_outplane"] = self.yield_mpa / sigma_out

        # 로커 피벗 축: SHF16 클램프 중심 → 로커 베어링 쌍 중심 캔틸레버 (SUS304)
        y_clamp = self.frame_half_w + self.shf_flange_t + self.shf_boss_len / 2
        brg = BEARINGS[self.rocker_shaft_d]
        y_brg = self.derived["y_rocker_in"] + (2 * brg["width"] + 6) / 2
        z_shaft = math.pi * self.rocker_shaft_d ** 3 / 32.0
        sf["rocker_shaft_at_shf16"] = 205.0 / (side_dyn * (y_brg - y_clamp) / z_shaft)

        # 전방 스터브 액슬 (SUS304, σy≈205 MPa)
        overhang = self.derived["y_wheel_c"] - self.derived["y_rocker_c"]
        z_axle = math.pi * self.axle_d ** 3 / 32.0
        sf["front_axle"] = 205.0 / (n_dyn * overhang / z_axle)
        return sf

    # ------------------------------------------------------------------
    # 기구학: 차동 바 연동
    # ------------------------------------------------------------------
    def upright_top(self, theta: float) -> Tuple[float, float]:
        """로커가 피벗 기준 theta(rad, 전방 상승 +) 회전했을 때 U점 (X, Z)."""
        length = self.link_lengths()["rocker_upright"]
        # 로커 전방이 올라가면(θ>0) 상방 암 끝은 후방(-X)으로 이동
        return (
            self.x_rocker_pivot - length * math.sin(theta),
            self.z_rocker_pivot + length * math.cos(theta),
        )

    def _link_residual(self, theta: float, phi: float, side: float) -> float:
        """링크 길이 구속 잔차. side=+1 좌측, -1 우측."""
        ux, uz = self.upright_top(theta)
        uy = side * self.derived["y_rocker_c"]
        half = self.derived["diff_bar_half_len"]
        # 차동 바: Z축 회전 phi (위에서 봤을 때 반시계 +). 좌측 끝(+Y)은 -X 로 이동.
        bx = self.derived["x_diff_pivot"] - side * half * math.sin(phi)
        by = side * half * math.cos(phi)
        bz = self.z_diff_bar
        return math.dist((ux, uy, uz), (bx, by, bz)) - self.diff_link_len

    def differential_response(self, theta_left: float) -> Tuple[float, float]:
        """좌측 로커 각도가 주어졌을 때 차동 바 회전각과 우측 로커 각도를 푼다.

        Args:
            theta_left: 좌측 로커 회전각(rad).

        Returns:
            (phi_bar, theta_right) — 차동 바 Z축 회전각, 우측 로커 각도(rad).
        """
        up_len = self.link_lengths()["rocker_upright"]
        half = self.derived["diff_bar_half_len"]
        # 1차 근사 초기값: 바 끝단 X 변위 ≈ 상방 암 끝 X 변위
        ratio = max(-1.0, min(1.0, up_len * math.sin(theta_left) / half))
        phi0 = math.asin(ratio)
        phi = _bisect(lambda p: self._link_residual(theta_left, p, +1.0),
                      -math.pi / 2, math.pi / 2, x0=phi0)
        theta_r = _bisect(lambda t: self._link_residual(t, phi, -1.0),
                          -math.pi / 2, math.pi / 2, x0=-theta_left)
        return phi, theta_r

    def body_pitch(self, theta_left: float, theta_right: float) -> float:
        """차체 피치 = 좌우 로커 각도의 평균 (차동 특성)."""
        return 0.5 * (theta_left + theta_right)


def _bisect(fn, lo: float, hi: float, tol: float = 1e-9,
            x0: float = 0.0) -> float:
    """fn(x)=0 의 근을 찾는다. x0 에서 바깥쪽으로 부호 변화를 탐색한 뒤 이분한다.

    링크 길이 잔차는 V자형(비단조, 근 2개)이므로 초기 추정값 x0 에
    가장 가까운 근(물리적으로 연속인 분기)을 선택한다.
    """
    step = (hi - lo) / 720.0
    if abs(fn(x0)) < tol:
        return x0
    bracket = None
    for i in range(1, 361):
        for sgn in (+1.0, -1.0):
            x_in, x_out = x0 + sgn * (i - 1) * step, x0 + sgn * i * step
            if x_out < lo or x_out > hi:
                continue
            if (fn(x_in) < 0) != (fn(x_out) < 0):
                bracket = (min(x_in, x_out), max(x_in, x_out))
                break
        if bracket:
            break
    if bracket is None:
        raise ValueError("링크 길이 구속을 만족하는 해가 없음")
    a, b = bracket
    f_a = fn(a)
    for _ in range(100):
        m = 0.5 * (a + b)
        f_m = fn(m)
        if abs(f_m) < tol:
            return m
        if (f_a < 0) == (f_m < 0):
            a, f_a = m, f_m
        else:
            b = m
    return 0.5 * (a + b)


def report(p: SuspensionParams) -> str:
    """설계 요약 문자열을 생성한다."""
    pts = p.side_points()
    lens = p.link_lengths()
    loads = p.static_wheel_loads()
    sf = p.safety_factors()
    lines = ["=== Rocker-Bogie 설계 요약 (단위 mm, N) ===", "", "[조인트 좌표 XZ, 좌측 Y]"]
    y_map = {"P": p.derived["y_rocker_c"], "U": p.derived["y_rocker_c"],
             "B": p.derived["y_bogie_c"], "F": p.derived["y_wheel_c"],
             "M": p.derived["y_wheel_c"], "R": p.derived["y_wheel_c"]}
    for k, (x, z) in pts.items():
        lines.append(f"  {k}: X={x:7.1f}  Y={y_map[k]:6.1f}  Z={z:6.1f}")
    lines.append(f"  D: X={p.derived['x_diff_pivot']:7.1f}  Y=   0.0  "
                 f"Z={p.z_diff_bar:6.1f}   (차동 바 피벗, Z축 회전)")
    lines += ["", "[링크 길이]"]
    lines += [f"  {k:15s}: {v:7.1f}" for k, v in lens.items()]
    lines += [f"  track_width    : {p.derived['track_width']:7.1f}", "", "[정하중 (한쪽)]"]
    lines += [f"  {k:9s}: {v:7.1f} N" for k, v in loads.items()]
    lines += ["", f"[안전율 (동하중 ×{p.dynamic_factor:.0f})]"]
    lines += [f"  {k:24s}: {v:5.1f}" for k, v in sf.items()]
    return "\n".join(lines)


if __name__ == "__main__":
    print(report(SuspensionParams()))
