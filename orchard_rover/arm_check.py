"""로봇팔 권장 사양 점검 — 토크, 스테퍼 속도-토크, 처짐, 전도, 질량.

단위: mm, kg, N, N·m. 팔은 수평 완전 전개(최악 자세), 툴 3 kg.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

G = 9.81
E_AL = 69_000.0        # MPa (6061-T6)
YIELD_AL = 276.0       # MPa (6061-T6)
I_HFS6_3030 = 28_300.0  # mm^4, MISUMI HFS6-3030 단면 2차 모멘트 (카탈로그 2.83×10⁴)
A_HFS6_3030 = 333.0     # mm², 단면적 (0.9 kg/m)
I_FORE_TUBE = math.pi / 64 * (50.0 ** 4 - 46.0 ** 4)   # mm^4, 전완 Al 튜브 Ø50×2t
DYN = 1.5              # 동적 계수 (관절 액추에이터 선정과 동일)


@dataclass
class Segment:
    """팔을 어깨에서 팁까지 직렬 질점으로 모델링한 요소."""
    name: str
    length: float   # 이 요소의 길이 (mm), 질점은 중앙
    mass: float     # kg (링크 + 그 끝에 달린 액추에이터 포함 시 별도 요소로 분리)


# 어깨 축 기준 직렬 질량 분포 (수평 전개)
SEGMENTS: List[Segment] = [
    Segment("upper_arm_link", 500, 1.6),    # HFS6-3030×2 (0.9 kg/m × 0.33 m ×2 = 0.59) + 측판 2장
    Segment("elbow_actuator", 0, 2.4),       # NEMA23 1.3 + 사이클로이드 1:45 1.0 + 하우징
    Segment("j4_roll_actuator", 40, 0.9),    # 6축: 전완 롤 NEMA17 + 1:36 유성 (팔꿈치 직후, 질량 안쪽)
    Segment("forearm_link", 410, 1.0),       # Al 튜브 Ø50×2t (롤 회전부)
    Segment("wrist_j5_j6", 0, 1.6),          # NEMA17×2 + 유성 ×2 + 블록 + ISO 플랜지
    Segment("wrist_links", 100, 0.4),
    Segment("tool", 0, 3.0),                 # 가반하중 (툴 CG는 플랜지 +80 로 가정)
]
TOOL_CG_OFFSET = 80.0


def _point_masses() -> List[Tuple[float, float]]:
    """SEGMENTS 를 어깨 축 기준 (x_cg, mass) 질점 목록으로."""
    x = 0.0
    masses: List[Tuple[float, float]] = []
    for s in SEGMENTS:
        if s.name == "tool":
            masses.append((x + TOOL_CG_OFFSET, s.mass))
        else:
            masses.append((x + s.length / 2, s.mass))
        x += s.length
    return masses


def torque_about(x0: float) -> float:
    """수평 전개 자세에서 x0 (어깨 축 기준, mm) 보다 바깥 질량의 중력 모멘트 (N·m)."""
    return sum(max(0.0, xc - x0) * m * G for xc, m in _point_masses()) / 1000.0


def weight_beyond(x0: float) -> float:
    """x0 보다 바깥 질량의 무게 (N)."""
    return sum(m * G for xc, m in _point_masses() if xc > x0)


def joint_static_torques() -> Dict[str, float]:
    """어깨(J2)·팔꿈치(J3)·손목 피치(J4) 축의 정적 중력 토크(N·m)."""
    reach = sum(s.length for s in SEGMENTS) + TOOL_CG_OFFSET
    x_elbow = 500.0
    x_wrist = 950.0

    return {
        "reach_to_tool_cg": reach,
        "J2_shoulder": torque_about(0.0),
        "J3_elbow": torque_about(x_elbow),
        "J4_wrist": torque_about(x_wrist),
        "arm_mass_excl_tool": sum(s.mass for s in SEGMENTS if s.name != "tool"),
    }


def stepper_torque_at_rpm(holding: float, rpm: float, v_bus: float = 36.0,
                          corner_rpm_24v: float = 300.0) -> float:
    """스테퍼 속도-토크 근사: 코너 rpm 이하 일정, 이상은 반비례.

    코너 속도는 버스 전압에 비례(24 V 기준 ~300 rpm).
    """
    corner = corner_rpm_24v * v_bus / 24.0
    return holding if rpm <= corner else holding * corner / rpm


def joint_capacity(holding: float, ratio: float, joint_deg_s: float,
                   eta: float = 0.8) -> Tuple[float, float]:
    """관절 속도에서의 출력 토크와 그때 모터 rpm."""
    motor_rpm = joint_deg_s / 360.0 * 60.0 * ratio
    return stepper_torque_at_rpm(holding, motor_rpm) * ratio * eta, motor_rpm


def tip_deflection() -> Dict[str, float]:
    """상완(HFS6-3030 사다리 2본, 간격 60)과 전완(Al 튜브 Ø50×2t) 캔틸레버 처짐, 툴 3 kg + 자중."""
    p_tip = 3.0 * G + 1.8 * G           # 툴 + 손목부
    # 상완: 2본 평행, 간격 d=60 → I = 2(I0 + A·(d/2)^2), A≈ 3.2 cm^2 = 320 mm^2
    i_upper = 2 * (I_HFS6_3030 + A_HFS6_3030 * 30.0 ** 2)
    i_fore = I_FORE_TUBE
    l_u, l_f = 500.0, 450.0
    # 전완 자체 처짐 (끝 하중) + 상완 처짐이 전완 끝까지 기울기로 전파
    m_fore_end = p_tip
    d_fore = m_fore_end * l_f ** 3 / (3 * E_AL * i_fore)
    load_at_elbow = p_tip + 2.2 * G + 1.1 * G
    moment_at_elbow = p_tip * l_f / 1.0      # N·mm
    d_upper_end = load_at_elbow * l_u ** 3 / (3 * E_AL * i_upper) \
        + moment_at_elbow * l_u ** 2 / (2 * E_AL * i_upper)
    slope_upper = load_at_elbow * l_u ** 2 / (2 * E_AL * i_upper) \
        + moment_at_elbow * l_u / (E_AL * i_upper)
    return {
        "forearm_self": d_fore,
        "upper_arm_end": d_upper_end,
        "tip_total": d_fore + d_upper_end + slope_upper * (l_f + 100),
        "I_upper": i_upper, "I_fore": i_fore,
    }


def plate_strength() -> Dict[str, Dict[str, float]]:
    """2026-10-29 형상 변경 부품의 응력·안전율 (수평 완전 전개, 툴 3 kg, 동적 ×1.5, 항복 기준).

    - J1 브라켓 (10t, J2 어댑터 겸용): J2 축 하중을 하단 단면에서 면내(J2 토크 + 수평력×높이)
      + 면외(수직력 × 상완 하중면 Y 오프셋) 굽힘으로 합산. J2 토크는 액추에이터 최대 출력.
    - 전완판 (10t, J3 어댑터 겸용): J4 모터 창 단면(상하 띠 2개)의 굽힘.
    - J4 하우징: 롤 베어링 시트 (가장 얇은 벽) 단면의 굽힘 + J4 출력 토크 비틀림 (von Mises).
    - J5 축 Ø12: 손목 이후 하중의 절반, 블록 면 → 지지 베어링 중심 캔틸레버.
    """
    from orchard_rover.arm_assembly import PLATE_T, UPPER_LEN, upper_plate_y
    from orchard_rover.base_joint import BaseJointParams
    from orchard_rover.shoulder_joint import ADAPTER_T, ShoulderParams, gas_spring_arm_anchor, gas_spring_state
    from orchard_rover.wrist_joint import NEMA17, WristParams

    out: Dict[str, Dict[str, float]] = {}
    b, p2, w = BaseJointParams(), ShoulderParams(), WristParams()

    # --- J1 브라켓 ---
    weight = weight_beyond(-1.0)
    (bx, bz), (ax, az) = p2.gs_base, gas_spring_arm_anchor(p2, 0.0)
    L, f0, _ = gas_spring_state(p2, 0.0)
    f_spring = (f0 * (ax - bx) / L, f0 * (az - bz) / L)                    # 상완 앵커에 작용 (x, z)
    cases = [(f_spring[0], f_spring[1] - weight), (0.0, -weight)]          # 스프링 작동 / 상실
    fx, fz = max(cases, key=lambda f: math.hypot(*f))
    fx, fz = DYN * abs(fx), DYN * abs(fz)
    t_j2 = p2.motor_holding * 0.5 * p2.cyc.ratio * 0.8 * 1000.0           # N·mm, 액추에이터 최대
    y_off = abs(upper_plate_y(p2) + PLATE_T / 2) - b.bracket_t / 2         # 상완 하중면 ↔ 브라켓 중면
    m_in = t_j2 + fx * b.shoulder_rise
    m_out = fz * y_off
    z_in = b.bracket_t * b.bracket_w ** 2 / 6

    def out_of_plane(depth: float) -> Tuple[float, float]:
        """판 (Y −t..0) + 수직 거셋 2장 (Y 0..depth) T 단면의 X 축 굽힘 (I, 최외단 거리)."""
        a1, y1 = b.bracket_w * b.bracket_t, -b.bracket_t / 2
        a2, y2 = 2 * b.gusset_t * depth, depth / 2
        yc = (a1 * y1 + a2 * y2) / (a1 + a2)
        i = (b.bracket_w * b.bracket_t ** 3 / 12 + a1 * (y1 - yc) ** 2
             + 2 * b.gusset_t * depth ** 3 / 12 + a2 * (y2 - yc) ** 2)
        return i, max(yc + b.bracket_t, depth - yc)

    i0, c0 = out_of_plane(b.gusset_depth)
    sig = m_in / z_in + m_out * c0 / i0
    # 면외 굽힘 (터릿 상면 → J2 축 캔틸레버, 끝 모멘트, 거셋 깊이는 높이에 따라 선형 감소):
    # 판 상단이 Y 로 휘고 X 축 둘레로 기운다. X 축(팔 방향) 둘레 회전이라 팔 끝 변위는
    # 리치가 아니라 J2 축 높이·Y 오프셋 거리에 비례.
    n, h = 400, b.shoulder_rise
    tilt = tip_y = 0.0
    for k in range(n):
        z = (k + 0.5) * h / n
        depth = max(0.0, b.gusset_depth * (1 - z / b.gusset_h))
        tilt += m_out / (E_AL * out_of_plane(depth)[0]) * h / n
        tip_y += tilt * h / n
    tip = tip_y + tilt * math.hypot(h, y_off)
    out["J1_bracket"] = {"sigma_MPa": sig, "SF": YIELD_AL / sig, "M_inplane_Nm": m_in / 1000,
                         "M_outplane_Nm": m_out / 1000, "tilt_mrad": tilt * 1000, "tip_mm": tip}

    # --- 전완판 J4 모터 창 단면 ---
    x_h = UPPER_LEN + w.j4_offset
    x_win = x_h + 10 - 48 - 2
    m = DYN * torque_about(x_win) * 1000.0
    win, h_out = NEMA17 / 2 + 2, 45.0
    h = h_out - win
    d = (h_out + win) / 2
    i_sec = 2 * (ADAPTER_T * h ** 3 / 12 + ADAPTER_T * h * d ** 2)
    sig = m / (i_sec / h_out)
    out["fore_plate_window"] = {"sigma_MPa": sig, "SF": YIELD_AL / sig, "M_Nm": m / 1000}

    # --- J4 하우징 베어링 시트 단면 ---
    x_seat = x_h + w.j4_housing_len - 9.0
    m = DYN * torque_about(x_seat) * 1000.0
    t = 11.5e3                                                             # J4 출력 토크 N·mm
    D, d_in = w.j4_housing_d, 65.0
    z_b = math.pi * (D ** 4 - d_in ** 4) / (32 * D)
    sig_b, tau = m / z_b, t / (2 * z_b)
    sig = math.sqrt(sig_b ** 2 + 3 * tau ** 2)
    out["j4_housing_seat"] = {"sigma_MPa": sig, "SF": YIELD_AL / sig, "M_Nm": m / 1000, "wall_mm": (D - d_in) / 2}

    # --- J5 축 Ø12 --- (손목 질점 x=950 부터 바깥: 손목 액추에이터·링크·툴, 두 축이 반씩)
    f = DYN * weight_beyond(949.0) / 2
    lever = 2.0 + 4.0                                                      # 블록 면 → 6001 폭 중앙
    sig = f * lever / (math.pi * 12.0 ** 3 / 32)
    out["J5_stub_shaft"] = {"sigma_MPa": sig, "SF": 205.0 / sig, "F_N": f}   # SUS304 σy 205
    return out


def tipover(base_mass: float = 60.0, arm_mass: float = 12.0, tool: float = 3.0,
            half_track: float = None, base_cg_z: float = 450.0,
            arm_base_z: float = 950.0, slope_deg: float = 15.0) -> Dict[str, float]:
    """측면 완전 전개 + 횡경사에서 전복 마진. half_track 기본값 = 서스펜션 휠 중심 Y."""
    if half_track is None:
        from orchard_rover.rocker_bogie_params import SuspensionParams
        half_track = SuspensionParams().derived["y_wheel_c"]
    reach = 1050.0
    # 팔 CG 는 전개 시 어깨에서 ~0.45 m, 툴은 1.05 m (모두 측방)
    m_over = (tool * reach + arm_mass * 450.0) * G / 1000.0
    total = base_mass + arm_mass + tool
    # 경사: 차체 CG 높이 위에서 횡 분력 모멘트
    z_cg = (base_mass * base_cg_z + (arm_mass + tool) * arm_base_z) / total
    m_slope = total * G * math.sin(math.radians(slope_deg)) * z_cg / 1000.0
    m_restore = total * G * math.cos(math.radians(slope_deg)) * half_track / 1000.0
    return {"overturning_arm": m_over, "slope_moment": m_slope,
            "restoring": m_restore,
            "margin_flat": m_restore / m_over,
            "margin_slope": m_restore / (m_over + m_slope)}


if __name__ == "__main__":
    t = joint_static_torques()
    print("=== 정적 토크 (수평 전개, 툴 3 kg) ===")
    for k, v in t.items():
        print(f"  {k:22s}: {v:8.1f}")

    print("\n=== 스테퍼 속도-토크 (36 V) ===")
    joints = {
        "J2 NEMA23 3.0N·m 1:50": (3.0, 50, t["J2_shoulder"]),
        "J3 NEMA23 2.0N·m 1:30": (2.0, 30, t["J3_elbow"]),
        "J4 NEMA17 0.65N·m 1:20": (0.65, 20, t["J4_wrist"]),
    }
    for name, (hold, ratio, need) in joints.items():
        for spd in (10, 20, 30, 45):
            cap, rpm = joint_capacity(hold, ratio, spd)
            ok = "OK " if cap >= need * 1.5 else "부족"
            print(f"  {name:24s} {spd:2d}°/s  motor {rpm:5.0f} rpm  "
                  f"출력 {cap:6.1f} N·m  필요×1.5 {need*1.5:6.1f}  {ok}")

    d = tip_deflection()
    print("\n=== 처짐 ===")
    for k, v in d.items():
        print(f"  {k:16s}: {v:10.1f}")

    print("\n=== 전도 ===")
    for k, v in tipover().items():
        print(f"  {k:16s}: {v:8.2f}")

    print("\n=== 부품 강도 (동적 ×1.5, 항복 기준) ===")
    for part, r in plate_strength().items():
        print(f"  {part:18s} " + "  ".join(f"{k} {v:7.1f}" for k, v in r.items()))
