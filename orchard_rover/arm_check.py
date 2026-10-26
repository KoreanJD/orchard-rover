"""로봇팔 권장 사양 점검 — 토크, 스테퍼 속도-토크, 처짐, 전도, 질량.

단위: mm, kg, N, N·m. 팔은 수평 완전 전개(최악 자세), 툴 3 kg.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

G = 9.81
E_AL = 69_000.0        # MPa (6061-T6)
I_HFS5_3030 = 28_000.0  # mm^4, HFS5-3030 단면 2차 모멘트 (카탈로그 Ix≈2.8 cm^4)


@dataclass
class Segment:
    """팔을 어깨에서 팁까지 직렬 질점으로 모델링한 요소."""
    name: str
    length: float   # 이 요소의 길이 (mm), 질점은 중앙
    mass: float     # kg (링크 + 그 끝에 달린 액추에이터 포함 시 별도 요소로 분리)


# 어깨 축 기준 직렬 질량 분포 (수평 전개)
SEGMENTS: List[Segment] = [
    Segment("upper_arm_link", 500, 1.6),    # HFS5-3030×2 (0.51 kg/m×1.0) + 조인트 플레이트
    Segment("elbow_actuator", 0, 2.4),       # NEMA23 1.3 + 사이클로이드 1:45 1.0 + 하우징
    Segment("j4_roll_actuator", 40, 0.9),    # 6축: 전완 롤 NEMA17 + 1:36 유성 (팔꿈치 직후, 질량 안쪽)
    Segment("forearm_link", 410, 1.0),       # Al 튜브 Ø50×2t (롤 회전부)
    Segment("wrist_j5_j6", 0, 1.6),          # NEMA17×2 + 유성 ×2 + 블록 + ISO 플랜지
    Segment("wrist_links", 100, 0.4),
    Segment("tool", 0, 3.0),                 # 가반하중 (툴 CG는 플랜지 +80 로 가정)
]
TOOL_CG_OFFSET = 80.0


def joint_static_torques() -> Dict[str, float]:
    """어깨(J2)·팔꿈치(J3)·손목 피치(J4) 축의 정적 중력 토크(N·m)."""
    x = 0.0
    masses: List[Tuple[float, float]] = []   # (x_cg from shoulder, mass)
    for s in SEGMENTS:
        if s.name == "tool":
            masses.append((x + TOOL_CG_OFFSET, s.mass))
        else:
            masses.append((x + s.length / 2, s.mass))
        x += s.length
    reach = x + TOOL_CG_OFFSET
    x_elbow = 500.0
    x_wrist = 950.0

    def torque_about(x0: float) -> float:
        return sum(max(0.0, xc - x0) * m * G for xc, m in masses) / 1000.0

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
    """상완(사다리형 2본, 간격 60)과 전완(1본) 캔틸레버 처짐, 툴 3 kg + 자중."""
    p_tip = 3.0 * G + 1.8 * G           # 툴 + 손목부
    # 상완: 2본 평행, 간격 d=60 → I = 2(I0 + A·(d/2)^2), A≈ 3.2 cm^2 = 320 mm^2
    i_upper = 2 * (I_HFS5_3030 + 320.0 * 30.0 ** 2)
    i_fore = I_HFS5_3030
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


def tipover(base_mass: float = 60.0, arm_mass: float = 12.0, tool: float = 3.0,
            half_track: float = 506.0, base_cg_z: float = 450.0,
            arm_base_z: float = 950.0, slope_deg: float = 15.0) -> Dict[str, float]:
    """측면 완전 전개 + 횡경사에서 전복 마진."""
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
