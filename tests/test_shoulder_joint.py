"""J2 어깨 조인트(사이클로이드 감속기 + 가스 스프링) 검증 테스트."""

from __future__ import annotations

import math

import pytest

from orchard_rover.shoulder_joint import CycloidParams, ShoulderParams, gas_spring_balance


@pytest.fixture(scope="module")
def p() -> ShoulderParams:
    return ShoulderParams()


def test_cycloid_ratio_and_pin_spacing(p: ShoulderParams) -> None:
    """감속비 = 핀 − 1, e < R/N, 핀 간격 ≥ Rr (핀 겹침 방지 — 초기 설계 오류였음)."""
    c = p.cyc
    assert c.ratio == 50
    assert c.ecc < c.r_pin_circle / c.pins
    assert c.pin_gap >= c.r_pin


def test_profile_has_no_self_intersection(p: ShoulderParams) -> None:
    """프로파일 반경이 [R−e−Rr, R+e−Rr] 범위를 벗어나지 않아야 한다 (루프 없음)."""
    from orchard_rover.shoulder_joint import cycloid_profile

    c = p.cyc
    rads = [math.hypot(x, y) for x, y in cycloid_profile(c, 2040)]
    assert min(rads) == pytest.approx(c.r_pin_circle - c.ecc - c.r_pin, abs=0.05)
    assert max(rads) == pytest.approx(c.r_pin_circle + c.ecc - c.r_pin, abs=0.05)


def test_output_hole_equals_pin_plus_2e(p: ShoulderParams) -> None:
    c = p.cyc
    assert c.d_out_hole == pytest.approx(c.d_out_pin + 2 * c.ecc)


def test_output_pins_inside_disc_and_clear_of_cam(p: ShoulderParams) -> None:
    c = p.cyc
    assert c.r_out_pcd + c.d_out_hole / 2 < c.disc_od / 2 - 8
    assert c.r_out_pcd - c.d_out_hole / 2 > c.d_cam_bearing / 2 + 4


def test_gas_spring_balance(p: ShoulderParams) -> None:
    """수평에서 중력 토크의 35~50% 보상, 스트로크 ≤ 120 (상용 가스 스프링 범위)."""
    g = gas_spring_balance(p)
    assert 0.35 <= g["balance_ratio"] <= 0.5
    assert g["stroke"] <= 120.0
    assert g["L_min"] > 150.0           # 압축 상태 최소 길이 (실린더+로드)


def test_residual_torque_within_actuator(p: ShoulderParams) -> None:
    """잔류 토크 ×1.5 ≤ NEMA23 4.0 N·m ×50% ×50 ×0.8 = 80 N·m."""
    g = gas_spring_balance(p)
    assert g["residual_horizontal"] * 1.5 <= 80.0


cq = pytest.importorskip("cadquery")


def test_disc_is_valid_solid(p: ShoulderParams) -> None:
    from orchard_rover.shoulder_joint import make_cycloid_disc

    c = p.cyc
    d = make_cycloid_disc(c).val()
    assert d.isValid()
    expected = math.pi * (c.disc_od / 2) ** 2 * c.disc_t
    assert 0.8 * expected < d.Volume() < 1.1 * expected


def test_elbow_params_valid() -> None:
    """J3: 1:40, 핀 간격 확보, 출력 홀이 디스크 안쪽."""
    from orchard_rover.shoulder_joint import elbow_params

    p3 = elbow_params()
    c = p3.cyc
    assert c.ratio == 45 and c.pin_gap >= c.r_pin
    assert c.r_out_pcd + c.d_out_hole / 2 < c.disc_od / 2 - 8
    # 실사용 출력 ≥ 필요 토크 ×1.5
    assert p3.motor_holding * 0.5 * c.ratio * 0.8 >= p3.torque_static * 1.5


def test_arm_assembly_axes() -> None:
    """J2 축 원점, J3 축 X=500, 전완 끝 X≈950, 부품 수 > 20."""
    from orchard_rover.arm_assembly import UPPER_LEN, FORE_LEN, build_arm

    asm = build_arm()
    comp = asm.toCompound()
    assert len(comp.Solids()) > 20
    bb = comp.BoundingBox()
    assert bb.xmax == pytest.approx(UPPER_LEN + FORE_LEN + 100, abs=5)   # ISO 플랜지 면
    names = {ch.name for ch in asm.children}
    assert {"J2_output_flange", "J3_output_flange", "upper_plate_J2", "upper_plate_J3",
            "j4_housing", "forearm_tube", "j5_yoke", "wrist_block", "j6_gear", "iso_flange"} <= names


def test_full_robot_integration() -> None:
    """J1 위치가 차동 바 스윙 영역 밖, J2 축 높이 ≈ 950, 전체 조립 생성."""
    from orchard_rover.base_joint import BaseJointParams, build_full_robot
    from orchard_rover.rocker_bogie_params import SuspensionParams

    b, p = BaseJointParams(), SuspensionParams()
    swing_half = p.derived["diff_bar_half_len"] * math.sin(math.radians(25)) + 40
    assert b.x_mount - b.plate_x / 2 > p.derived["x_diff_pivot"] + swing_half
    full, info = build_full_robot(0.0)
    assert 930 <= info["z_j2_axis"] <= 980
    assert len(full.toCompound().Solids()) > 60


def test_cable_routing_constraints() -> None:
    """감김 각도 15~330° 유지, 롤 구간 꼬임 길이 ≥ 200 mm/180°, 번들이 Ø20 주름관에 들어감, J3 한계 동기."""
    from orchard_rover.arm_workspace import LIMITS
    from orchard_rover.cable_routing import CAMERA, TOOL, TRUNK, bundle_od, define_routes

    routes = {r.name: r for r in define_routes()}
    for r in routes.values():
        a = r.analyse()                       # wrap 범위 벗어나면 ValueError
        if r.kind == "roll":
            assert a["cable_len"] >= 200.0 * max(abs(x) for x in r.range_deg) / 180.0
    assert bundle_od(TRUNK + TOOL + CAMERA) < 20.0 * 0.7     # 주름관 내경의 70% 충전율 이하
    assert routes["J3_elbow"].range_deg == LIMITS["J3"]
