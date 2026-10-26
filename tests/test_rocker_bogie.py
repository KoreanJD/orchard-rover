"""Rocker-Bogie 서스펜션 설계 검증 테스트.

실행: pytest test_rocker_bogie.py -v
"""

from __future__ import annotations

import math

import pytest

from orchard_rover.rocker_bogie_params import BEARINGS, WHEELS, SuspensionParams


@pytest.fixture(scope="module")
def params() -> SuspensionParams:
    """기본 설계 파라미터."""
    return SuspensionParams()


# ---------------------------------------------------------------------------
# 구조(토폴로지) 검증 — 이전 대화에서 틀렸던 항목들
# ---------------------------------------------------------------------------
def test_front_wheel_on_rocker_and_two_wheels_on_bogie(params: SuspensionParams) -> None:
    """MSL 방식: 전방 휠은 로커, 중간/후방 휠은 보기에 달린다."""
    pts = params.side_points()
    # 보기 피벗은 중간 휠과 후방 휠 사이에 있어야 한다
    assert pts["R"][0] < pts["B"][0] < pts["M"][0]
    # 전방 휠은 보기 피벗보다 전방, 로커 피벗보다 전방
    assert pts["F"][0] > pts["P"][0] > pts["B"][0]


def test_three_body_attachment_points(params: SuspensionParams) -> None:
    """차체 연결점 = 좌/우 로커 피벗 2개 + 차동 바 피벗 1개."""
    d = params.derived
    rocker_pivots = [(params.x_rocker_pivot, +d["y_rocker_c"]),
                     (params.x_rocker_pivot, -d["y_rocker_c"])]
    diff_pivot = (d["x_diff_pivot"], 0.0)
    assert len({*rocker_pivots, diff_pivot}) == 3
    assert diff_pivot[1] == 0.0                       # 차동 피벗은 중심선 상
    assert rocker_pivots[0][1] == -rocker_pivots[1][1]  # 좌우 대칭


def test_bogie_pivot_above_axles(params: SuspensionParams) -> None:
    """보기 피벗은 휠 축보다 높아야 (장애물 등반 시 모멘트 방향) 한다."""
    assert params.z_bogie_pivot > params.derived["z_axle"]
    assert params.z_rocker_pivot > params.z_bogie_pivot


# ---------------------------------------------------------------------------
# 정역학
# ---------------------------------------------------------------------------
def test_equal_static_wheel_loads(params: SuspensionParams) -> None:
    """등하중 피벗 위치에서 3개 휠 하중이 같아야 한다."""
    loads = params.static_wheel_loads()
    assert loads["front"] == pytest.approx(loads["mid"], rel=1e-9)
    assert loads["mid"] == pytest.approx(loads["rear"], rel=1e-9)
    assert loads["front"] + loads["mid"] + loads["rear"] == pytest.approx(loads["per_side"])


def test_total_load_equals_weight(params: SuspensionParams) -> None:
    """양측 하중 합 = 로봇 무게."""
    loads = params.static_wheel_loads()
    assert 2 * loads["per_side"] == pytest.approx(params.robot_mass_kg * 9.81)


@pytest.mark.parametrize("x_pivot_shift, expect_front_lighter", [(-60.0, True), (60.0, False)])
def test_pivot_shift_changes_front_load(x_pivot_shift: float, expect_front_lighter: bool) -> None:
    """로커 피벗을 뒤로 옮기면 전방 휠 하중이 줄어야 한다 (등반 유리)."""
    p = SuspensionParams()
    p.derived["x_rocker_pivot_equal_load"] += x_pivot_shift
    loads = p.static_wheel_loads()
    assert (loads["front"] < loads["mid"]) is expect_front_lighter


def test_safety_factors_above_minimum(params: SuspensionParams) -> None:
    """모든 안전율 ≥ 2.5 (동하중 계수 3 적용 후)."""
    for name, sf in params.safety_factors().items():
        assert sf >= 2.5, f"{name} 안전율 부족: {sf:.2f}"


# ---------------------------------------------------------------------------
# 기구학: 차동 바
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("deg", [-15, -10, -5, 5, 10, 15])
def test_differential_gives_opposite_rotation(params: SuspensionParams, deg: float) -> None:
    """±15° 범위에서 우측 로커 ≈ -좌측 로커 (오차 < 0.5°), 차체 피치 ≈ 0."""
    th_l = math.radians(deg)
    _, th_r = params.differential_response(th_l)
    assert math.degrees(th_r) == pytest.approx(-deg, abs=0.5)
    assert abs(math.degrees(params.body_pitch(th_l, th_r))) < 0.3


def test_differential_link_length_satisfied(params: SuspensionParams) -> None:
    """해에서 양쪽 링크 길이 구속이 만족되어야 한다."""
    th_l = math.radians(12)
    phi, th_r = params.differential_response(th_l)
    assert abs(params._link_residual(th_l, phi, +1.0)) < 1e-6
    assert abs(params._link_residual(th_r, phi, -1.0)) < 1e-6


def test_differential_at_rest_is_zero(params: SuspensionParams) -> None:
    """정지 자세에서 바 회전 0, 우측 0."""
    phi, th_r = params.differential_response(0.0)
    assert abs(phi) < 1e-9 and abs(th_r) < 1e-9


# ---------------------------------------------------------------------------
# 간섭 / 적층
# ---------------------------------------------------------------------------
def test_lateral_stackup_has_no_overlap(params: SuspensionParams) -> None:
    """Y 방향 적층: 브라켓 → 로커 → 보기 → 휠 순으로 겹침 없이 바깥으로."""
    d = params.derived
    t = params.plate_t
    assert params.frame_half_w + params.pivot_block_t < d["y_rocker_in"]
    assert d["y_rocker_in"] + t < d["y_bogie_in"]
    assert d["y_bogie_in"] + t < d["y_wheel_in"]


def test_diff_bar_clears_upright_through_travel(params: SuspensionParams) -> None:
    """±15° 전 구간에서 차동 바 끝단과 로커 상방 암 사이 X 간격 > 50 mm."""
    half = params.derived["diff_bar_half_len"]
    for deg in range(-15, 16):
        th = math.radians(deg)
        phi, _ = params.differential_response(th)
        ux, _ = params.upright_top(th)
        bar_end_x = params.derived["x_diff_pivot"] - half * math.sin(phi)
        clearance = abs(ux - bar_end_x) - params.diff_bar_w / 2 - params.arm_w / 2
        assert clearance > 50.0, f"{deg}° 에서 간격 {clearance:.1f}"


def test_link_over_top_rail_clearance(params: SuspensionParams) -> None:
    """차동 링크/바 하면이 프레임 상단 레일 위에 있어야 한다."""
    rail_top = params.z_frame_top_rail + params.frame_profile / 2
    assert params.z_diff_bar - params.diff_bar_t / 2 > rail_top + 10


def test_wheels_touch_ground(params: SuspensionParams) -> None:
    """휠 접지점 Z = 0."""
    assert params.derived["z_axle"] - params.derived["wheel_r"] == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# MISUMI 호환
# ---------------------------------------------------------------------------
def test_misumi_standard_sizes(params: SuspensionParams) -> None:
    """축경/휠이 MISUMI 표준 사이즈여야 한다."""
    assert params.rocker_shaft_d in BEARINGS
    assert params.bogie_shaft_d in BEARINGS
    assert params.wheel_d in WHEELS
    assert WHEELS[params.wheel_d]["bore"] == params.axle_d


# ---------------------------------------------------------------------------
# CAD 생성 (cadquery 설치 시)
# ---------------------------------------------------------------------------
cq = pytest.importorskip("cadquery")


def test_plates_are_single_solids(params: SuspensionParams) -> None:
    """로커/보기 플레이트가 하나의 연결된 솔리드로 생성되어야 한다."""
    from orchard_rover.rocker_bogie_cad import make_bogie_plate, make_rocker_plate

    for maker in (make_rocker_plate, make_bogie_plate):
        shape = maker(params).val()
        assert shape.isValid()
        assert len(shape.Solids()) == 1


def test_assembly_builds_at_pose(params: SuspensionParams) -> None:
    """조립체가 0° 와 12° 자세 모두에서 생성되고 부품 수가 같아야 한다."""
    from orchard_rover.rocker_bogie_cad import build_assembly

    n0 = len(build_assembly(params, 0.0).toCompound().Solids())
    n1 = len(build_assembly(params, math.radians(12)).toCompound().Solids())
    assert n0 == n1 > 20
