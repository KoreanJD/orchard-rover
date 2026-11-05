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
    """팔 장착 하중 케이스 전체·좌우의 최저 안전율 ≥ 2.5 (동하중 ×3).

    휠은 정격 하중에 동하중 ×3 을 건 보수적 값이라 ≥ 2.0 (정적 ≥ 6).
    """
    for name, sf in params.safety_factors().items():
        assert sf >= (2.0 if name == "wheel" else 2.5), f"{name} 안전율 부족: {sf:.2f}"


def test_load_cases_conserve_weight(params: SuspensionParams) -> None:
    """각 케이스에서 네 바퀴(좌우 ×3) 하중 합 = 총 무게, 들림 없음."""
    for name, (m, x, y) in params.load_cases().items():
        wl = params.wheel_loads(m, x, y)
        total = sum(v[k] for v in wl.values() for k in ("front", "mid", "rear"))
        assert total == pytest.approx(m * 9.81), name
        assert min(v[k] for v in wl.values() for k in ("front", "mid", "rear")) > 0, name


def test_arm_reach_loads_front_wheel(params: SuspensionParams) -> None:
    """팔을 앞으로 뻗으면 차동 바를 통한 피치 모멘트로 전방 휠 하중이 커진다."""
    m, x, y = params.load_cases()["reach_front"]
    wl = params.wheel_loads(m, x, y)["left"]
    assert wl["front"] > wl["mid"] == pytest.approx(wl["rear"])
    assert params.wheel_loads(*params.load_cases()["vehicle_only"])["left"]["front"] == \
        pytest.approx(params.static_wheel_loads()["front"])


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
    assert d["y_wheel_in"] + WHEELS[params.wheel_d]["width"] < d["y_fork_in"]


def test_rocker_shaft_end_inside_wheel_plane(params: SuspensionParams) -> None:
    """로커 축 끝·칼라가 휠 내측면보다 안쪽 (Y 가 겹치면 보기 회전 시 중간 휠과 충돌)."""
    from orchard_rover.rocker_bogie_params import BEARING_HOLDERS

    d = params.derived
    l_brg = BEARING_HOLDERS[params.rocker_journal_d]["L"]
    shaft_end = params.frame_half_w + params.pivot_block_t + params.block_gap + l_brg + 1 + params.collar_w + 2
    assert shaft_end + 1 < d["y_wheel_in"]


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
    assert params.rocker_journal_d in BEARINGS          # 베어링 내경 = 축 저널경
    assert params.bogie_shaft_d in BEARINGS
    assert params.wheel_d in WHEELS
    assert WHEELS[params.wheel_d]["bore"] == params.axle_d


def test_idler_axle_uses_rocker_pivot_parts(params: SuspensionParams) -> None:
    """아이들러 축 = 휠 허브 베어링 내경 = SHF20·PSCS20-12 축경, 보기-휠 간격 = 홀더 T20 + 스페이서."""
    assert params.axle_d in BEARINGS
    assert params.axle_d == params.rocker_shaft_d
    assert params.wheel_gap - params.pivot_block_t >= 2.0


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


# ---------------------------------------------------------------------------
# MISUMI 표준품 체결 (HFS5-4040 홈 2줄: 면 중심 ±10, T너트 M5 까지)
# ---------------------------------------------------------------------------
def test_pivot_holder_bolts_on_rail_slot(params: SuspensionParams) -> None:
    """SHF20 볼트 줄(피벗 축 높이)이 하단 레일 측면의 홈 중심선 위."""
    assert params.z_rocker_pivot == pytest.approx(params.z_frame_low_rail - params.hfs5_slot_offset)


def test_shf20_catalog_length(params: SuspensionParams) -> None:
    """SHF20 축방향 전체 길이 T = 20 (플랜지 t8 포함)."""
    assert params.pivot_block_t == pytest.approx(20.0)
    assert params.shf_flange_t == pytest.approx(8.0)


def test_diff_post_bolts_on_crossmember_slots(params: SuspensionParams) -> None:
    """차동 포스트 볼트 홀이 크로스멤버(Y 방향) 상면 홈 X ±10 위, M5 (Ø5.5)."""
    from OCP.BRepAdaptor import BRepAdaptor_Surface

    from orchard_rover.rocker_bogie_cad import make_diff_post

    holes = set()
    for f in make_diff_post(params).val().Faces():
        if f.geomType() == "CYLINDER":
            c = BRepAdaptor_Surface(f.wrapped).Cylinder()
            if abs(c.Radius() - 2.75) < 1e-6:
                holes.add((round(c.Location().X(), 3), round(c.Location().Y(), 3)))
    assert len(holes) == 4
    assert all(abs(abs(x) - params.hfs5_slot_offset) < 1e-6 for x, _ in holes)


def test_base_plate_bolts_on_slots(params: SuspensionParams) -> None:
    """J1 베이스 플레이트 볼트가 레일(Y ±380)·크로스멤버(X ±60) 홈 중심선(±10) 위."""
    from orchard_rover.base_joint import BaseJointParams, base_plate_bolts

    b, s = BaseJointParams(), params.hfs5_slot_offset
    rail_c = params.frame_half_w - params.frame_profile / 2
    for x, y in base_plate_bolts(b):
        if abs(y) > rail_c - params.frame_profile:
            assert abs(abs(y) - rail_c) == pytest.approx(s)
        else:
            assert abs(abs(x) - b.crossmember_dx) == pytest.approx(s)


# ---------------------------------------------------------------------------
# 프레임: HFS5-4040 부재 (맞대기 + 끝단 탭)
# ---------------------------------------------------------------------------
def test_frame_members_are_profiles_without_overlap(params: SuspensionParams) -> None:
    """부재별 단일 솔리드, 카탈로그 길이 범위, 서로 겹치지 않고 맞대기 부재는 관통 부재에 닿음."""
    from itertools import combinations

    from orchard_rover.rocker_bogie_cad import frame_members, make_frame

    members = {f"frame_{m.name}": m for m in frame_members(params)}
    shapes = {n: o.val() for n, o in make_frame(params).items()}
    assert len(shapes) == 13
    for n, s in shapes.items():
        assert s.isValid() and len(s.Solids()) == 1, n
        assert 50 <= members[n].length <= 4000
    for a, b in combinations(shapes, 2):
        assert shapes[a].intersect(shapes[b]).Volume() < 0.01, (a, b)
    for n, m in members.items():
        if m.butt_ends:
            touching = [o for o in shapes if o != n and shapes[n].distance(shapes[o]) < 1e-6]
            assert len(touching) >= 2, n


def test_frame_slots_under_pivot_holder_bolts(params: SuspensionParams) -> None:
    """하단 레일 바깥 측면에서 SHF20 볼트 위치(Z 300)가 홈 개구(폭 6) 안."""
    from orchard_rover.rocker_bogie_cad import make_frame

    rail = make_frame(params)["frame_rail_low_L"].val()
    for x in (-params.shf_hole_pitch / 2, params.shf_hole_pitch / 2):
        probe = cq.Workplane("XZ").center(x, params.z_rocker_pivot).circle(2.5).extrude(-3).translate(
            (0, params.frame_half_w - 1.5, 0)).val()                # 바깥 면 안쪽 1.5 (홈 립 깊이 이내)
        assert rail.intersect(probe).Volume() < 1e-6


# ---------------------------------------------------------------------------
# 아이들러 휠 축 고정 (SFJ20 + SHF20 홀더 + 스페이서 + 베어링 ×2 + 와셔 + PSCS20-12)
# ---------------------------------------------------------------------------
def test_idler_axle_stack_is_retained(params: SuspensionParams) -> None:
    """보기 판 → 홀더 → 스페이서 → 휠 베어링 → 와셔 → 칼라가 빈틈없이 닿고 겹치지 않으며, 축이 칼라를 관통."""
    from orchard_rover.rocker_bogie_cad import build_assembly

    shapes = {}

    def walk(a, loc):
        for ch in a.children:
            l = loc * ch.loc if ch.loc is not None else loc
            if ch.obj is not None:
                shapes[ch.name] = ch.obj.val().moved(l)
            walk(ch, l)

    walk(build_assembly(params, 0.0), cq.Location())
    for k in ("M", "R"):
        chain = ["bogie_plate_L", f"axle_holder_{k}_L", f"idler_spacer_{k}_L", f"wheel_bearing_{k}_L",
                 f"idler_washer_{k}_L", f"idler_collar_{k}_L"]
        for a, b in zip(chain, chain[1:]):
            assert shapes[a].distance(shapes[b]) < 1e-6, (a, b)
            assert shapes[a].intersect(shapes[b]).Volume() < 0.01, (a, b)
        axle = shapes[f"axle_{k}_L"].BoundingBox()
        collar = shapes[f"idler_collar_{k}_L"].BoundingBox()
        assert collar.ymax < axle.ymax <= collar.ymax + 2 + 1e-6
        assert shapes[f"wheel_{k}_L"].intersect(shapes[f"wheel_bearing_{k}_L"]).Volume() < 0.01


# ---------------------------------------------------------------------------
# 베어링 보스 → MISUMI BGRBB 베어링 홀더 볼트 체결 (판은 단일 두께 워터젯)
# ---------------------------------------------------------------------------
def _placed_shapes(params: SuspensionParams, deg: float = 0.0) -> dict:
    from orchard_rover.rocker_bogie_cad import build_assembly

    shapes = {}

    def walk(a, loc):
        for ch in a.children:
            l = loc * ch.loc if ch.loc is not None else loc
            if ch.obj is not None:
                shapes[ch.name] = ch.obj.val().moved(l)
            walk(ch, l)

    walk(build_assembly(params, math.radians(deg)), cq.Location())
    return shapes


def test_plates_have_no_bosses(params: SuspensionParams) -> None:
    """로커·보기 판과 차동 바가 단일 두께 (2D 가공만으로 제작)."""
    from orchard_rover.rocker_bogie_cad import make_bogie_plate, make_diff_bar, make_rocker_plate

    for maker in (make_rocker_plate, make_bogie_plate):
        bb = maker(params).val().BoundingBox()
        assert bb.ylen == pytest.approx(params.plate_t)
    assert make_diff_bar(params).val().BoundingBox().zlen == pytest.approx(params.diff_bar_t)


def test_bearing_holders_seat_and_stack(params: SuspensionParams) -> None:
    """홀더 플랜지가 판에 앉고, 축 위 스페이서·베어링·칼라가 빈틈없이 닿으며 겹치지 않는다."""
    s = _placed_shapes(params)

    def touch(a, b):
        assert s[a].distance(s[b]) < 1e-6, (a, b)
        assert s[a].intersect(s[b]).Volume() < 0.01, (a, b)

    for a, b in (("rocker_holder_L", "rocker_plate_L"), ("bogie_holder_L", "bogie_plate_L"),
                 ("diff_holder", "diff_bar"), ("rocker_holder_L", "rocker_bearing_L"),
                 ("bogie_holder_L", "bogie_bearing_L"), ("diff_holder", "diff_bearing")):
        touch(a, b)
    # 축 방향 적층 (정지측): SHF20 | 스페이서 | 6004 ×2 | 스페이서 | 칼라, 로커 판 | 스페이서 | 6001 ×2 | 스페이서 | 칼라
    for chain in (["pivot_block_L", "rocker_spacer_in_L", "rocker_bearing_L", "rocker_spacer_out_L", "shaft_collar_L"],
                  ["rocker_plate_L", "bogie_spacer_L", "bogie_bearing_L", "bogie_spacer_out_L", "bogie_collar_L"],
                  ["diff_post", "diff_spacer", "diff_bearing", "diff_spacer_top", "diff_ring", "diff_post"]):
        for a, b in zip(chain, chain[1:]):
            touch(a, b)
    # 회전측 홀더는 정지측 부품과 간극
    assert s["rocker_holder_L"].distance(s["pivot_block_L"]) >= params.block_gap - 1e-6
    assert s["rocker_holder_L"].distance(s["shaft_collar_L"]) >= 1.0 - 1e-6
    assert s["bogie_holder_L"].distance(s["rocker_plate_L"]) >= 2.0
    assert s["bogie_holder_L"].distance(s["bogie_collar_L"]) >= 1.0 - 1e-6
    assert s["diff_holder"].distance(s["diff_post"]) >= 2.0 - 1e-6
    # 차동 핀: 스냅링이 홀더와 간극, 핀 끝은 홀더 윗면 위 4 이내 (팔 접힘 자세 간섭 방지)
    assert s["diff_holder"].distance(s["diff_ring"]) >= 1.0 - 1e-6
    assert s["diff_post"].BoundingBox().zmax <= s["diff_holder"].BoundingBox().zmax + 4 + 1e-6
    # 보기 축: 칼라가 축 끝 안쪽, 칼라·축 끝이 중간 휠 내측면보다 안쪽
    collar, shaft = s["bogie_collar_L"].BoundingBox(), s["bogie_shaft_L"].BoundingBox()
    assert collar.ymax < shaft.ymax <= collar.ymax + 2 + 1e-6
    assert shaft.ymax < s["wheel_M_L"].BoundingBox().ymin
