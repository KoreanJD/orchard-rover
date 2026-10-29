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


def test_disc2_phase_aligns_lobes_and_output_holes() -> None:
    """2번 디스크 회전각: 로브 반 피치 위상 + 출력 홀 피치 배수 (J2 90°, J3 180°)."""
    from orchard_rover.shoulder_joint import elbow_params

    for c, expected in ((ShoulderParams().cyc, 90.0), (elbow_params().cyc, 180.0)):
        a, lobe = c.disc2_phase, 360.0 / c.ratio
        assert a == pytest.approx(expected)
        assert math.remainder(a, 360.0 / c.n_out_pins) == pytest.approx(0.0, abs=1e-9)
        assert abs(math.remainder(a, lobe)) == pytest.approx(lobe / 2)


def test_cam_bore_on_motor_axis(p: ShoulderParams) -> None:
    """캠 보어 중심 = 모터 축(원점), 두 로브는 ±e."""
    from OCP.BRepAdaptor import BRepAdaptor_Surface

    from orchard_rover.shoulder_joint import make_eccentric_cam

    cam = make_eccentric_cam(p.cyc, p.motor_shaft_d).val()
    cyls = []
    for f in cam.Faces():
        if f.geomType() == "CYLINDER":
            c = BRepAdaptor_Surface(f.wrapped).Cylinder()
            cyls.append((c.Radius(), c.Location().X(), c.Location().Y()))
    bore = [(x, y) for r, x, y in cyls if r < 5]
    lobes = sorted(x for r, x, y in cyls if r == pytest.approx(15.0))
    assert bore and all(math.hypot(x, y) < 1e-6 for x, y in bore)
    assert lobes == pytest.approx([-p.cyc.ecc, p.cyc.ecc])


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


def test_arm_mating_faces_flush() -> None:
    """체결면 밀착: J2·J3 출력 플랜지↔상완 측판 +Y 면, 전완판 = J3 어댑터 자리 (Y 간격 0)."""
    from orchard_rover.arm_assembly import build_arm
    from orchard_rover.shoulder_joint import ADAPTER_T, elbow_params, joint_stack_length

    asm = build_arm()
    bb = {ch.name: ch.obj.val().BoundingBox() for ch in asm.children}
    assert bb["upper_plate_J2"].ymax == pytest.approx(bb["J2_output_flange"].ymin, abs=1e-6)
    assert bb["upper_plate_J3"].ymax == pytest.approx(bb["J3_output_flange"].ymin, abs=1e-6)
    y_j3_face = bb["upper_plate_J3"].ymax + joint_stack_length(elbow_params())
    assert bb["fore_plate_J3"].ymax == pytest.approx(y_j3_face, abs=1e-6)
    assert bb["fore_plate_J3"].ymin == pytest.approx(y_j3_face - ADAPTER_T, abs=1e-6)
    assert "J3_adapter" not in bb and "J2_adapter" in bb
    # 레일은 측판 +Y 면에 밀착
    assert bb["upper_rail_0"].ymin == pytest.approx(bb["upper_plate_J2"].ymax, abs=1e-6)


def test_rails_clear_ring_housings() -> None:
    """레일 안쪽 모서리가 J2·J3 링 하우징 외경 밖 (≥ 2 mm)."""
    from orchard_rover.arm_assembly import RAIL_GAP, UPPER_LEN, build_arm
    from orchard_rover.shoulder_joint import adapter_radius, elbow_params

    bb = {ch.name: ch.obj.val().BoundingBox() for ch in build_arm().children}
    rail = bb["upper_rail_0"]
    z_in = RAIL_GAP / 2
    assert math.hypot(rail.xmin, z_in) >= adapter_radius(ShoulderParams()) + 2
    assert math.hypot(UPPER_LEN - rail.xmax, z_in) >= adapter_radius(elbow_params()) + 2


def test_bracket_replaces_j2_adapter_and_spring_anchors() -> None:
    """전체 로봇: J1 브라켓이 J2 어댑터 자리 (팔 Y −10..0), 가스 스프링 아이가 두 앵커 핀에 걸림."""
    from orchard_rover.base_joint import build_full_robot
    from orchard_rover.shoulder_joint import ADAPTER_T

    full, info = build_full_robot(0.0)
    shapes = {}

    def walk(a, loc):
        for ch in a.children:
            l = loc * ch.loc if ch.loc is not None else loc
            if ch.obj is not None:
                shp = ch.obj.val() if isinstance(ch.obj, cq.Workplane) else ch.obj
                shapes[ch.name] = shp.moved(l)
            walk(ch, l)

    walk(full, cq.Location())
    assert "J2_adapter" not in shapes
    br = shapes["shoulder_bracket"].BoundingBox()
    assert br.ymin == pytest.approx(-ADAPTER_T, abs=1e-6)
    assert shapes["shoulder_bracket"].intersect(shapes["J2_motor"]).Volume() < 1.0
    spring = shapes["gas_spring"]
    for anchor in ("gs_clevis", "gs_anchor_arm"):
        assert spring.intersect(shapes[anchor]).Volume() < 1.0
        assert spring.distance(shapes[anchor]) < 0.2            # 핀 Ø8 ↔ 아이 구멍 Ø8.2
    # 베이스 앵커는 J2 축 아래 (압축 스프링이 팔을 들어 올림)
    assert shapes["gs_clevis"].BoundingBox().zmax < info["z_j2_axis"] - 100


def test_arm_excludes_joint_placeholders() -> None:
    """조인트 단독 시각화용 임시 상완판·가스 스프링은 팔 조립에 들어가지 않는다."""
    from orchard_rover.arm_assembly import JOINT_PLACEHOLDERS, build_arm

    names = {ch.name for ch in build_arm().children}
    assert not {f"{j}_{n}" for j in ("J2", "J3") for n in JOINT_PLACEHOLDERS} & names


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
