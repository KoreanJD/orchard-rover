"""판재 DXF: 두께·구멍 수·구멍 위치가 조립 형상과 일치."""

from __future__ import annotations

import pytest

ezdxf = pytest.importorskip("ezdxf")
pytest.importorskip("cadquery")

from orchard_rover.plate_dxf import export  # noqa: E402


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    d = tmp_path_factory.mktemp("dxf")
    export(d)
    return d


def _circles(path):
    msp = ezdxf.readfile(str(path)).modelspace()
    return {(round(e.dxf.center.x, 2), round(e.dxf.center.y, 2), round(2 * e.dxf.radius, 2))
            for e in msp.query("CIRCLE")}


def test_all_plates_exported(out) -> None:
    names = {p.stem for p in out.glob("*.dxf")}
    assert {"upper_plate_J2", "upper_plate_J3", "fore_plate_J3", "fore_offset_bracket", "shoulder_bracket",
            "shoulder_bracket_gusset", "j1_base_plate", "j1_turret"} <= names


def test_flange_bolt_circles_on_joint_axes(out) -> None:
    """측판 플랜지 볼트 6개가 관절 축 중심 PCD 위 (J2 Ø90 @0, J3 Ø76 @500)."""
    import math

    for name, cx, pcd in (("upper_plate_J2", 0.0, 90.0), ("upper_plate_J3", 500.0, 76.0)):
        bolts = [c for c in _circles(out / f"{name}.dxf") if c[2] == 6.6 and abs(c[1]) < 45]
        assert len(bolts) == 6
        assert all(math.hypot(x - cx, y) == pytest.approx(pcd / 2, abs=0.05) for x, y, _ in bolts)


def test_bracket_motor_pattern_at_j2_axis(out) -> None:
    """J1 브라켓: 모터 보스 Ø40 + NEMA23 M5 ×4 가 J2 축 높이 200 중심."""
    cs = _circles(out / "shoulder_bracket.dxf")
    assert (0.0, 200.0, 40.0) in cs
    assert {(sx * 23.57, 200 + sz * 23.57, 5.2) for sx in (1, -1) for sz in (1, -1)} <= cs
