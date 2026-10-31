"""URDF/SRDF: 트리·관절 한계, 메시, 그리고 URDF 순기구학이 CAD 자세(place)와 일치."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

cq = pytest.importorskip("cadquery")

from orchard_rover.arm_pose_render import place, stage_of  # noqa: E402
from orchard_rover.motion_check import LIMITS, load_parts  # noqa: E402
from orchard_rover.transport_check import J2_ORIGIN  # noqa: E402
from orchard_rover.urdf_export import LINKS, PKG, export, link_shapes  # noqa: E402


@pytest.fixture(scope="module")
def pkg(tmp_path_factory):
    out = tmp_path_factory.mktemp("urdf")
    export(out, visual_tol=2.0, collision_tol=4.0)
    return out / PKG


@pytest.fixture(scope="module")
def urdf(pkg):
    return ET.parse(pkg / "urdf" / "orchard_rover.urdf").getroot()


def _rot(axis, deg):
    a = np.asarray(axis, float)
    a /= np.linalg.norm(a)
    t = math.radians(deg)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * k + (1 - math.cos(t)) * k @ k


def fk(urdf, q):
    """링크별 (R, p[mm]) — base_link 기준, 관절각 q (deg)."""
    joints = {j.find("child").get("link"): j for j in urdf.findall("joint")}
    out = {"base_link": (np.eye(3), np.zeros(3))}

    def get(link):
        if link in out:
            return out[link]
        j = joints[link]
        R0, p0 = get(j.find("parent").get("link"))
        xyz = np.array([float(v) for v in j.find("origin").get("xyz").split()]) * 1000
        R = R0
        if j.get("type") == "revolute":
            axis = [float(v) for v in j.find("axis").get("xyz").split()]
            R = R0 @ _rot(axis, q.get(j.get("name"), 0.0))
        out[link] = (R, p0 + R0 @ xyz)
        return out[link]

    for link in joints:
        get(link)
    return out


def test_tree_and_limits(urdf) -> None:
    links = {e.get("name") for e in urdf.findall("link")}
    assert set(LINKS) | {"tool0"} == links
    rev = {j.get("name"): j for j in urdf.findall("joint") if j.get("type") == "revolute"}
    assert set(rev) == set(LIMITS)
    for name, (lo, hi) in LIMITS.items():
        lim = rev[name].find("limit")
        assert float(lim.get("lower")) == pytest.approx(math.radians(lo), abs=1e-6)
        assert float(lim.get("upper")) == pytest.approx(math.radians(hi), abs=1e-6)


def test_meshes_and_masses(pkg, urdf) -> None:
    for link in LINKS:
        for kind in ("visual", "collision"):
            f = pkg / "meshes" / kind / f"{link}.stl"
            assert f.stat().st_size > 1000, f
        mass = float(urdf.find(f"link[@name='{link}']/inertial/mass").get("value"))
        assert mass > 0.01
    assert (pkg / "meshes/collision/base_link.stl").stat().st_size < (pkg / "meshes/visual/base_link.stl").stat().st_size


@pytest.mark.parametrize("q", [
    {},
    {"J1": 35.0, "J2": 50.0, "J3": -90.0, "J4": 60.0, "J5": -70.0, "J6": 120.0},
    {"J1": -150.0, "J2": -20.0, "J3": 110.0, "J4": -170.0, "J5": 100.0, "J6": -45.0},
])
def test_fk_matches_cad_pose(urdf, q) -> None:
    """URDF 순기구학으로 옮긴 부품 질량중심 = CAD place() 자세의 질량중심 (0.01 mm)."""
    T = fk(urdf, q)
    org = np.array(J2_ORIGIN)
    by_link = {n: s for parts in link_shapes().values() for n, s in parts}
    link_of = {n: link for link, parts in link_shapes().items() for n, _ in parts}
    checked = 0
    for name, grp, shp in load_parts():
        if grp == "fixed" or name.startswith("J1:"):
            continue
        if name not in ("J2_output_flange", "J3_ring", "fore_plate_J3", "j4_housing", "forearm_tube",
                        "j5_motor", "wrist_block", "wrist_camera", "iso_flange"):
            continue
        expect = np.array(place(shp, stage_of(name), q).Center().toTuple()) + org
        R, p = T[link_of[name]]
        got = p + R @ np.array(by_link[name].Center().toTuple())
        assert np.allclose(got, expect, atol=0.01), (name, got, expect)
        checked += 1
    assert checked == 9


def test_srdf_group_and_poses(pkg) -> None:
    s = ET.parse(pkg / "srdf" / "orchard_rover.srdf").getroot()
    chain = s.find("group[@name='arm']/chain")
    assert (chain.get("base_link"), chain.get("tip_link")) == ("base_link", "tool0")
    poses = {g.get("name"): {j.get("name"): float(j.get("value")) for j in g} for g in s.findall("group_state")}
    assert set(poses) == {"home", "transport"}
    assert poses["transport"]["J3"] == pytest.approx(math.radians(-120))
