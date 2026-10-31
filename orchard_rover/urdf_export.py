"""MoveIt2 용 로봇 기술 패키지 — URDF + SRDF + 링크별 STL (시각 / 충돌).

CAD 부품을 체인 단계(arm_pose_render.stage_of, motion_check 와 동일)로 묶어 링크를 만들고,
관절 축·부호는 arm_pose_render.place() 와 같게 둔다 (같은 관절각이면 같은 자세).

링크 (구동체 좌표, X 전방 / Y 좌측 / Z 상방):
  base_link     구동체 + J1 고정부 (베이스 플레이트·크로스멤버·구동부·크로스롤러), 지면 원점
  j1_link       터릿·J1 브라켓·가스 스프링 클레비스 + J2 고정부 (모터·링 하우징 등)   ← J1 (Z)
  upper_arm     J2 출력 플랜지·상완 레일·측판·J3 출력 플랜지                            ← J2 (−Y)
  forearm_root  J3 모터·링·전완판·연결 브라켓·J4 하우징                                ← J3 (−Y)
  forearm_tube  전완 튜브·J5 요크·J5 모터                                               ← J4 (X)
  wrist         손목 블록·J6 모터·감속기·카메라                                         ← J5 (−Y)
  flange        ISO 9409 플랜지                                                         ← J6 (X)
  tool0         플랜지 면 (고정)

관절 원점은 팔 좌표계(J2 축 = 구동체 (300, 0, 958))의 축 위 점. 메시는 mm STL, URDF 에서 0.001 배.
관성은 CAD 체적 × Al 6061 밀도 (모터 등 실제 밀도와 다름 — 계획용 근사).
가스 스프링(양단이 j1_link·upper_arm 에 걸린 2체 부품)은 충돌 모델에 포함하지 않는다.

출력: output/urdf/orchard_rover_description/ (package.xml, CMakeLists.txt, urdf/, srdf/, meshes/)
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Tuple
from xml.sax.saxutils import escape

import cadquery as cq
from OCP.BRepGProp import BRepGProp
from OCP.BRepTools import BRepTools
from OCP.GProp import GProp_GProps

from orchard_rover.arm_assembly import UPPER_LEN
from orchard_rover.arm_pose_render import stage_of
from orchard_rover.motion_check import LIMITS, load_parts
from orchard_rover.transport_check import CANDIDATES, J2_ORIGIN
from orchard_rover.wrist_joint import WristParams, wrist_axes

PKG = "orchard_rover_description"
DENSITY = 2700.0                         # kg/m³ (Al 6061)
LINKS = ["base_link", "j1_link", "upper_arm", "forearm_root", "forearm_tube", "wrist", "flange"]
STAGE_LINK = {0: "j1_link", 1: "upper_arm", 2: "forearm_root", 3: "forearm_tube", 4: "wrist", 5: "flange"}
# 관절 출력 토크(N·m)·속도(°/s) — docs/robot-arm-spec.md 액추에이터 표
EFFORT = {"J1": 24.0, "J2": 80.0, "J3": 54.0, "J4": 11.5, "J5": 11.5, "J6": 3.6}
VELOCITY = {"J1": 45.0, "J2": 30.0, "J3": 30.0, "J4": 45.0, "J5": 45.0, "J6": 60.0}


def joint_frames() -> List[Tuple[str, str, str, Tuple[float, float, float], Tuple[float, float, float]]]:
    """(관절, 부모, 자식, 자식 원점 (팔 좌표계 mm, 홈), 축) — place() 와 같은 축·부호."""
    w = WristParams(x_j3=UPPER_LEN, x_flange=UPPER_LEN + 450 + 100)
    ax = wrist_axes(w)
    x_j4 = UPPER_LEN + w.j4_offset                  # J4 롤 축 위 점 (하우징 후단)
    return [
        ("J1", "base_link", "j1_link", (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
        ("J2", "j1_link", "upper_arm", (0.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
        ("J3", "upper_arm", "forearm_root", (UPPER_LEN, 0.0, 0.0), (0.0, -1.0, 0.0)),
        ("J4", "forearm_root", "forearm_tube", (x_j4, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ("J5", "forearm_tube", "wrist", (ax["J5_pitch_x"], 0.0, 0.0), (0.0, -1.0, 0.0)),
        ("J6", "wrist", "flange", (ax["flange_x"], 0.0, 0.0), (1.0, 0.0, 0.0)),
    ]


def link_origins() -> Dict[str, Tuple[float, float, float]]:
    """각 링크 원점의 홈 자세 위치 (구동체 좌표 mm)."""
    ox, oy, oz = J2_ORIGIN
    out = {"base_link": (0.0, 0.0, 0.0)}
    for _, _, child, (x, y, z), _ in joint_frames():
        out[child] = (ox + x, oy + y, oz + z)
    return out


def link_shapes() -> Dict[str, List[Tuple[str, cq.Shape]]]:
    """링크별 (부품명, 링크 좌표계 형상 mm)."""
    org = link_origins()
    out: Dict[str, List[Tuple[str, cq.Shape]]] = {n: [] for n in LINKS}
    ox, oy, oz = J2_ORIGIN
    for name, grp, shp in load_parts():
        if grp == "fixed":
            link = "base_link"
            world = shp
        elif name.startswith("J1:"):
            link = "j1_link"
            world = shp
        else:
            link = STAGE_LINK[stage_of(name)]
            world = shp.translate(cq.Vector(ox, oy, oz))         # 팔 좌표 → 구동체 좌표
        lx, ly, lz = org[link]
        out[link].append((name, world.translate(cq.Vector(-lx, -ly, -lz))))
    return out


def inertial(shapes: List[cq.Shape]) -> Tuple[float, Tuple[float, float, float], Tuple[float, ...]]:
    """(질량 kg, 질량중심 m, 관성 ixx ixy ixz iyy iyz izz kg·m², 질량중심 기준) — 균일 밀도."""
    props = GProp_GProps()
    for s in shapes:
        p = GProp_GProps()
        BRepGProp.VolumeProperties_s(s.wrapped, p)
        props.Add(p)
    vol = props.Mass()                                            # mm³
    c = props.CentreOfMass()
    m = props.MatrixOfInertia()                                   # mm^5 (체적 기준, 질량중심)
    k = DENSITY * 1e-9 * 1e-6                                     # → kg·m²
    mass = vol * 1e-9 * DENSITY
    ixx, iyy, izz = m.Value(1, 1) * k, m.Value(2, 2) * k, m.Value(3, 3) * k
    ixy, ixz, iyz = m.Value(1, 2) * k, m.Value(1, 3) * k, m.Value(2, 3) * k
    return mass, (c.X() / 1000, c.Y() / 1000, c.Z() / 1000), (ixx, ixy, ixz, iyy, iyz, izz)


def _f(v: float) -> str:
    return f"{0.0 if abs(v) < 1e-12 else v:.9g}"


def _xyz(v) -> str:
    return " ".join(_f(x) for x in v)


def build_urdf(inertials: Dict[str, Tuple]) -> str:
    org = link_origins()
    lines = ['<?xml version="1.0"?>', '<robot name="orchard_rover">',
             '  <!-- orchard_rover.urdf_export 생성. 단위 m, 메시는 mm STL × 0.001 -->']
    for link in LINKS:
        mass, com, (ixx, ixy, ixz, iyy, iyz, izz) = inertials[link]
        mesh = f"package://{PKG}/meshes/{{kind}}/{link}.stl"
        lines += [f'  <link name="{link}">',
                  "    <inertial>",
                  f'      <origin xyz="{_xyz(com)}" rpy="0 0 0"/>',
                  f'      <mass value="{_f(mass)}"/>',
                  f'      <inertia ixx="{_f(ixx)}" ixy="{_f(ixy)}" ixz="{_f(ixz)}" '
                  f'iyy="{_f(iyy)}" iyz="{_f(iyz)}" izz="{_f(izz)}"/>',
                  "    </inertial>"]
        for kind, tag in (("visual", "visual"), ("collision", "collision")):
            lines += [f"    <{tag}>",
                      '      <origin xyz="0 0 0" rpy="0 0 0"/>',
                      f'      <geometry><mesh filename="{escape(mesh.format(kind=kind))}" scale="0.001 0.001 0.001"/></geometry>',
                      f"    </{tag}>"]
        lines.append("  </link>")
    lines += ['  <link name="tool0"/>']
    for joint, parent, child, _, axis in joint_frames():
        po, co = org[parent], org[child]
        xyz = tuple((c - p) / 1000 for c, p in zip(co, po))
        lo, hi = LIMITS[joint]
        lines += [f'  <joint name="{joint}" type="revolute">',
                  f'    <parent link="{parent}"/>', f'    <child link="{child}"/>',
                  f'    <origin xyz="{_xyz(xyz)}" rpy="0 0 0"/>',
                  f'    <axis xyz="{_xyz(axis)}"/>',
                  f'    <limit lower="{_f(math.radians(lo))}" upper="{_f(math.radians(hi))}" '
                  f'effort="{_f(EFFORT[joint])}" velocity="{_f(math.radians(VELOCITY[joint]))}"/>',
                  "  </joint>"]
    lines += ['  <joint name="tool0_fixed" type="fixed">', '    <parent link="flange"/>',
              '    <child link="tool0"/>', '    <origin xyz="0 0 0" rpy="0 0 0"/>', "  </joint>",
              "</robot>", ""]
    return "\n".join(lines)


def build_srdf() -> str:
    adjacent = [(p, c) for _, p, c, _, _ in joint_frames()] + [("flange", "tool0")]
    lines = ['<?xml version="1.0"?>', '<robot name="orchard_rover">',
             '  <group name="arm">', '    <chain base_link="base_link" tip_link="tool0"/>', "  </group>",
             '  <end_effector name="tool" parent_link="tool0" group="arm"/>']
    poses = {"home": {j: 0.0 for j in EFFORT}}
    poses["transport"] = {j: CANDIDATES["T3_fold_back"].get(j, 0.0) for j in EFFORT}   # 운반 기본 T3
    for pname, q in poses.items():
        lines.append(f'  <group_state name="{pname}" group="arm">')
        lines += [f'    <joint name="{j}" value="{_f(math.radians(v))}"/>' for j, v in q.items()]
        lines.append("  </group_state>")
    for a, b in adjacent:
        lines.append(f'  <disable_collisions link1="{a}" link2="{b}" reason="Adjacent"/>')
    lines += ["</robot>", ""]
    return "\n".join(lines)


PACKAGE_XML = f"""<?xml version="1.0"?>
<package format="3">
  <name>{PKG}</name>
  <version>0.1.0</version>
  <description>Orchard rover (rocker-bogie + 6-axis arm) URDF/SRDF and meshes, generated from CadQuery CAD.</description>
  <maintainer email="noreply@example.com">orchard-rover</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
"""

CMAKE = f"""cmake_minimum_required(VERSION 3.8)
project({PKG})
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf srdf meshes DESTINATION share/${{PROJECT_NAME}})
ament_package()
"""


def export(out: Path, visual_tol: float = 0.5, collision_tol: float = 3.0) -> Dict[str, Tuple]:
    """패키지를 out/orchard_rover_description 에 쓴다. 반환: 링크별 inertial."""
    root = out / PKG
    for sub in ("urdf", "srdf", "meshes/visual", "meshes/collision"):
        (root / sub).mkdir(parents=True, exist_ok=True)
    inertials = {}
    for link, parts in link_shapes().items():
        shapes = [s for _, s in parts]
        comp = cq.Compound.makeCompound(shapes)
        inertials[link] = inertial(shapes)
        # OCC 는 형상에 남은 삼각망을 재사용하므로 내보내기마다 지운다 (그래야 공차가 적용됨)
        for kind, tol, ang in (("collision", collision_tol, 0.6), ("visual", visual_tol, 0.3)):
            BRepTools.Clean_s(comp.wrapped)
            cq.exporters.export(cq.Workplane().add(comp), str(root / "meshes" / kind / f"{link}.stl"),
                                tolerance=tol, angularTolerance=ang)
    (root / "urdf" / "orchard_rover.urdf").write_text(build_urdf(inertials), encoding="utf-8")
    (root / "srdf" / "orchard_rover.srdf").write_text(build_srdf(), encoding="utf-8")
    (root / "package.xml").write_text(PACKAGE_XML, encoding="utf-8")
    (root / "CMakeLists.txt").write_text(CMAKE, encoding="utf-8")
    return inertials


if __name__ == "__main__":
    res = export(Path(__file__).resolve().parents[1] / "output" / "urdf")
    for link, (m, com, _) in res.items():
        print(f"{link:14s} mass {m:7.2f} kg  com {tuple(round(c, 3) for c in com)}")
