"""J2 어깨 조인트 — NEMA23 + 1:50 사이클로이드 감속기 + 전자 브레이크 + 가스 스프링 밸런서.

좌표계: 조인트 회전축 = Y축 (팔 피치). 모터는 -Y 쪽, 출력 플랜지(상완 연결)는 +Y 쪽.
단위 mm.

출력: output/shoulder/ — parts/*.step, shoulder_assembly.step, *.png, cycloid_disc.dxf,
      design_summary.txt (감속기 파라미터, 가스 스프링 기하)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Tuple

import cadquery as cq


# ---------------------------------------------------------------------------
# 파라미터
# ---------------------------------------------------------------------------
@dataclass
class CycloidParams:
    """단단 사이클로이드 감속기 파라미터. 감속비 = 로브 수 = pins - 1."""

    pins: int = 51            # 링 핀 수 → 50:1
    r_pin_circle: float = 70  # 핀 원 반경 R  (핀 피치 2πR/N = 8.6)
    r_pin: float = 2.5        # 핀 반경 Rr → Ø5 다월핀 (핀 간격 3.6 확보)
    ecc: float = 0.6          # 편심 e  (e·N/R = 0.44)
    disc_t: float = 8.0       # 디스크 두께 (2장, 180° 위상)
    n_out_pins: int = 8       # 출력 핀 수
    r_out_pcd: float = 40.0   # 출력 핀 PCD 반경
    d_out_pin: float = 8.0    # 출력 핀 지름
    d_cam_bearing: float = 32.0   # 편심 캠 베어링 OD (6002: 15×32×9)
    housing_t: float = 10.0

    @property
    def ratio(self) -> int:
        return self.pins - 1

    @property
    def pin_pitch(self) -> float:
        return 2 * math.pi * self.r_pin_circle / self.pins

    @property
    def pin_gap(self) -> float:
        """인접 핀 사이 간격 — 양수여야 하며 ≥ Rr 권장."""
        return self.pin_pitch - 2 * self.r_pin

    @property
    def disc_od(self) -> float:
        return 2 * (self.r_pin_circle - self.r_pin + self.ecc)

    @property
    def d_out_hole(self) -> float:
        """디스크 출력 홀 지름 = 핀 지름 + 2e."""
        return self.d_out_pin + 2 * self.ecc


@dataclass
class ShoulderParams:
    """조인트 전체 치수 및 하중."""

    cyc: CycloidParams = field(default_factory=CycloidParams)
    motor_frame: float = 57.0
    motor_len: float = 112.0        # NEMA23 4.0 N·m
    motor_shaft_d: float = 8.0
    brake_len: float = 30.0         # 후면 전자 브레이크 모듈
    output_bearing: Tuple[float, float, float] = (65.0, 85.0, 10.0)  # 61813 류: ID, OD, W
    arm_plate_t: float = 12.0
    arm_plate_gap: float = 60.0     # 상완 플레이트 간 간격 (HFS5-3030 2본 사이)
    name: str = "J2_shoulder"
    has_gas_spring: bool = True
    motor_holding: float = 4.0      # N·m
    # 하중
    torque_static: float = 72.8     # N·m (수평 전개)
    # 가스 스프링 기하 (피벗 기준): 베이스 앵커 높이 h, 상완 앵커 거리 r
    gs_h: float = 180.0
    gs_r: float = 100.0
    gs_force: float = 350.0         # N (MISUMI 가스 스프링 350 N 급)
    arm_angle_min: float = -20.0    # deg, 수평 기준
    arm_angle_max: float = 80.0


# ---------------------------------------------------------------------------
# 사이클로이드 프로파일
# ---------------------------------------------------------------------------
def cycloid_profile(c: CycloidParams, n_pts: int = 1200) -> List[Tuple[float, float]]:
    """사이클로이드 디스크 외형 점열 (표준 에피트로코이드 등가 곡선).

    x(t) = R cos t − Rr cos(t+ψ) − e cos(N t)
    y(t) = −R sin t + Rr sin(t+ψ) + e sin(N t)
    ψ = −atan2( sin((1−N)t), R/(eN) − cos((1−N)t) )
    """
    R, Rr, e, N = c.r_pin_circle, c.r_pin, c.ecc, c.pins

    # 기준 에피트로코이드 (핀 중심 궤적의 등가 곡선, N-1 로브) 와 그 도함수
    def base(t: float) -> Tuple[float, float]:
        return (R * math.cos(t) - e * math.cos(N * t),
                R * math.sin(t) - e * math.sin(N * t))

    def deriv(t: float) -> Tuple[float, float]:
        return (-R * math.sin(t) + e * N * math.sin(N * t),
                R * math.cos(t) - e * N * math.cos(N * t))

    # 실제 디스크 윤곽 = 기준 곡선을 핀 반경 Rr 만큼 안쪽(중심 방향)으로 법선 오프셋
    pts = []
    for i in range(n_pts):
        t = 2 * math.pi * i / n_pts
        bx, by = base(t)
        dx, dy = deriv(t)
        norm = math.hypot(dx, dy)
        nx, ny = dy / norm, -dx / norm          # 반시계 진행 곡선의 오른쪽 법선 = 외향
        pts.append((bx - Rr * nx, by - Rr * ny))
    return pts


def make_cycloid_disc(c: CycloidParams) -> cq.Workplane:
    """사이클로이드 디스크 (XY 평면에 생성, 두께 Z). 중앙 캠 베어링 홀 + 출력 핀 홀."""
    pts = cycloid_profile(c)
    # 주기 스플라인 + close() 는 퇴화 면을 만들므로 고해상도 폴리라인 사용 (가공 DXF 와 동일)
    disc = cq.Workplane("XY").polyline(pts).close().extrude(c.disc_t)
    disc = disc.faces(">Z").workplane().circle(c.d_cam_bearing / 2 - 0.005).cutThruAll()
    out_pts = [(c.r_out_pcd * math.cos(2 * math.pi * k / c.n_out_pins),
                c.r_out_pcd * math.sin(2 * math.pi * k / c.n_out_pins))
               for k in range(c.n_out_pins)]
    disc = disc.faces(">Z").workplane().pushPoints(out_pts).circle(c.d_out_hole / 2).cutThruAll()
    return disc


def make_ring_housing(c: CycloidParams, length: float) -> cq.Workplane:
    """핀 링 하우징 — 외경 R+Rr+벽, 핀 N개 (Ø2Rr) 일체 표현."""
    r_out = c.r_pin_circle + c.r_pin + c.housing_t
    ring = cq.Workplane("XY").circle(r_out).circle(c.r_pin_circle + c.r_pin + 0.3).extrude(length)
    pins = None
    for k in range(c.pins):
        a = 2 * math.pi * k / c.pins
        p = (cq.Workplane("XY").center(c.r_pin_circle * math.cos(a), c.r_pin_circle * math.sin(a))
             .circle(c.r_pin).extrude(length))
        pins = p if pins is None else pins.union(p)
    return ring.union(pins)


def make_eccentric_cam(c: CycloidParams, shaft_d: float) -> cq.Workplane:
    """이중 편심 캠 (180° 위상), 모터축 보어 + 키/세트스크류."""
    j1 = cq.Workplane("XY").center(c.ecc, 0).circle(15.0).extrude(c.disc_t)          # 6002 ID 15
    j2 = (cq.Workplane("XY").center(-c.ecc, 0).circle(15.0).extrude(c.disc_t)
          .translate((0, 0, c.disc_t)))
    cam = j1.union(j2)
    cam = cam.faces(">Z").workplane().circle(shaft_d / 2 + 0.01).cutThruAll()
    return cam


def make_output_flange(c: CycloidParams, p: ShoulderParams) -> cq.Workplane:
    """출력 플랜지 — 출력 핀 8개 + 출력 베어링 저널 + 상완 플레이트 체결 보스."""
    ib, ob, wb = p.output_bearing
    t = 10.0
    fl = cq.Workplane("XY").circle(ib / 2).extrude(wb + t)          # 베어링 저널
    cap = cq.Workplane("XY").circle(ob / 2 + 12).extrude(t).translate((0, 0, wb))
    fl = fl.union(cap)
    pins = None
    for k in range(c.n_out_pins):
        a = 2 * math.pi * k / c.n_out_pins
        pn = (cq.Workplane("XY").center(c.r_out_pcd * math.cos(a), c.r_out_pcd * math.sin(a))
              .circle(c.d_out_pin / 2).extrude(2 * c.disc_t + 2).translate((0, 0, -2 * c.disc_t - 2)))
        pins = pn if pins is None else pins.union(pn)
    fl = fl.union(pins)
    # 상완 플레이트 체결 M6 ×6 (PCD 90)
    bolt_pts = [(45 * math.cos(2 * math.pi * k / 6), 45 * math.sin(2 * math.pi * k / 6)) for k in range(6)]
    fl = fl.faces(">Z").workplane().pushPoints(bolt_pts).circle(3.3).cutThruAll()
    return fl


def make_nema23(p: ShoulderParams) -> cq.Workplane:
    """NEMA23 57×112 더미 + 후면 브레이크 모듈."""
    f = p.motor_frame
    body = cq.Workplane("XY").rect(f, f).extrude(p.motor_len).edges("|Z").chamfer(5)
    boss = cq.Workplane("XY").circle(38.1 / 2).extrude(1.6).translate((0, 0, p.motor_len))
    shaft = cq.Workplane("XY").circle(p.motor_shaft_d / 2).extrude(21).translate((0, 0, p.motor_len))
    brake = cq.Workplane("XY").rect(f, f).extrude(p.brake_len).translate((0, 0, -p.brake_len))
    holes = [(23.57, 23.57), (-23.57, 23.57), (23.57, -23.57), (-23.57, -23.57)]
    body = body.faces(">Z").workplane().pushPoints(holes).circle(2.6).cutBlind(-6)
    return body.union(boss).union(shaft).union(brake)


def make_gas_spring(length: float) -> cq.Workplane:
    """가스 스프링 더미 (실린더 Ø22 + 로드 Ø10), X축 방향, 원점=실린더 끝 아이."""
    cyl_len = length * 0.55
    cyl = cq.Workplane("YZ").circle(11).extrude(cyl_len)
    rod = cq.Workplane("YZ").circle(5).extrude(length - cyl_len).translate((cyl_len, 0, 0))
    eyes = cq.Workplane("XZ").circle(8).extrude(6, both=True).union(
        cq.Workplane("XZ").circle(8).extrude(6, both=True).translate((length, 0, 0)))
    return cyl.union(rod).union(eyes)


# ---------------------------------------------------------------------------
# 가스 스프링 밸런서 계산
# ---------------------------------------------------------------------------
def gas_spring_balance(p: ShoulderParams) -> Dict[str, float]:
    """베이스 앵커 (0, h) 위, 상완 앵커 팔 위 r 거리. 팔 각도 φ(수평=0).

    스프링 길이 L(φ) = sqrt(r² + h² + 2 r h sinφ)
    밸런스 토크 T(φ) = F · r · h · cosφ / L(φ)   (중력 토크 ∝ cosφ 와 같은 형상)
    """
    r, h, F = p.gs_r, p.gs_h, p.gs_force

    def length(phi_deg: float) -> float:
        return math.sqrt(r * r + h * h + 2 * r * h * math.sin(math.radians(phi_deg)))

    def torque(phi_deg: float) -> float:
        return F * r * h * math.cos(math.radians(phi_deg)) / length(phi_deg) / 1000.0

    l_min, l_max = length(p.arm_angle_min), length(p.arm_angle_max)
    t0 = torque(0.0)
    return {
        "L_horizontal": length(0.0),
        "L_min": l_min, "L_max": l_max, "stroke": l_max - l_min,
        "balance_torque_horizontal": t0,
        "gravity_torque_horizontal": p.torque_static,
        "residual_horizontal": p.torque_static - t0,
        "balance_ratio": t0 / p.torque_static,
        "residual_at_45deg": p.torque_static * math.cos(math.radians(45)) - torque(45.0),
    }


# ---------------------------------------------------------------------------
# 조립 & 내보내기
# ---------------------------------------------------------------------------
def build(p: ShoulderParams) -> Tuple[cq.Assembly, Dict[str, cq.Workplane]]:
    """조인트 조립체. 축 방향 적층(Z 로 생성 후 Y 축으로 회전).

    Z 적층 (모터→출력):  브레이크 | 모터 112 | 어댑터 플레이트 10 | 디스크 2×8 | 출력 베어링 10 | 플랜지 10
    """
    c = p.cyc
    parts = {
        "cycloid_disc": make_cycloid_disc(c),
        "ring_housing": make_ring_housing(c, 2 * c.disc_t + 2),
        "eccentric_cam": make_eccentric_cam(c, p.motor_shaft_d),
        "output_flange": make_output_flange(c, p),
        "nema23_brake": make_nema23(p),
    }
    z_motor_face = 0.0                     # 모터 전면 = Z 0, 모터는 -Z 쪽
    z_disc0 = 12.0                         # 어댑터 플레이트 10 + 간극
    asm = cq.Assembly(name=p.name)
    asm.add(parts["nema23_brake"].translate((0, 0, -p.motor_len)), name="motor", color=cq.Color(0.2, 0.2, 0.25))
    adapter = (cq.Workplane("XY").circle(c.r_pin_circle + c.r_pin + c.housing_t).extrude(10)
               .faces(">Z").workplane().circle(20).cutThruAll())
    asm.add(adapter, name="adapter", color=cq.Color(0.6, 0.6, 0.62))
    asm.add(parts["ring_housing"].translate((0, 0, z_disc0 - 1)), name="ring", color=cq.Color(0.55, 0.55, 0.6))
    asm.add(parts["eccentric_cam"].translate((0, 0, z_disc0)), name="cam", color=cq.Color(0.85, 0.7, 0.1))
    d1 = parts["cycloid_disc"].translate((c.ecc, 0, z_disc0))
    d2 = parts["cycloid_disc"].rotate((0, 0, 0), (0, 0, 1), 180 / c.ratio).translate((-c.ecc, 0, z_disc0 + c.disc_t))
    asm.add(d1, name="disc1", color=cq.Color(0.85, 0.3, 0.3))
    asm.add(d2, name="disc2", color=cq.Color(0.95, 0.5, 0.2))
    z_fl = z_disc0 + 2 * c.disc_t + 2
    asm.add(parts["output_flange"].translate((0, 0, z_fl)), name="output_flange", color=cq.Color(0.2, 0.5, 0.85))
    ib, ob, wb = p.output_bearing
    brg = cq.Workplane("XY").circle(ob / 2).circle(ib / 2).extrude(wb).translate((0, 0, z_fl))
    asm.add(brg, name="output_bearing", color=cq.Color(0.8, 0.8, 0.8))
    # 상완 플레이트 (출력 플랜지에 체결, 팔 방향 +X)
    arm_plate = (cq.Workplane("XY").center(100, 0).slot2D(260, 70, 0).extrude(p.arm_plate_t)
                 .translate((0, 0, z_fl + wb + 10)))
    asm.add(arm_plate, name="upper_arm_plate", color=cq.Color(0.85, 0.3, 0.3))
    # 가스 스프링: 베이스 앵커 (0, h) → 상완 앵커 (r, 0), 수평 자세
    if p.has_gas_spring:
        gs = gas_spring_balance(p)
        L = gs["L_horizontal"]
        ang = math.degrees(math.atan2(-p.gs_h, p.gs_r))
        spring = (make_gas_spring(L).rotate((0, 0, 0), (0, 0, 1), ang)
                  .translate((0, p.gs_h, z_fl + wb + 10 + p.arm_plate_t + 15)))
        asm.add(spring, name="gas_spring", color=cq.Color(0.2, 0.7, 0.3))
    return asm, parts


def joint_stack_length(p: ShoulderParams) -> float:
    """모터 전면(Z=0)에서 출력 플랜지 상면까지의 축방향 길이."""
    return 12.0 + 2 * p.cyc.disc_t + 2 + p.output_bearing[2] + 10.0


def elbow_params() -> ShoulderParams:
    """J3 팔꿈치: 1:45 (핀 46), NEMA23 3.0 N·m (57×76), 가스 스프링 없음, 브레이크 있음."""
    cyc = CycloidParams(pins=46, r_pin_circle=63.0, r_pin=2.5, ecc=0.6, r_out_pcd=36.0)
    return ShoulderParams(cyc=cyc, motor_len=76.0, name="J3_elbow", has_gas_spring=False,
                          motor_holding=3.0, torque_static=30.1,
                          output_bearing=(55.0, 72.0, 9.0))


def export(p: ShoulderParams, out: Path) -> str:
    out.mkdir(parents=True, exist_ok=True)
    (out / "parts").mkdir(exist_ok=True)
    asm, parts = build(p)
    for n, obj in parts.items():
        cq.exporters.export(obj, str(out / "parts" / f"{n}.step"))
    asm.save(str(out / "shoulder_assembly.step"))
    cq.exporters.exportDXF(cq.Workplane("XY").add(parts["cycloid_disc"].val()).section(p.cyc.disc_t / 2),
                           str(out / "cycloid_disc.dxf"))
    comp = cq.Workplane().add(asm.toCompound())
    for name, d in {"iso": (1, -1.2, 0.8), "top": (0, 0, 1)}.items():
        cq.exporters.export(comp, str(out / f"shoulder_{name}.svg"),
                            opt={"width": 1200, "height": 900, "projectionDir": d, "showHidden": False,
                                 "strokeWidth": 0.5})
    try:
        import cairosvg
        from PIL import Image, ImageOps
        for name in ("iso", "top"):
            svg, png = out / f"shoulder_{name}.svg", out / f"shoulder_{name}.png"
            cairosvg.svg2png(url=str(svg), write_to=str(png), output_width=1200, background_color="white")
            if name != "top":
                ImageOps.flip(Image.open(png)).save(png)
    except ImportError:
        pass

    c = p.cyc
    gs = gas_spring_balance(p)
    lines = [f"=== {p.name} 조인트 설계 요약 ===", "",
             f"[사이클로이드 감속기] 비 {c.ratio}:1, 핀 {c.pins}개, R={c.r_pin_circle}, Rr={c.r_pin}, e={c.ecc}",
             f"  e·N/R = {c.ecc * c.pins / c.r_pin_circle:.2f}, 핀 피치 {c.pin_pitch:.1f}, 핀 간격 {c.pin_gap:.1f}",
             f"  디스크 OD {c.disc_od:.1f}, 두께 {c.disc_t}×2, 출력 핀 Ø{c.d_out_pin}×{c.n_out_pins} @PCD {2 * c.r_out_pcd}, 디스크 홀 Ø{c.d_out_hole:.1f}",
             f"  하우징 OD {2 * (c.r_pin_circle + c.r_pin + c.housing_t):.0f}, 출력 베어링 {p.output_bearing}",
             f"  모터 NEMA23 {p.motor_holding} N·m, 실사용 출력 {p.motor_holding * 0.5 * c.ratio * 0.8:.0f} N·m, 축방향 길이 {joint_stack_length(p) + p.motor_len + p.brake_len:.0f}"]
    if p.has_gas_spring:
        lines += ["", "[가스 스프링 밸런서]  F=%.0f N, h=%.0f, r=%.0f" % (p.gs_force, p.gs_h, p.gs_r)]
        lines += [f"  {k:28s}: {v:8.1f}" for k, v in gs.items()]
    txt = "\n".join(lines)
    (out / "design_summary.txt").write_text(txt, encoding="utf-8")
    return txt


if __name__ == "__main__":
    print(export(ShoulderParams(), Path(__file__).resolve().parents[1] / "output" / "shoulder"))
    print()
    print(export(elbow_params(), Path(__file__).resolve().parents[1] / "output" / "elbow"))
