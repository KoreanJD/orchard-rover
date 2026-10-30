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

# 어댑터 플레이트 두께 — 모터 전면과 링 하우징 사이. 조립에 따라 J1 브라켓(J2)이나
# 전완판(J3)이 이 자리를 대신한다.
ADAPTER_T = 10.0


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

    @property
    def disc2_phase(self) -> float:
        """2번 디스크 회전각(deg) — 로브 반 피치 위상과 출력 홀 정렬을 동시에 만족.

        180° 편심 위치의 디스크는 로브가 반 피치(180/비) 어긋나야 링 핀과 맞물린다.
        출력 홀 피치의 배수 중 로브 피치로 나눈 나머지가 반 피치인 각을 쓰면
        같은 부품을 돌려 끼우는 것만으로 홀이 출력 핀에 정렬된다.
        """
        lobe = 360.0 / self.ratio
        for k in range(self.n_out_pins):
            a = k * 360.0 / self.n_out_pins
            if abs(math.remainder(a - lobe / 2, lobe)) < 1e-6:
                return a
        raise ValueError(f"출력 핀 {self.n_out_pins}개·로브 {self.ratio}개 조합은 동일 디스크 2장으로 "
                         "위상과 홀 정렬을 동시에 만족할 수 없음")


@dataclass(frozen=True)
class GasSpringSpec:
    """가스 스프링 카탈로그 사양 (아이 중심 간 길이)."""

    name: str
    stroke: float
    force: float            # 반력 N (20 ℃, 최대 길이 − 10 mm 일 때)
    l_min: float
    l_max: float
    d_cyl: float
    d_rod: float
    progression: float = 0.3    # 최소 길이에서 반력 증가율 (카탈로그 미기재 → 일반값 가정)

    def force_at(self, length: float) -> float:
        return self.force * (1 + self.progression * (self.l_max - 10 - length) / self.stroke)


# MISUMI 가스스프링 장착방향자유형 (kr.misumi-ec.com, 2026-10-30 확인)
FGSS22120C = GasSpringSpec("FGSS22120C", stroke=120, force=392, l_min=320, l_max=440, d_cyl=22, d_rod=10)


@dataclass
class ShoulderParams:
    """조인트 전체 치수 및 하중."""

    cyc: CycloidParams = field(default_factory=CycloidParams)
    motor_frame: float = 57.0
    motor_len: float = 112.0        # NEMA23 4.0 N·m
    motor_shaft_d: float = 8.0
    brake_len: float = 30.0         # 후면 전자 브레이크 모듈
    output_bearing: Tuple[float, float, float] = (65.0, 85.0, 10.0)  # 61813 류: ID, OD, W
    flange_bolt_pcd: float = 90.0   # 출력 플랜지 → 상완 측판 M6 ×6
    arm_plate_t: float = 12.0
    arm_plate_gap: float = 60.0     # 상완 플레이트 간 간격 (HFS6-3030 2본 사이)
    name: str = "J2_shoulder"
    has_gas_spring: bool = True
    motor_holding: float = 4.0      # N·m
    # 하중 (arm_check.joint_static_torques: 수평 완전 전개 + 툴 3 kg / 전완 접힘·툴 없음)
    torque_static: float = 79.7     # N·m (수평 전개)
    torque_static_min: float = 23.2
    # 가스 스프링 (팔 좌표계 XZ, J2 축 원점, 팔 수평): 베이스 앵커 (x, z) 는 J1 터릿 클레비스,
    # 상완 앵커는 측판 바깥면 스터드 — 팔 방향에서 δ 만큼 위, 반경 r
    gs_spec: GasSpringSpec = FGSS22120C
    gs_base: Tuple[float, float] = (-35.0, -150.0)
    gs_arm_r: float = 280.0
    gs_arm_deg: float = 10.0
    arm_angle_min: float = -20.0    # deg, 수평 기준 (J2 가동 범위)
    arm_angle_max: float = 90.0


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


def make_ring_housing(c: CycloidParams, p: ShoulderParams) -> cq.Workplane:
    """핀 링 하우징 — 외경 R+Rr+벽, 핀 N개 (Ø2Rr) 일체 + 출력 베어링 외륜 시트.

    Z=0 은 어댑터 상면(밀착). 핀 구간은 디스크 2장 + 양측 간극 1, 그 위로 출력 베어링
    외륜을 받는 시트(보어 = 베어링 OD)가 베어링 폭 −0.5 까지 — 회전하는 플랜지 캡과 0.5 간극.
    """
    r_out = c.r_pin_circle + c.r_pin + c.housing_t
    l_pin = 2 * c.disc_t + 4
    ib, ob, wb = p.output_bearing
    ring = cq.Workplane("XY").circle(r_out).circle(c.r_pin_circle + c.r_pin + 0.3).extrude(l_pin)
    seat = cq.Workplane("XY").circle(r_out).circle(ob / 2).extrude(wb - 0.5).translate((0, 0, l_pin))
    pins = None
    for k in range(c.pins):
        a = 2 * math.pi * k / c.pins
        pn = (cq.Workplane("XY").center(c.r_pin_circle * math.cos(a), c.r_pin_circle * math.sin(a))
              .circle(c.r_pin).extrude(l_pin))
        pins = pn if pins is None else pins.union(pn)
    return ring.union(seat).union(pins)


def make_cam_bearings(c: CycloidParams) -> cq.Workplane:
    """편심 캠 베어링 6002 ×2 (ID 15 / OD 32, 디스크 두께 폭) — 캠 로브와 디스크 보어 사이."""
    out = None
    for i, x in enumerate((c.ecc, -c.ecc)):
        b = (cq.Workplane("XY").center(x, 0).circle(c.d_cam_bearing / 2 - 0.005).circle(15.0)
             .extrude(c.disc_t).translate((0, 0, i * c.disc_t)))
        out = b if out is None else out.union(b)
    return out


def make_eccentric_cam(c: CycloidParams, shaft_d: float) -> cq.Workplane:
    """이중 편심 캠 (180° 위상), 모터축 보어 + 키/세트스크류."""
    j1 = cq.Workplane("XY").center(c.ecc, 0).circle(15.0).extrude(c.disc_t)          # 6002 ID 15
    j2 = (cq.Workplane("XY").center(-c.ecc, 0).circle(15.0).extrude(c.disc_t)
          .translate((0, 0, c.disc_t)))
    cam = j1.union(j2)
    # 보어는 모터 축(원점) 기준 — j1 의 .center(ecc) 원점을 상속하지 않도록 origin 명시
    cam = cam.faces(">Z").workplane(origin=(0, 0, 0)).circle(shaft_d / 2 + 0.01).cutThruAll()
    return cam


def make_output_flange(c: CycloidParams, p: ShoulderParams) -> cq.Workplane:
    """출력 플랜지 — 출력 핀 8개 + 출력 베어링 저널 + 상완 플레이트 체결 보스."""
    ib, ob, wb = p.output_bearing
    t = 10.0
    fl = cq.Workplane("XY").circle(ib / 2).extrude(wb + t)          # 베어링 저널
    cap = cq.Workplane("XY").circle(ob / 2 + 12).extrude(t).translate((0, 0, wb))
    # 캡 하면 릴리프 0.5: 베어링 내륜만 누르고 외륜·하우징 시트와는 간극
    relief = (cq.Workplane("XY").circle(ob / 2 + 12).circle((ib + ob) / 4).extrude(0.5)
              .translate((0, 0, wb)))
    fl = fl.union(cap.cut(relief))
    pins = None
    for k in range(c.n_out_pins):
        a = 2 * math.pi * k / c.n_out_pins
        pn = (cq.Workplane("XY").center(c.r_out_pcd * math.cos(a), c.r_out_pcd * math.sin(a))
              .circle(c.d_out_pin / 2).extrude(2 * c.disc_t + 2).translate((0, 0, -2 * c.disc_t - 2)))
        pins = pn if pins is None else pins.union(pn)
    fl = fl.union(pins)
    # 상완 플레이트 체결 M6 ×6 — 캡 두께만 (관통하면 출력 핀을 자름)
    r_b = p.flange_bolt_pcd / 2
    bolt_pts = [(r_b * math.cos(2 * math.pi * k / 6), r_b * math.sin(2 * math.pi * k / 6)) for k in range(6)]
    fl = fl.faces(">Z").workplane().pushPoints(bolt_pts).circle(3.3).cutBlind(-t)
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


def make_gas_spring(length: float, spec: GasSpringSpec = FGSS22120C, neck: float = 22.0) -> cq.Workplane:
    """가스 스프링 더미 (실린더 Ø d_cyl + 로드 Ø d_rod), X축 방향, 원점=실린더 끝 아이.

    아이: Ø16 × 폭 12, M8 핀 구멍 Ø8.2 (Y 축). 실린더는 아이에서 neck 만큼 떨어져 시작
    (클레비스 치크와 겹치지 않도록). 실린더 끝 = 최소 길이 − 30 (최소 길이에서도 로드 노출 24).
    """
    cyl_len = spec.l_min - 30
    neck_rod = cq.Workplane("YZ").circle(spec.d_rod / 2).extrude(neck - 6).translate((6, 0, 0))
    cyl = cq.Workplane("YZ").circle(spec.d_cyl / 2).extrude(cyl_len - neck).translate((neck, 0, 0))
    rod = (cq.Workplane("YZ").circle(spec.d_rod / 2).extrude(length - 6 - cyl_len)
           .translate((cyl_len, 0, 0)))
    eyes = None
    for x in (0.0, length):
        eye = (cq.Workplane("XZ").circle(8).circle(4.1).extrude(6, both=True)
               .translate((x, 0, 0)))
        eyes = eye if eyes is None else eyes.union(eye)
    return neck_rod.union(cyl).union(rod).union(eyes)


# ---------------------------------------------------------------------------
# 가스 스프링 밸런서 계산
# ---------------------------------------------------------------------------
def gas_spring_arm_anchor(p: ShoulderParams, phi_deg: float = 0.0) -> Tuple[float, float]:
    """팔 각도 φ 에서 상완 앵커 (x, z) — 팔 좌표계, J2 축 원점."""
    a = math.radians(phi_deg + p.gs_arm_deg)
    return p.gs_arm_r * math.cos(a), p.gs_arm_r * math.sin(a)


def gas_spring_state(p: ShoulderParams, phi_deg: float) -> Tuple[float, float, float]:
    """(스프링 길이 mm, 반력 N, 팔을 들어 올리는 토크 N·m) — 압축 스프링이 앵커를 밀어냄."""
    ax, az = gas_spring_arm_anchor(p, phi_deg)
    bx, bz = p.gs_base
    vx, vz = ax - bx, az - bz
    L = math.hypot(vx, vz)
    F = p.gs_spec.force_at(L)
    return L, F, (ax * vz - az * vx) / L * F / 1000.0


def gas_spring_balance(p: ShoulderParams, step: float = 1.0) -> Dict[str, float]:
    """J2 가동 범위 전체에서 스프링 길이·토크와 J2 액추에이터 부담.

    부담 = max(|중력(전개+툴) − 스프링|, |중력(접힘·툴 없음) − 스프링|) × 동적 1.5
    (가스 스프링 반력은 거의 일정이라 cosφ 형 중력을 정확히 따라가지 못함 → 각도별로 확인)
    """
    n = int(round((p.arm_angle_max - p.arm_angle_min) / step))
    lengths, worst, worst_phi = [], 0.0, 0.0
    for k in range(n + 1):
        phi = p.arm_angle_min + k * step
        L, _, tau = gas_spring_state(p, phi)
        g = math.cos(math.radians(phi))
        demand = 1.5 * max(abs(p.torque_static * g - tau), abs(p.torque_static_min * g - tau))
        lengths.append(L)
        if demand > worst:
            worst, worst_phi = demand, phi
    L0, F0, t0 = gas_spring_state(p, 0.0)
    s = p.gs_spec
    return {
        "L_horizontal": L0,
        "L_min": min(lengths), "L_max": max(lengths), "stroke": max(lengths) - min(lengths),
        "margin_compressed": min(lengths) - s.l_min, "margin_extended": s.l_max - max(lengths),
        "force_horizontal": F0,
        "balance_torque_horizontal": t0,
        "gravity_torque_horizontal": p.torque_static,
        "balance_ratio": t0 / p.torque_static,
        "worst_actuator_demand": worst, "worst_demand_angle": worst_phi,
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
        "ring_housing": make_ring_housing(c, p),
        "eccentric_cam": make_eccentric_cam(c, p.motor_shaft_d),
        "output_flange": make_output_flange(c, p),
        "nema23_brake": make_nema23(p),
    }
    z_motor_face = 0.0                     # 모터 전면 = Z 0, 모터는 -Z 쪽
    z_disc0 = ADAPTER_T + 2.0              # 어댑터 플레이트 + 간극
    asm = cq.Assembly(name=p.name)
    asm.add(parts["nema23_brake"].translate((0, 0, -p.motor_len)), name="motor", color=cq.Color(0.2, 0.2, 0.25))
    adapter = (cq.Workplane("XY").circle(adapter_radius(p)).extrude(ADAPTER_T)
               .faces(">Z").workplane().circle(20).cutThruAll())
    asm.add(adapter, name="adapter", color=cq.Color(0.6, 0.6, 0.62))
    asm.add(parts["ring_housing"].translate((0, 0, ADAPTER_T)), name="ring", color=cq.Color(0.55, 0.55, 0.6))
    asm.add(parts["eccentric_cam"].translate((0, 0, z_disc0)), name="cam", color=cq.Color(0.85, 0.7, 0.1))
    asm.add(make_cam_bearings(c).translate((0, 0, z_disc0)), name="cam_bearing", color=cq.Color(0.8, 0.8, 0.8))
    d1 = parts["cycloid_disc"].translate((c.ecc, 0, z_disc0))
    d2 = parts["cycloid_disc"].rotate((0, 0, 0), (0, 0, 1), c.disc2_phase).translate((-c.ecc, 0, z_disc0 + c.disc_t))
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
    # 가스 스프링: 베이스 앵커 → 상완 앵커, 수평 자세 (조인트 프레임에서 팔 피치 평면 = XY)
    if p.has_gas_spring:
        (bx, bz), (ax, az) = p.gs_base, gas_spring_arm_anchor(p, 0.0)
        L = math.hypot(ax - bx, az - bz)
        ang = math.degrees(math.atan2(az - bz, ax - bx))
        spring = (make_gas_spring(L, p.gs_spec).rotate((0, 0, 0), (0, 0, 1), ang)
                  .translate((bx, bz, z_fl + wb + 10 + p.arm_plate_t + 15)))
        asm.add(spring, name="gas_spring", color=cq.Color(0.2, 0.7, 0.3))
    return asm, parts


def joint_stack_length(p: ShoulderParams) -> float:
    """모터 전면(Z=0)에서 출력 플랜지 상면까지의 축방향 길이."""
    return ADAPTER_T + 2.0 + 2 * p.cyc.disc_t + 2 + p.output_bearing[2] + 10.0


def adapter_radius(p: ShoulderParams) -> float:
    """어댑터 플레이트·링 하우징 외경 반경."""
    c = p.cyc
    return c.r_pin_circle + c.r_pin + c.housing_t


def elbow_params() -> ShoulderParams:
    """J3 팔꿈치: 1:45 (핀 46), NEMA23 3.0 N·m (57×76), 가스 스프링 없음, 브레이크 있음."""
    cyc = CycloidParams(pins=46, r_pin_circle=63.0, r_pin=2.5, ecc=0.6, r_out_pcd=36.0)
    return ShoulderParams(cyc=cyc, motor_len=76.0, name="J3_elbow", has_gas_spring=False,
                          motor_holding=3.0, torque_static=30.1,
                          output_bearing=(55.0, 72.0, 9.0), flange_bolt_pcd=76.0)


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
        s = p.gs_spec
        lines += ["", f"[가스 스프링 밸런서]  {s.name} (스트로크 {s.stroke:.0f}, {s.l_min:.0f}~{s.l_max:.0f}, "
                      f"{s.force:.0f} N), 베이스 앵커 {p.gs_base}, 상완 앵커 r {p.gs_arm_r:.0f} / δ {p.gs_arm_deg:+.0f}°"]
        lines += [f"  {k:28s}: {v:8.1f}" for k, v in gs.items()]
    txt = "\n".join(lines)
    (out / "design_summary.txt").write_text(txt, encoding="utf-8")
    return txt


if __name__ == "__main__":
    print(export(ShoulderParams(), Path(__file__).resolve().parents[1] / "output" / "shoulder"))
    print()
    print(export(elbow_params(), Path(__file__).resolve().parents[1] / "output" / "elbow"))
