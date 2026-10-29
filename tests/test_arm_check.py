"""팔 정적 토크·부품 강도 검증."""

from __future__ import annotations

import pytest

from orchard_rover.arm_check import joint_static_torques, plate_strength, torque_about


def test_torque_about_matches_joint_torques() -> None:
    t = joint_static_torques()
    assert torque_about(0.0) == pytest.approx(t["J2_shoulder"])
    assert torque_about(500.0) == pytest.approx(t["J3_elbow"])


@pytest.fixture(scope="module")
def strength():
    return plate_strength()


def test_changed_parts_safety_factor(strength) -> None:
    """형상 변경 부품 (J1 브라켓 10t·전완판 10t·J4 하우징 시트·J5 축) 항복 안전율 ≥ 3 (동적 ×1.5 포함)."""
    for part, r in strength.items():
        assert r["SF"] >= 3.0, part


def test_bracket_tip_contribution_small(strength) -> None:
    """J1 브라켓 면외 휨이 팔 끝에 주는 변위가 팔 자체 처짐의 10% 이하."""
    from orchard_rover.arm_check import tip_deflection

    assert strength["J1_bracket"]["tip_mm"] <= 0.1 * tip_deflection()["tip_total"]
