"""전체 로봇 부품 연결(지지 경로)과 관절 가동 범위 간섭 검증."""

from __future__ import annotations

import pytest

cq = pytest.importorskip("cadquery")

from orchard_rover.motion_check import (LIMITS, _bb_overlap, collisions, load_parts,  # noqa: E402
                                        mating_pairs, posed)

FIT_TOL = 0.06   # 반경 끼워맞춤 여유 ≤ 0.05 (휠 보어 Ø16.1 / 축 Ø16 등) 는 접촉으로 본다


@pytest.fixture(scope="module")
def parts():
    return load_parts()


@pytest.fixture(scope="module")
def mates(parts):
    return mating_pairs(posed(parts, {}))


def test_every_part_supported_from_frame(parts) -> None:
    """홈 자세에서 모든 부품이 접촉(끼워맞춤 포함)으로 프레임까지 이어진다 — 떠 있는 부품 없음."""
    home = posed(parts, {})
    names = list(home)
    bbs = {n: home[n][1].BoundingBox() for n in names}
    adj = {n: set() for n in names}
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if _bb_overlap(bbs[a], bbs[b], FIT_TOL) and home[a][1].distance(home[b][1]) <= FIT_TOL:
                adj[a].add(b)
                adj[b].add(a)
    seen, stack = {"rover:frame_rail_low_L"}, ["rover:frame_rail_low_L"]
    while stack:
        for m in adj[stack.pop()] - seen:
            seen.add(m)
            stack.append(m)
    assert sorted(set(names) - seen) == []


@pytest.mark.parametrize("joint", sorted(LIMITS))
def test_single_joint_limits_clear(parts, mates, joint) -> None:
    """각 관절 한계·중간 자세 (나머지 0) 에서 링크 간 간섭 없음."""
    lo, hi = LIMITS[joint]
    for a in (lo, (lo + hi) / 2, hi):
        assert collisions(posed(parts, {joint: a}), mates) == [], f"{joint}={a}"
