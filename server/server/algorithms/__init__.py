"""Алгоритмы и профили управления."""

from .profiles import (
    AutonomyProfile1,
    BaseControlProfile,
    FollowingProfile,
    PauseProfile,
    SillyFollowingProfile,
    StaringProfile,
    TeleoperationProfile,
    TeleopSlamProfile,
)

__all__ = [
    "BaseControlProfile",
    "PauseProfile",
    "TeleoperationProfile",
    "TeleopSlamProfile",
    "StaringProfile",
    "SillyFollowingProfile",
    "AutonomyProfile1",
    "FollowingProfile",
]
