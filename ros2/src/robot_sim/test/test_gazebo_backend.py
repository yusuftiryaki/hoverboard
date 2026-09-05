"""Gazebo backend conversion tests; no Gazebo process required."""

import math

import pytest

from robot_sim.gazebo_backend import GazeboBackend


def test_wheel_units_round_trip_through_mps_conversion():
    backend = object.__new__(GazeboBackend)
    backend._radius = 0.0825
    backend._units_per_rpm = 1.0
    speed = 0.75
    units = backend._mps_to_units(speed)
    assert backend._units_to_mps(units) == pytest.approx(speed)


def test_differential_wheel_units_have_expected_turn_rate():
    backend = object.__new__(GazeboBackend)
    backend._radius = 0.0825
    backend._separation = 0.5
    backend._units_per_rpm = 1.0
    left = backend._units_to_mps(backend._mps_to_units(0.25))
    right = backend._units_to_mps(backend._mps_to_units(0.75))
    assert (right - left) / backend._separation == pytest.approx(1.0)
    assert math.isfinite(left)