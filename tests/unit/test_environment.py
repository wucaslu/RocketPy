import json
import os

import numpy as np
import numpy.ma as ma
import pytest
import pytz

from rocketpy import Environment


@pytest.mark.parametrize(
    "latitude, longitude", [(-21.960641, -47.482122), (0, 0), (21.960641, 47.482122)]
)
def test_location_set_location_saves_location(latitude, longitude, example_plain_env):
    """Tests location is saved correctly in the environment obj.

    Parameters
    ----------
    example_plain_env : rocketpy.Environment
    latitude: float
        The latitude in decimal degrees.
    longitude: float
        The longitude in decimal degrees.
    """
    example_plain_env.set_location(latitude, longitude)
    assert example_plain_env.latitude == latitude
    assert example_plain_env.longitude == longitude


@pytest.mark.parametrize("elevation", [(-200), (0), (200)])
def test_elevation_set_elevation_saves_elevation(elevation, example_plain_env):
    """Tests the wether the 'set_elevation' method within the Environment class
    sets the elevation correctly.

    Parameters
    ----------
    example_plain_env : rocketpy.Environment
    """

    example_plain_env.set_elevation(elevation=elevation)
    assert example_plain_env.elevation == elevation


@pytest.mark.parametrize(
    "latitude, longitude, theoretical_elevation",
    [(46.90479, 8.07575, 1565), (46.00001, 8.00001, 2562), (46.99999, 8.99999, 2832)],
)
def test_location_set_topographic_profile_computes_elevation(
    latitude, longitude, theoretical_elevation, example_plain_env
):
    """Tests elevation computation given topographic profile in the environment obj.

    Parameters
    ----------
    example_plain_env : rocketpy.Environment
    latitude: float
        The latitude in decimal degrees.
    longitude: float
        The longitude in decimal degrees.
    """
    example_plain_env.set_topographic_profile(
        type="NASADEM_HGT",
        file="data/sites/switzerland/NASADEM_NC_n46e008.nc",
        dictionary="netCDF4",
    )
    computed_elevation = example_plain_env.get_elevation_from_topographic_profile(
        latitude, longitude
    )
    assert computed_elevation == theoretical_elevation


def test_geodesic_coordinate_geodesic_to_utm_converts_coordinate():
    """Tests the conversion from geodesic to UTM coordinates."""
    x, y, utm_zone, utm_letter, hemis, EW = Environment.geodesic_to_utm(
        lat=32.990254,
        lon=-106.974998,
        semi_major_axis=6378137.0,  # WGS84
        flattening=1 / 298.257223563,  # WGS84
    )
    assert np.isclose(x, 315468.64, atol=1e-5) == True
    assert np.isclose(y, 3651938.65, atol=1e-5) == True
    assert utm_zone == 13
    assert utm_letter == "S"
    assert hemis == "N"
    assert EW == "W"


def test_utm_to_geodesic_converts_coordinates():
    """Tests the utm_to_geodesic method within the Environment
    class and checks the conversion results from UTM to geodesic
    coordinates.
    """

    lat, lon = Environment.utm_to_geodesic(
        x=315468.64,
        y=3651938.65,
        utm_zone=13,
        hemis="N",
        semi_major_axis=6378137.0,  # WGS84
        flattening=1 / 298.257223563,  # WGS84
    )
    assert np.isclose(lat, 32.99025, atol=1e-5) == True
    assert np.isclose(lon, -106.9750, atol=1e-5) == True


@pytest.mark.parametrize(
    "latitude, theoretical_radius",
    [(0, 6378137.0), (90, 6356752.31424518), (-90, 6356752.31424518)],
)
def test_latitude_calculate_earth_radius_computes_radius(latitude, theoretical_radius):
    """Tests earth radius calculation.

    Parameters
    ----------
    latitude : float
        The latitude in decimal degrees.
    theoretical_radius : float
        The expected radius in meters at the given latitude.
    """
    semi_major_axis = 6378137.0  # WGS84
    flattening = 1 / 298.257223563  # WGS84
    computed_radius = Environment.calculate_earth_radius(
        latitude, semi_major_axis, flattening
    )
    assert pytest.approx(computed_radius, abs=1e-8) == theoretical_radius


@pytest.mark.parametrize(
    "angle, theoretical_degree, theoretical_arc_minutes, theoretical_arc_seconds",
    [
        (-106.974998, -106.0, 58, 29.9928),
        (32.990254, 32, 59, 24.9144),
        (90.0, 90, 0, 0),
    ],
)
def test_decimal_degrees_to_arc_seconds_computes_correct_values(
    angle, theoretical_degree, theoretical_arc_minutes, theoretical_arc_seconds
):
    """Tests the conversion from decimal degrees to arc minutes and arc seconds.

    Parameters
    ----------
    angle : float
        Angle in decimal degrees.
    theoretical_degree : int
        Expected computed integer degrees.
    theoretical_arc_minutes : int
        Expected computed arc minutes.
    theoretical_arc_seconds : float
        Expected computed arc seconds.
    """
    computed_data = Environment.decimal_degrees_to_arc_seconds(angle)

    assert pytest.approx(computed_data[0], abs=1e-8) == theoretical_degree
    assert pytest.approx(computed_data[1], abs=1e-8) == theoretical_arc_minutes
    assert pytest.approx(computed_data[2], abs=1e-8) == theoretical_arc_seconds
