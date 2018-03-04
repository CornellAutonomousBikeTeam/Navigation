import pyproj
import numpy as np

# Documentation: https://jswhit.github.io/pyproj/pyproj.Geod-class.html


ORIG_LAT =  42.444393
ORIG_LON = -76.483509


_geod = pyproj.Geod(ellps='WGS84')


# Projects a (latitude, longitude) point to a local (x, y) coordinate system in meters.
def global_to_local(lat, lon, origin=(ORIG_LAT, ORIG_LON)):
    azimuth, _, dist = _geod.inv(
        lons1=origin[1], lats1=origin[0], lons2=lon, lats2=lat)
    theta = -1 * np.deg2rad(azimuth - 90)
    return dist * np.cos(theta), dist * np.sin(theta)


# Inverse projection, takes a local (x, y) to global (latitude, longitude). 
def local_to_global(x, y, origin=(ORIG_LAT, ORIG_LON)):
    azimuth = 90 - np.rad2deg(np.arctan2(y, x))
    dist = np.linalg.norm([x, y])
    lat, lon, _ = _geod.fwd(origin[1], origin[0], azimuth, dist)
    return lon, lat
