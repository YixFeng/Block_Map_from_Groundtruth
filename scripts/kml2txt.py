import math
import numpy as np
from math import sin, cos, radians
CONSTANTS_RADIUS_OF_EARTH = 6371000.     # meters (m)

def lon_lat_to_xyz(lon, lat, R = 6371.0088):
    '''
    Convert lon, lat in degrees to x, y, z coordinates.
    lon: longitude in degrees
    lat: latitude in degrees
    R: radius of the earth
    return: x, y, z
    '''
    lon, lat = radians(lon), radians(lat)
    x = R * cos(lat) * cos(lon)
    y = R * cos(lat) * sin(lon)
    z = R * sin(lat)
    return x, y, z

def xyz_to_lon_lat(x, y, z):
    '''
    Convert x, y, z coordinates to lon, lat in degrees.
    x: x coordinate
    y: y coordinate
    z: z coordinate
    return: lon, lat in degrees
    '''
    lon = np.arctan2(y, x)
    lat = np.arctan2(z, np.sqrt(x**2 + y**2))
    return np.degrees(lon), np.degrees(lat)

def GPStoXY( lat, lon, ref_lat, ref_lon, CONSTANTS_RADIUS_OF_EARTH=6371000.):
        # input GPS and Reference GPS in degrees
        # output XY in meters (m) X:North Y:East
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        ref_lat_rad = math.radians(ref_lat)
        ref_lon_rad = math.radians(ref_lon)

        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        ref_sin_lat = math.sin(ref_lat_rad)
        ref_cos_lat = math.cos(ref_lat_rad)

        cos_d_lon = math.cos(lon_rad - ref_lon_rad)

        arg = np.clip(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon, -1.0, 1.0)
        c = math.acos(arg)

        k = 1.0
        if abs(c) > 0:
            k = (c / math.sin(c))

        x = float(k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH)
        y = float(k * cos_lat * math.sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH)

        return x, y

def XYtoGPS(self,x, y, ref_lat, ref_lon):
        x_rad = float(x) / self.CONSTANTS_RADIUS_OF_EARTH
        y_rad = float(y) / self.CONSTANTS_RADIUS_OF_EARTH
        c = math.sqrt(x_rad * x_rad + y_rad * y_rad)

        ref_lat_rad = math.radians(ref_lat)
        ref_lon_rad = math.radians(ref_lon)

        ref_sin_lat = math.sin(ref_lat_rad)
        ref_cos_lat = math.cos(ref_lat_rad)

        if abs(c) > 0:
            sin_c = math.sin(c)
            cos_c = math.cos(c)

            lat_rad = math.asin(cos_c * ref_sin_lat + (x_rad * sin_c * ref_cos_lat) / c)
            lon_rad = (ref_lon_rad + math.atan2(y_rad * sin_c, c * ref_cos_lat * cos_c - x_rad * ref_sin_lat * sin_c))

            lat = math.degrees(lat_rad)
            lon = math.degrees(lon_rad)

        else:
            lat = math.degrees(ref_lat)
            lon = math.degrees(ref_lon)

        return lat, lon


if __name__ == "__main__":
    data = lon_lat_to_xyz(114.17960962355994,22.331885859664602, R = 6371.0088)
    data2 = GPStoXY( 114.17960962355994,22.331885859664602, 114.17953745456023,22.330503634739262, CONSTANTS_RADIUS_OF_EARTH=6371000.)
    print(data)
    print(data2)