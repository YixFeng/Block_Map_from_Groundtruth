import os
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

data_path = '/home/yixfeng/Block-SLAM/gt/M2DGR/street_02/street_02.txt'
out_path = '/home/yixfeng/Block-SLAM/gt/M2DGR/street_02/street_02_after.txt'
# street_01 origin long=121.43686746° lat=31.02607212° h=16.226m
# street_02 origin long=121.44075346° lat=31.02489017° h=16.489m

a = 6378137
b = 6356752.3142
f = (a - b) / a
e_sq = f * (2 - f)
lon0 = 121.44075346
lat0 = 31.02489017
h0 = 16.489

def geodetic_to_ecef(lat, lon, h):
    # (lat, lon) in WSG-84 degrees
    # h in meters
    lamb = math.radians(lat)
    phi = math.radians(lon)
    s = math.sin(lamb)
    N = a / math.sqrt(1 - e_sq * s * s)

    sin_lambda = math.sin(lamb)
    cos_lambda = math.cos(lamb)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)

    x = (h + N) * cos_lambda * cos_phi
    y = (h + N) * cos_lambda * sin_phi
    z = (h + (1 - e_sq) * N) * sin_lambda

    return x, y, z

def ecef_to_enu(x, y, z, lat0, lon0, h0):
    lamb = math.radians(lat0)
    phi = math.radians(lon0)
    s = math.sin(lamb)
    N = a / math.sqrt(1 - e_sq * s * s)

    sin_lambda = math.sin(lamb)
    cos_lambda = math.cos(lamb)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)

    x0 = (h0 + N) * cos_lambda * cos_phi
    y0 = (h0 + N) * cos_lambda * sin_phi
    z0 = (h0 + (1 - e_sq) * N) * sin_lambda

    xd = x - x0
    yd = y - y0
    zd = z - z0

    xEast = -sin_phi * xd + cos_phi * yd
    yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd
    zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd

    return xEast, yNorth, zUp

def geodetic_to_enu(lat, lon, h, lat_ref, lon_ref, h_ref):
    x, y, z = geodetic_to_ecef(lat, lon, h)

    return ecef_to_enu(x, y, z, lat_ref, lon_ref, h_ref)

if __name__ == "__main__":
    first_data_ = []
    x0, y0, z0 = 0, 0, 0
    with open(data_path, 'r') as gt:
        with open(out_path, 'w') as gt_process:
            for (num,line) in enumerate(gt):
                data = line.rstrip('\n').split()
                if num == 0:
                    first_data_ = list(map(float, data))
                    x0, y0, z0 = ecef_to_enu(first_data_[1], first_data_[2], first_data_[3], lat0, lon0, h0)
                data_ = list(map(float, data))
                time = data_[0]
                x, y, z = ecef_to_enu(data_[1], data_[2], data_[3], lat0, lon0, h0)

                t_first = np.array([x0, y0, z0])
                q_first = [first_data_[4], first_data_[5], first_data_[6], first_data_[7]]
                t = np.array([x, y, z])
                q = [data_[4], data_[5], data_[6], data_[7]]

                rotation = R.from_quat(q)
                rotation_mat = rotation.as_matrix()
                rotation_first = R.from_quat(q_first)
                rotation_first_mat = rotation_first.as_matrix()

                final_mat = np.dot(rotation_first_mat.transpose(), rotation_mat)
                t_final = np.dot(rotation_first_mat.transpose(), t) - np.dot(rotation_first_mat.transpose(), t_first)

                # quaternion format is "xyzw"
                q2 = R.from_matrix(final_mat)
                q_out = q2.as_quat()
                gt_process.write(str(time) + " " + str(t_final[0]) + " " + str(t_final[1]) + " " + str(t_final[2]) + " " + str(q_out[0]) + " " + str(q_out[1]) + " " + str(q_out[2]) + " " + str(q_out[3]) + '\n')
        gt_process.close()
    gt.close()