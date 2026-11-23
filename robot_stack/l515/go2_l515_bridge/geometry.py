"""Vectorized XYZ projection and RealSense extrinsics, independent of ROS."""
import numpy as np

def xyz_from_depth(raw, intr, scale, stride=2, max_range=9.0):
    if stride < 1:
        raise ValueError('stride must be >= 1')
    fx, fy, cx, cy = intr
    v, u = np.mgrid[0:raw.shape[0]:stride, 0:raw.shape[1]:stride]
    z = raw[::stride, ::stride].astype(np.float32) * scale
    good = (z > 0.15) & (z <= max_range) & np.isfinite(z)
    x = (u.astype(np.float32) - cx) * z / fx
    y = (v.astype(np.float32) - cy) * z / fy
    return np.stack((x[good], y[good], z[good]), axis=1).astype('<f4')

def quaternion_from_matrix(m):
    """Quaternion (x,y,z,w) for 3x3 proper rotation matrix."""
    m = np.asarray(m, dtype=np.float64)
    q = np.empty(4, dtype=np.float64)
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        q[:] = [(m[2,1]-m[1,2])/s, (m[0,2]-m[2,0])/s,
                (m[1,0]-m[0,1])/s, 0.25*s]
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i+1)%3, (i+2)%3
        s = np.sqrt(1 + m[i,i] - m[j,j] - m[k,k]) * 2
        q[i] = 0.25*s
        q[j] = (m[i,j] + m[j,i]) / s
        q[k] = (m[i,k] + m[k,i]) / s
        q[3] = (m[k,j] - m[j,k]) / s
    q /= np.linalg.norm(q)
    return q

def color_pose_in_depth(extr):
    """SDK p_color = R * p_depth + t; TF needs color->depth (R.T, -R.T*t)."""
    r = np.array(extr[:9], dtype=np.float64).reshape(3, 3, order='F')
    t = np.array(extr[9:12], dtype=np.float64)
    return -r.T @ t, quaternion_from_matrix(r.T)
