
import numpy as np

def plane_from_three_points(p1, p2, p3):
    ''' Computes a plane in space function given three points
    code taken from: https://www.geeksforgeeks.org/program-to-find-equation-of-a-plane-passing-through-3-points/
        It returns the plane as a function ax + by + cz + d = 0
    '''

    # finding two vectors in the plane
    a1 = p2[0] - p1[0] # x2 - x1
    b1 = p2[1] - p1[1] # y2 - y1
    c1 = p2[2] - p1[2] # z2 - z1

    a2 = p3[0] - p1[0] # x3 - x1
    b2 = p3[1] - p1[1] # y3 - y1
    c2 = p3[2] - p1[2] # z3 - z1

    # computing the plane coefficients
    a = b1 * c2 - b2 * c1
    b = a2 * c1 - a1 * c2
    c = a1 * b2 - b1 * a2
    d = (- a * p1[0] - b * p1[1] - c * p1[2])

    return {'a':a, 'b':b, 'c':c, 'd':d}


def dist_from_point_to_plane(point, plane):
    '''Computes the perpendicular distance from a 3D point vector 
    to a plane defined as ax + by + cz + d = 0
    Formula taken from V ittal - analytical geometry
    '''
    return abs((plane['a']*point[0] + plane['b']*point[1] + plane['c']*point[2] + plane['d'])/ np.sqrt( plane['a']**2 + plane['b']**2 + plane['c']**2))


def dist_frame_origin_to_plane(plane):
    ''' Computes the distance from frame origin to a 
    a plane defined as ax + by + cz + d = 0.
    It is similar to dist_from_point_to_plane, but with fewer computations (to gain speed).
    Formula taken from vittal - analytical geometry
    '''
    return abs( plane['d']) / np.sqrt( plane['a']**2 + plane['b']**2 + plane['c']**2)


def dist_point_to_plane_along_vector(pt, pl, v):
    ''' Computes the distance from a point to a plane along a unit vector v
        Input:
            -pt <list 3 elements>: point vector
            -pl <dict>: dictionary containing the plane defined by ax + by + cz + d = 0
            -v <list 3 elements>: unit vector along which the distance is computed
        Output:
            -dist: distance from the point to the plane.
            -touchp: vector containing the point where the projection intersects the plane.
    '''

    # computing the distance
    dist = - (pl['a']*pt[0] + pl['b']*pt[1] + pl['c']*pt[2] + pl['d']) / (v[0]*pl['a'] +  v[1]*pl['b'] + v[2]*pl['c'])

    # intersection point
    touchp = [pt[0]+v[0]*dist, pt[1]+v[1]*dist, pt[2]+v[2]*dist]

    return dist, touchp


def projectionV1toV2_norm(v1, v2):
    '''Return magnitude of the projection vector of v1 onto v2 '''
    return np.dot(v1, v2)/np.linalg.norm(v2)