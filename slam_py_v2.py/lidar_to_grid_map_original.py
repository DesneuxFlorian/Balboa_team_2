"""
LIDAR to 2D grid map example
author: Erno Horvath, Csaba Hajdu based on Atsushi Sakai's scripts
"""

import math
import matplotlib.pyplot as plt
import numpy as np
from roboviz import MapVisualizer
from time import time



def file_read(f):
    """
    Reading LIDAR laser beams (angles and corresponding distance data)
    """
    with open(f) as data:
        measures = [line.split(",") for line in data]
    angles = []
    distances = []
    for measure in measures:
        angles.append(float(measure[0]))
        distances.append(float(measure[1]))
    angles = np.array(angles)
    distances = np.array(distances)
    return angles, distances


def bresenham(start, end):
    """
    Implementation of Bresenham's line drawing algorithm
    See en.wikipedia.org/wiki/Bresenham's_line_algorithm
    Bresenham's Line Algorithm
    Produces a np.array from start and end (original from roguebasin.com)
    >>> points1 = bresenham((4, 4), (6, 10))
    >>> print(points1)
    np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
    """
    # setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
    is_steep = abs(dy) > abs(dx)  # determine how steep the line is
    if is_steep:  # rotate line
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    # swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
    dx = x2 - x1  # recalculate differentials
    dy = y2 - y1  # recalculate differentials
    error = int(dx / 2.0)  # calculate error
    y_step = 1 if y1 < y2 else -1
    # iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = [y, x] if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += y_step
            error += dx
    if swapped:  # reverse the list if the coordinates were swapped
        points.reverse()
    points = np.array(points)
    return points




def atan_zero_to_twopi(y, x):
    angle = math.atan2(y, x)
    if angle < 0.0:
        angle += math.pi * 2.0
    return angle


def generate_ray_casting_grid_map(robotPos,ox, oy, od, map_size_pixel,occupancy_map, breshen=True):
    """
    The breshen boolean tells if it's computed with bresenham ray casting
    (True) or with flood fill (False)
    """

    


    center_y,center_x=robotPos
    center_x=int(center_x)
    center_y=int(center_y)
    

    if breshen:
        for (x, y ,d) in zip(ox, oy,od):
            # x coordinate of the the occupied area
            ix = int(round(x+center_x))
            # y coordinate of the the occupied area
            iy = int(round(y+center_y))
            laser_beams = bresenham((center_x, center_y), (
                ix, iy))  # line form the lidar to the occupied point
            for laser_beam in laser_beams:

                if laser_beam[0]>=0 and laser_beam[1]>=0 and laser_beam[0]<=(map_size_pixel-1) and laser_beam[1]<=(map_size_pixel-1):
                    occupancy_map[laser_beam[0]][
                        laser_beam[1]] = 255  # free area 255

            if d>=1200:#if dist =1m no object detected because that is the lidar max range
                pass
            elif ix>=0 and iy>=0 and ix<=(map_size_pixel-2) and iy<=(map_size_pixel-2):
                occupancy_map[ix][iy] = 0  # occupied area 0
                occupancy_map[ix + 1][iy] = 0  # extend the occupied area
                occupancy_map[ix][iy + 1] = 0  # extend the occupied area
                occupancy_map[ix + 1][iy + 1] = 0  # extend the occupied area

        # occupancy grid computed with with flood fill

    return occupancy_map

MAP_SIZE_PIXELS =250
MAP_SIZE_METERS = 10
SPEED_MPS=0

def main():
    """
    Example usage
    """
    print(__file__, "start")
    angG, distG = file_read("lidar01.csv")
    ox = np.cos(angG) * distG*MAP_SIZE_PIXELS/MAP_SIZE_METERS
    oy = np.sin(angG) * distG*MAP_SIZE_PIXELS/MAP_SIZE_METERS
    map=MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, title='MapVisualizer',show_trajectory=True)
    pose = np.array([MAP_SIZE_METERS/2,MAP_SIZE_METERS/2,360*np.random.random()])

    # Start timing
    prevtime = time()
    occupancy_map = np.ones((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS),np.uint8) *127


    for i in range(0,len(angG)-40,10):
        oxa=ox[i:i+40]
        oya=oy[i:i+40]
        od=distG[i:i+40]
        occupancy_map=generate_ray_casting_grid_map((pose[0]*MAP_SIZE_PIXELS/MAP_SIZE_METERS,pose[1]*MAP_SIZE_PIXELS/MAP_SIZE_METERS),oxa, oya, od,MAP_SIZE_PIXELS,occupancy_map, True)
        # print(occupancy_map)
        # print(occupancy_map.dtype)
        # print(np.size(occupancy_map))
        # print(np.size(np.frombuffer(occupancy_map, dtype=np.float16)))
        if not map.display(*pose,occupancy_map):
            exit(0)
        #print((pose[0]*MAP_SIZE_PIXELS/MAP_SIZE_METERS,pose[1]*MAP_SIZE_PIXELS/MAP_SIZE_METERS))

        currtime = time()
        s = SPEED_MPS * (currtime - prevtime)
        prevtime = currtime
        theta = np.radians(pose[2])
        pose[0] += s * np.cos(theta)
        pose[1] += s * np.sin(theta)
        pose[2] += 10 * np.random.randn()
    plt.pause(10000)


if __name__ == '__main__':
    main()