import numpy as np
import matplotlib.pyplot as plt

def next_waypoint(x, y, mapx, mapy):
    closest_wp = get_closest_waypoints(x, y, mapx, mapy)

    if closest_wp>=len(mapx)-1:
        closest_wp -=1
    map_vec = [mapx[closest_wp + 1] - mapx[closest_wp], mapy[closest_wp + 1] - mapy[closest_wp]]
    ego_vec = [x - mapx[closest_wp], y - mapy[closest_wp]]

    direction  = np.sign(np.dot(map_vec, ego_vec))

    if direction >= 0:
        next_wp = closest_wp + 1
    else:
        next_wp = closest_wp

    return next_wp

def get_closest_waypoints(x, y, mapx, mapy):
    min_len = 1e10
    closeset_wp = 0

    for i in range(len(mapx)):
        _mapx = mapx[i]
        _mapy = mapy[i]
        dist = get_dist(x, y, _mapx, _mapy)

        if dist < min_len:
            min_len = dist
            closest_wp = i

    return closest_wp

def get_dist(x, y, _x, _y):
    return np.sqrt((x - _x)**2 + (y - _y)**2)

def get_frenet(x, y, mapx, mapy):
    next_wp = next_waypoint(x, y, mapx, mapy)
    prev_wp = next_wp -1

    n_x = mapx[next_wp] - mapx[prev_wp]
    n_y = mapy[next_wp] - mapy[prev_wp]
    x_x = x - mapx[prev_wp]
    x_y = y - mapy[prev_wp]

    try:
        proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y)
    except:
        print("ZeroDivisionError")
        return 0, 0
    proj_x = proj_norm*n_x
    proj_y = proj_norm*n_y

    #-------- get frenet d
    frenet_d = get_dist(x_x,x_y,proj_x,proj_y)

    ego_vec = [x-mapx[prev_wp], y-mapy[prev_wp], 0]
    map_vec = [n_x, n_y, 0]
    d_cross = np.cross(ego_vec,map_vec)
    
    if d_cross[-1] > 0:
        frenet_d = -frenet_d

    #-------- get frenet s
    frenet_s = 0
    for i in range(prev_wp):
        frenet_s = frenet_s + get_dist(mapx[i],mapy[i],mapx[i+1],mapy[i+1])

    frenet_s = frenet_s + get_dist(0,0,proj_x,proj_y)

    # plt.figure(figsize=(8, 6))
    # plt.plot(mapx, mapy, 'k--', label='Map')
    # plt.pot(x, y, 'ro', label='Current POsition')
    # plt.text(x-5, y+5, f'Frenet s: {frenet_s:.2f}', fontsize=12)
    # plt.arrow(mapx[prev_wp], mapy[prev_wp], proj_x/2, proj_y/2, width=0.5, head_width=3, head_length=3, fc='blue', ec='blue', length_includes_head=True)
    # plt.text(mapx[prev_wp] + proj_x/2 - 10, mapy[prev_wp] + proj_y/2 - 10, f'Frent d: {frenet_d:.2f}', fontsize=12)

    # plt.legend()
    # plt.grid(True)
    # plt.axis('equal')
    # plt.show()

    return frenet_s, frenet_d

def get_cartesian(s, d, mapx, mapy, maps):
    prev_wp = 0

#     s = np.mod(s, maps[-2])

    while(s > maps[prev_wp+1]) and (prev_wp < len(maps)-2):
        prev_wp = prev_wp + 1

    next_wp = np.mod(prev_wp+1,len(mapx))

    dx = (mapx[next_wp]-mapx[prev_wp])
    dy = (mapy[next_wp]-mapy[prev_wp])

    heading = np.arctan2(dy, dx) # [rad]

    # the x,y,s along the segment
    seg_s = s - maps[prev_wp]

    seg_x = mapx[prev_wp] + seg_s*np.cos(heading)
    seg_y = mapy[prev_wp] + seg_s*np.sin(heading)

    perp_heading = heading + 90 * np.pi/180
    x = seg_x + d*np.cos(perp_heading)
    y = seg_y + d*np.sin(perp_heading)

    return x, y, heading

