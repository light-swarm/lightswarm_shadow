#!/usr/bin/env python

import rospy

from lightswarm_core.msg import Point # [x,y]

from lightswarm_core.msg import Cylinder # [point, height, radius]
from lightswarm_core.msg import Objects # list of cylinders

from lightswarm_core.msg import Polygon # list of points
from lightswarm_core.msg import Shadow # [projector id, polygon]

from lightswarm_core.msg import Penumbras # list of shadows
from lightswarm_core.msg import Obstacles # list of polygons

import math
import shapely.geometry as gm
import numpy as np

import yaml

CONFIG_FILENAME = 'src/lightswarm_core/params/config.yaml'

def vec_normalize(vec): # vec: np.array
    return vec / np.linalg.norm(vec)

def get_ortho_vec(vec): # vec: np.array
    ortho_vec = np.array([1, -vec[0]/vec[1]], float)
    return vec_normalize(ortho_vec)

def get_dist(vec1, vec2):
    return np.linalg.norm(vec1 - vec2)

def get_shadow_point(vertex, light): # vertex: np.array, light_pos: np.array / both 3d
    alpha = vertex[2] / (light[2] - vertex[2])
    x_shadow = (vertex[0] * light[2] - light[0] * vertex[2]) / (light[2] - vertex[2])
    y_shadow = (vertex[1] * light[2] - light[1] * vertex[2]) / (light[2] - vertex[2])
    return (x_shadow, y_shadow)

def get_tangent_points(cylinder, light3d): # cylinder: Cylinder, light3d: tuple
    k1 = cylinder.location.x - light3d[0]
    k2 = cylinder.location.y - light3d[1]
    k3 = cylinder.radius

    if k2 == 0:
        theta1 = math.pi/2
        theta2 = -math.pi/2
    else:
        x1 = (-k1 * k3 - math.sqrt(k1*k1*k2*k2 + k2**4 - k2*k2*k3*k3)) / (k1*k1 + k2*k2)
        y1 = (-k3 + (k1*k1*k3)/(k1*k1+k2*k2) + k1*math.sqrt(-k2*k2*(-k1*k1-k2*k2+k3*k3))/(k1*k1 + k2*k2))/k2
        
        x2 = (-k1 * k3 + math.sqrt(k1*k1*k2*k2 + k2**4 - k2*k2*k3*k3)) / (k1*k1 + k2*k2)
        y2 = (-k3 + (k1*k1*k3)/(k1*k1+k2*k2) - k1*math.sqrt(-k2*k2*(-k1*k1-k2*k2+k3*k3))/(k1*k1 + k2*k2))/k2

        theta1 = np.arctan2(y1, x1)
        theta2 = np.arctan2(y2, x2)
    
    tx1 = cylinder.location.x + cylinder.radius * np.cos(theta1)
    ty1 = cylinder.location.y + cylinder.radius * np.sin(theta1)

    tx2 = cylinder.location.x + cylinder.radius * np.cos(theta2)
    ty2 = cylinder.location.y + cylinder.radius * np.sin(theta2)

    return [(tx1, ty1), (tx2, ty2)]

def get_intersection(s1, v1, s2, v2): # s1 + a * v1 = s2 + b * v2 # all np.arrays
    A = np.transpose(np.array([v1, v2], float))
    B = s2 - s1
    x = np.linalg.solve(A, B)
    return s1 + x[0] * v1

def get_shorter_side_points(cylinder, light3d, tangent_points):
    # vectorize
    center = np.array([cylinder.location.x, cylinder.location.y], float)
    light = np.array([light3d[0], light3d[1]], float)
    tangent1 = np.array([tangent_points[0][0], tangent_points[0][1]], float)
    tangent2 = np.array([tangent_points[1][0], tangent_points[1][1]], float)

    center2light = light - center
    light2tangent1 = tangent1 - light
    light2tangent2 = tangent2 - light

    anchor = center + (vec_normalize(center2light) * cylinder.radius)
    ortho_center2light = get_ortho_vec(center2light)

    res1 = get_intersection(anchor, ortho_center2light, light, light2tangent1)
    res2 = get_intersection(anchor, ortho_center2light, light, light2tangent2)

    return [tuple(res1), tuple(res2)]

def get_longer_side_points(cylinder, light3d, tangent_points):
    # vectorize
    center = np.array([cylinder.location.x, cylinder.location.y], float)
    light = np.array([light3d[0], light3d[1]], float)
    tangent1 = np.array([tangent_points[0][0], tangent_points[0][1]], float)
    tangent2 = np.array([tangent_points[1][0], tangent_points[1][1]], float)

    light2center = center - light 
    light2tangent1 = tangent1 - light
    light2tangent2 = tangent2 - light

    anchor = center + (vec_normalize(light2center) * cylinder.radius)
    ortho_light2center = get_ortho_vec(light2center)

    shadow_point = get_shadow_point((anchor[0], anchor[1], min(cylinder.height, light3d[2]-1)), light3d)

    res1 = get_intersection(shadow_point, ortho_light2center, light, light2tangent1)
    res2 = get_intersection(shadow_point, ortho_light2center, light, light2tangent2)

    return [tuple(res1), tuple(res2)]

def convert_tuples2points(tuples):
    res = []
    for p in tuples:
        res.append(Point(x=p[0], y=p[1]))
    return res

def intersect_all(polygons):
    res = polygons[0]
    for p in polygons:
        res = res.intersection(p)
    return res

def union_all(polygons):
    res = polygons[0]
    for p in polygons:
        res = res.union(p)
    return res

class ShadowCalculator(object):
    def __init__(self):
        rospy.init_node('shadow_calculator')
        self.sub = rospy.Subscriber('/objects', Objects, self.objects_callback)
        self.penumbras_pub = rospy.Publisher('/penumbras', Penumbras)
        self.obstacles_pub = rospy.Publisher('/obstacles', Obstacles)
        # configuration
        #self.light_sources = CONFIG.projector_locations
        config_filename = rospy.get_param('config_file', CONFIG_FILENAME)
        self.read_in_config(config_filename)
        self.create_proj_shadows()
        
    def read_in_config(self, filename):
        f = open(filename)
        config_map = yaml.safe_load(f)
        f.close()
        self.light_sources = config_map.get('projector_locations')
        #print self.light_sources[1][1]
        self.proj_coord = config_map.get('projector_coordinates')
        self.sim_world_perim = config_map.get('sim_world_perimeter_projectable')
        
        
    def create_proj_shadows(self):
        world_poly = gm.Polygon(self.sim_world_perim)
        
        self.proj_display_shadow = []
        for coords in self.proj_coord:
            proj_poly = gm.Polygon(np.fliplr(coords))
            proj_shadow = world_poly.difference(proj_poly)
            self.proj_display_shadow.append(proj_shadow)
        

    def objects_callback(self, objects):
        obstacles = []
        penumbras = []
        polygon_objects_per_light_source = []

        for i, light_source in enumerate(self.light_sources):
            proj_id = light_source[0]
            light3d = light_source[1]
            polygon_objects = []
            for cylinder in objects.cylinders:
                rep1 = self.compute_shadow(cylinder, light3d) # just list of tuples (first = last)

                rep2 = gm.Polygon(rep1)
                rep3 = convert_tuples2points(rep1)
                
                polygon_objects.append(rep2)
                penumbras.append(Shadow(projector_id=proj_id, polygon=Polygon(points=rep3)))
            polygon_objects_per_light_source.append(polygon_objects)

        if len(objects.cylinders) > 0 and len(self.light_sources) > 0:
            polygon_objects = []
            for i in range(0, len(polygon_objects_per_light_source)):
                polygon_objects.append(union_all(polygon_objects_per_light_source[i]))
                
            intersections = intersect_all(polygon_objects) # returns either polygon or multipolygon
            
            if isinstance(intersections, gm.polygon.Polygon):
                obstacles.append(Polygon(points=convert_tuples2points(list(intersections.exterior.coords))))
            else:
                for p in list(intersections):
                    obstacles.append(Polygon(points=convert_tuples2points(list(p.exterior.coords))))

        self.obstacles_pub.publish(Obstacles(polygons=obstacles))
        self.penumbras_pub.publish(Penumbras(projector_shadows=penumbras))

    def compute_shadow(self, cylinder, light3d):
        # check if inside; quite ad-hoc...
        cx = cylinder.location.x
        cy = cylinder.location.y
        ch = cylinder.height
        r = cylinder.radius
        v1 = np.array([cx, cy], float)
        v2 = np.array([light3d[0], light3d[1]], float)
        if get_dist(v1, v2) <= cylinder.radius:
            sp1 = get_shadow_point((cx-r, cy+r, ch), light3d)
            sp2 = get_shadow_point((cx+r, cy+r, ch), light3d)
            sp3 = get_shadow_point((cx+r, cy-r, ch), light3d)
            sp4 = get_shadow_point((cx-r, cy-r, ch), light3d)
            return [sp1, sp2, sp3, sp4, sp1]

        tangent_points = get_tangent_points(cylinder, light3d)

        top = get_longer_side_points(cylinder, light3d, tangent_points)
        bot = get_shorter_side_points(cylinder, light3d, tangent_points)

        d1 = get_dist(np.array(top[0], float), np.array(bot[0], float))
        d2 = get_dist(np.array(top[0], float), np.array(bot[1], float))

        if d1 < d2:
            return [top[0], bot[0], bot[1], top[1], top[0]]
        else:
            return [top[0], bot[1], bot[0], top[1], top[0]]

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    shadow_calc = ShadowCalculator()
    shadow_calc.run()

