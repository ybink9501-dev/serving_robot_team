"""Simple operator polygons, including concave rooms; no ROS or wall inference."""
import math
import numpy as np


def validate_polygon(polygon):
    if not isinstance(polygon,(list,tuple)) or not 3<=len(polygon)<=64:raise ValueError('Polygon requires 3..64 vertices')
    points=[tuple(map(float,p)) for p in polygon]
    if any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in points):raise ValueError('Invalid polygon coordinates')
    def cross(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    def on(a,b,p):return abs(cross(a,b,p))<=1e-10 and min(a[0],b[0])-1e-10<=p[0]<=max(a[0],b[0])+1e-10 and min(a[1],b[1])-1e-10<=p[1]<=max(a[1],b[1])+1e-10
    edges=list(zip(points,points[1:]+points[:1]))
    if any(a==b for a,b in edges):raise ValueError('Zero length polygon edge')
    for i,(a,b) in enumerate(edges):
        for j,(c,d) in enumerate(edges):
            if j<=i or j==i+1 or (i==0 and j==len(edges)-1):continue
            if (cross(a,b,c)*cross(a,b,d)<0 and cross(c,d,a)*cross(c,d,b)<0) or any((on(a,b,c),on(a,b,d),on(c,d,a),on(c,d,b))):
                raise ValueError('Polygon intersects or touches itself')
    if abs(sum(a[0]*b[1]-a[1]*b[0] for a,b in edges))<1e-10:raise ValueError('Polygon has no area')
    return points


def contains(points,x,y):
    inside=False
    for a,b in zip(points,points[1:]+points[:1]):
        cross=(b[0]-a[0])*(y-a[1])-(b[1]-a[1])*(x-a[0])
        if abs(cross)<=1e-10 and min(a[0],b[0])-1e-10<=x<=max(a[0],b[0])+1e-10 and min(a[1],b[1])-1e-10<=y<=max(a[1],b[1])+1e-10:return True
        if (a[1]>y)!=(b[1]>y) and x<a[0]+(y-a[1])*(b[0]-a[0])/(b[1]-a[1]):inside=not inside
    return inside


def polygon_mask(points,x,y):
    x,y=np.broadcast_arrays(x,y);inside=np.zeros(x.shape,dtype=bool);boundary=inside.copy()
    for a,b in zip(points,points[1:]+points[:1]):
        cross=(b[0]-a[0])*(y-a[1])-(b[1]-a[1])*(x-a[0])
        boundary |= (abs(cross)<=1e-10)&(x>=min(a[0],b[0])-1e-10)&(x<=max(a[0],b[0])+1e-10)&(y>=min(a[1],b[1])-1e-10)&(y<=max(a[1],b[1])+1e-10)
        if b[1]!=a[1]:inside ^= ((a[1]>y)!=(b[1]>y)) & (x<a[0]+(y-a[1])*(b[0]-a[0])/(b[1]-a[1]))
    return inside|boundary
