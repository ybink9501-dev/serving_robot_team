"""Observation-only cell-face visibility. Never authorizes robot motion."""
import math


def cell_face_visible(free, width, height, x, y, tx, ty):
    def trace(ex, ey):
        px, py = x+.5, y+.5
        dx, dy = ex-px, ey-py
        sx, sy = (1 if dx>0 else -1), (1 if dy>0 else -1)
        ax, ay = abs(1/dx) if dx else math.inf, abs(1/dy) if dy else math.inf
        mx = ((x+1-px)/dx if dx>0 else (x-px)/dx) if dx else math.inf
        my = ((y+1-py)/dy if dy>0 else (y-py)/dy) if dy else math.inf
        xx, yy = x, y
        for _ in range(width+height+3):
            if (xx,yy)==(tx,ty):return True
            if not (0<=xx<width and 0<=yy<height and free[yy*width+xx]):return False
            if abs(mx-my)<1e-12:
                # A grazing corner cannot justify seeing through an occupied cell.
                for cx,cy in ((xx+sx,yy),(xx,yy+sy)):
                    if not (0<=cx<width and 0<=cy<height and free[cy*width+cx]):return False
                xx+=sx;yy+=sy;mx+=ax;my+=ay
            elif mx<my:xx+=sx;mx+=ax
            else:yy+=sy;my+=ay
        return False
    return any(trace(tx+ox,ty+oy) for ox,oy in ((.5,.5),(.05,.05),(.05,.95),(.95,.05),(.95,.95)))
