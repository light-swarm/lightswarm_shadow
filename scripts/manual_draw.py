from matplotlib import pyplot
from shapely.geometry import Polygon
from descartes.patch import PolygonPatch

COLOR = {
    True:  '#6699cc',
    False: '#ff3333'
}

def v_color(ob):
    return COLOR[ob.is_valid]

def plot_coords(ax, ob):
    x, y = ob.xy
    ax.plot(x, y, 'o', color='#999999', zorder=1)
    
fig = pyplot.figure(1, figsize=(10, 10), dpi=90)

# 1: valid polygon
ax = fig.add_subplot(111)

ext1 = [(-42, -71), (-15, 20), (15, 20), (42, -70)]
ext2 = [(-100, -60), (-1, 25), (24, 7), (-24, -114)]
ext3 = [(-42, -71), (-17.5, 11.3), (-7.5, 20), (6.4,20), (17,12), (21,-1), (-7,-70)]
polygon1 = Polygon(ext1)
polygon2 = Polygon(ext2)
polygon3 = Polygon(ext3)

plot_coords(ax, polygon1.exterior)
plot_coords(ax, polygon2.exterior)
plot_coords(ax, polygon3.exterior)

patch1 = PolygonPatch(polygon1, facecolor=v_color(polygon1), edgecolor=v_color(polygon1), alpha=0.5, zorder=2)
patch2 = PolygonPatch(polygon2, facecolor=v_color(polygon2), edgecolor=v_color(polygon2), alpha=0.5, zorder=2)
patch3 = PolygonPatch(polygon3, facecolor=v_color(polygon3), edgecolor=v_color(polygon3), alpha=0.5, zorder=2)

ax.add_patch(patch1)
ax.add_patch(patch2)
ax.add_patch(patch3)

xrange = [-200, 200]
yrange = [-200, 200]
ax.set_xlim(*xrange)
ax.set_xticks(range(xrange[0], xrange[1], 50) + [xrange[-1]])
ax.set_ylim(*yrange)
ax.set_yticks(range(yrange[0], yrange[1], 50) + [yrange[-1]])
ax.set_aspect(1)

pyplot.show()

