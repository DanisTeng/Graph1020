from two_path_graph import *
from matplotlib import pyplot as plt

def plot_convex_polygon(polygon: ConvexPolygon, color:str):
    x = [v.x for v in polygon.vertices]
    y = [v.y for v in polygon.vertices]
    x.append(polygon.vertices[0].x)
    y.append(polygon.vertices[0].y)
    plt.plot(x,y, color = color)



def test_sum_vehicle_box_with_speed_cone():
    #
    v1 = Vec2(-2.0, 2.0)
    v2 = Vec2(-2.0, 2.0)
    v3 = Vec2(-1.0, -1.0)
    v4 = Vec2(-1.0, -1.0)

    box = Box2(Vec2(-1.0,0.0),2.0,2.0,Vec2(1.0,0.0),Vec2(0.0,1.0))

    plot_convex_polygon(sum_vehicle_box_with_speed_cone(box,[v1,v2,v3,v4]),'b')
    plt.show()


def test_regulation_by_table():

    r1 = RegulationByTable(0, 10)
    r2 = RegulationByTable(0, 10)

    for i in range(5):
        r1.forbid(((0,0,0),(0,0,0)), i)
        r2.forbid(((0,0,0),(0,0,0)), i+4)

    r1.union_with(r2)
    assert not r1.can_do(((0,0,0),(0,0,0)), 4)
    for i in range(11):
        if i ==4:
            continue
        assert r1.can_do(((0, 0, 0), (0, 0, 0)), i)


if __name__ == "__main__":
    test_sum_vehicle_box_with_speed_cone()
    test_regulation_by_table()
