from robot_autonomy_seb.map import bresenham


def test_bresenham():
    result = []

    def func(x, y):
        result.append((x, y))

    bresenham((1, 1), (8, 5), func)

    for e, r in zip([(1, 1), (2, 2), (3, 2), (4, 3), (5, 3), (6, 4), (7, 4), (8, 5)], result):
        assert e[0] == r[0] and e[1] == r[1]
