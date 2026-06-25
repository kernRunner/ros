import math

radius = 0.17
height = 0.06

z_top = height / 2.0
z_bottom = -height / 2.0

top = []
bottom = []

for i in range(6):
    angle = math.radians(60 * i)
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    top.append((x, y, z_top))
    bottom.append((x, y, z_bottom))

triangles = []

for i in range(1, 5):
    triangles.append((top[0], top[i], top[i + 1]))

for i in range(1, 5):
    triangles.append((bottom[0], bottom[i + 1], bottom[i]))

for i in range(6):
    j = (i + 1) % 6
    triangles.append((bottom[i], bottom[j], top[j]))
    triangles.append((bottom[i], top[j], top[i]))

double_sided_triangles = []

for a, b, c in triangles:
    double_sided_triangles.append((a, b, c))
    double_sided_triangles.append((c, b, a))

def normal(a, b, c):
    ux = b[0] - a[0]
    uy = b[1] - a[1]
    uz = b[2] - a[2]

    vx = c[0] - a[0]
    vy = c[1] - a[1]
    vz = c[2] - a[2]

    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx

    length = math.sqrt(nx * nx + ny * ny + nz * nz)

    if length == 0:
        return (0.0, 0.0, 0.0)

    return (nx / length, ny / length, nz / length)

with open("hex_chassis.stl", "w") as f:
    f.write("solid hex_chassis\n")

    for tri in double_sided_triangles:
        n = normal(*tri)

        f.write(f"  facet normal {n[0]} {n[1]} {n[2]}\n")
        f.write("    outer loop\n")

        for v in tri:
            f.write(f"      vertex {v[0]} {v[1]} {v[2]}\n")

        f.write("    endloop\n")
        f.write("  endfacet\n")

    f.write("endsolid hex_chassis\n")
