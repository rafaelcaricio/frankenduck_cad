from __future__ import annotations
from cadgen import step


def make():
    from lib.steve_compat import to_build123d
    import cadquery as cq
    from lib.steve_compat import Model
    x0 = -48.6
    x1 = -37.45
    half_y = 25.1
    z_body = 153.17
    z_top = 158.0
    wing_x0 = -48.0
    wing_x1 = -38.6
    wing_y1 = 34.5
    wing_z = 155.0
    outer_r = 1.5
    ear_pad_x0 = -44.8
    ear_pad_x1 = -39.55
    ear_pad_y0 = 23.0
    ear_pad_y1 = 25.0
    ear_pad_z = 152.0
    foot_x0 = -44.5
    foot_x1 = -39.55
    foot_y0 = 6.0
    foot_y1 = 21.5
    foot_z = 149.0
    window_x0 = -45.6
    window_y = 17.5
    window_z0 = 147.8
    screw_x = -43.3
    screw_y = 31.0
    screw_r = 1.7
    SIDES = (1, -1)
    
    def box(xa, xb, ya, yb, za, zb):
        return cq.Solid.makeBox(xb - xa, yb - ya, zb - za, cq.Vector(xa, ya, za))
    
    def span(s, a, b):
        return (min(s * a, s * b), max(s * a, s * b))
    
    def wing(s):
        w = cq.Workplane().add(box(wing_x0, wing_x1, *span(s, half_y, wing_y1), wing_z, z_top))
        return w.edges('|Z').edges('>Y' if s > 0 else '<Y').fillet(outer_r).val()
    model = Model()
    model.add('body', box(x0, x1, -half_y, half_y, z_body, z_top))
    for s in SIDES:
        n = 'p' if s > 0 else 'n'
        model.add(f'wing_{n}', wing(s), op='join')
        model.add(f'ear_pad_{n}', box(ear_pad_x0, ear_pad_x1, *span(s, ear_pad_y0, ear_pad_y1), ear_pad_z, z_body + 0.5), op='join')
        model.add(f'pcb_foot_{n}', box(foot_x0, foot_x1, *span(s, foot_y0, foot_y1), foot_z, z_body + 0.5), op='join')
    model.add('plug_window', box(window_x0, x1 + 1, -window_y, window_y, window_z0, z_top + 1), op='cut')
    for s in SIDES:
        model.cylinder(f"screw_hole_{('p' if s > 0 else 'n')}", radius=screw_r, height=z_top - wing_z + 2, at=(screw_x, s * screw_y, wing_z - 1), op='cut')
    model.material('pla', color='#1d1d1f')
    result = model.result()
    return to_build123d(model, 'battery_donor_cover_native')


@step(out="../../STEP/parts/battery_donor_cover_native.step")
def battery_donor_cover_native():
    return make()


if __name__ == "__main__":
    battery_donor_cover_native()
