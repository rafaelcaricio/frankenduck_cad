from cadgen import step

def make():
    from lib.bearing_revision.adapter_bridge_v3 import vendor_parts
    from lib.bearing_revision.collision_bridge_v2 import place
    from cadgen import build123d as bd
    horn=next(p for p in vendor_parts() if '从动' in p.label)
    pose=((17.6,-35.1,235.769),((.5773502691896257,-.5773502691896257,-.5773502691896258),119.99999999999999))
    s=place(horn,pose).rotate(bd.Axis.Z,90)
    s.color='#bfc3c7'
    return s

@step(out="../../STEP/parts/import_horn_upper_passive.step")
def import_horn_upper_passive():
    return make()

if __name__ == "__main__":
    import_horn_upper_passive()
