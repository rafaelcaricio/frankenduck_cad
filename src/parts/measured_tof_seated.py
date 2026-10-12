from __future__ import annotations
from cadgen import step


def make():
    from lib.cadquery_adapter import to_build123d
    """Measured bare ToF board layout from user's caliper photos, for the Frankenduck fit study.
    Measured: 24.81 mm long, 10.16 mm wide, 18.72 mm hole centres (confirmed by user), M3 holes,
    7.02 mm bare sensor length. Depth, chip width and its offset are provisional; cover/header not represented."""
    from lib.cadquery_adapter import Model
    pcb_length = 24.81
    pcb_width = 10.16
    hole_pitch = 18.72
    hole_radius = 1.6
    pcb_thickness = 1.0
    sensor_length = 7.02
    sensor_width = 3.3
    sensor_height = 1.6
    y_center = 30.252
    z_center = 298.805
    seat_x = 105.64
    ear_radius = (pcb_length - hole_pitch) / 2
    model = Model()
    model.box('pcb', pcb_thickness, hole_pitch, pcb_width, at=(seat_x - pcb_thickness / 2, y_center, z_center), name='Measured PCB outline')
    for i, yy in enumerate((y_center - hole_pitch / 2, y_center + hole_pitch / 2)):
        model.cylinder(f'ear_{i}', ear_radius, pcb_thickness, at=(seat_x - pcb_thickness, yy, z_center), direction=(1, 0, 0), op='join')
    for i, yy in enumerate((y_center - hole_pitch / 2, y_center + hole_pitch / 2)):
        model.cylinder(f'mount_hole_{i}', hole_radius, pcb_thickness, at=(seat_x - pcb_thickness, yy, z_center), direction=(1, 0, 0), op='cut')
    model.box('sensor', sensor_height, sensor_length, sensor_width, at=(seat_x + sensor_height / 2, y_center, z_center), op='new', name='Bare sensor, nominal depth')
    model.material('pla', color='#24875c', bodies=['pcb'])
    model.material('pla', color='#232329', bodies=['sensor'])
    result = model.result()
    return to_build123d(model, 'measured_tof')


@step(out="../../STEP/parts/measured_tof_seated.step")
def measured_tof_seated():
    return make()


if __name__ == "__main__":
    measured_tof_seated()
