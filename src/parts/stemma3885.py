from __future__ import annotations
from cadgen import step


def make():
    from lib.cadquery_adapter import to_build123d
    """Adafruit STEMMA Speaker 3885 mechanical envelope, mm.
    Manufacturer PCB: https://github.com/adafruit/Adafruit-STEMMA-Speaker-PCB
    Designed by Limor Fried/Ladyada for Adafruit Industries, CC BY-SA; support open hardware.
    PCB outline and 2.2 mm drills from Eagle board. Assembly height 7.25 mm from user caliper.
    No matching STEP catalog part. Speaker/connector and underside magnet heights are simplified
    keep-out envelopes, not measured subcomponent geometry; PCB thickness 1.6 mm provisional.
    Origin at PCB lower-left underside; local +Z toward speaker face.
    """
    import cadquery as cq
    from lib.cadquery_adapter import Model
    board_w = 30.48
    board_l = 36.88
    board_t = 1.6
    hole_d = 2.2
    corner_r = 2.54
    speaker_top = 5.75
    back_depth = 1.5
    model = Model()
    pcb = cq.Workplane('XY').rect(board_w, board_l).extrude(board_t).edges('|Z').fillet(corner_r).translate((board_w / 2, board_l / 2, 0))
    for x in (2.54, 27.94):
        for y in (2.54, 34.29):
            pcb = pcb.cut(cq.Workplane('XY').center(x, y).circle(hole_d / 2).extrude(board_t))
    model.add('pcb', pcb, body='pcb')
    speaker = cq.Workplane('XY', origin=(15.24, 21.59, board_t)).slot2D(30, 20).extrude(speaker_top - board_t)
    model.add('speaker', speaker, body='speaker', op='new')
    model.box('magnet', 20, 14, back_depth, at=(15.24, 21.59, -back_depth / 2), body='magnet', op='new')
    model.box('stemma_jst', 10, 7.5, 4.15, at=(15.24, 4.322, board_t + 2.075), body='stemma_jst', op='new')
    model.box('speaker_jst', 4.7, 3.5, 3, at=(4.699, 8.255, board_t + 1.5), body='speaker_jst', op='new')
    model.cylinder('volume', 1.5, 2, at=(22.987, 3.048, board_t), body='volume', op='new')
    model.material('abs', bodies=['pcb', 'speaker', 'magnet'], color='#202329')
    model.material('abs', bodies=['stemma_jst', 'speaker_jst'], color='#eee8da')
    result = model.result()
    return to_build123d(model, 'stemma3885')


@step(out="../../STEP/parts/stemma3885.step")
def stemma3885():
    return make()


if __name__ == "__main__":
    stemma3885()
