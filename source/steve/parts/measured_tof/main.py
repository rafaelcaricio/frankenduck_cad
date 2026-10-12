"""Measured bare ToF board layout from user's caliper photos, for the Frankenduck fit study.
Measured: 24.81 mm long, 10.16 mm wide, 18.72 mm hole centres (confirmed by user), M3 holes,
7.02 mm bare sensor length. Depth, chip width and its offset are provisional; cover/header not represented."""
from steve import Model
pcb_length = 24.81  # mm Measured full PCB length
pcb_width = 10.16  # mm Measured PCB width
hole_pitch = 18.72  # mm User-confirmed centre-to-centre measurement
hole_radius = 1.6  # mm Nominal M3 clearance representation, actual bore diameter not measured
pcb_thickness = 1.0  # mm PROVISIONAL, retained from previous mechanical model
sensor_length = 7.02  # mm Measured bare package length
sensor_width = 3.3  # mm PROVISIONAL visual width
sensor_height = 1.6  # mm PROVISIONAL, retained previous nominal total 2.6 mm
y_center = 30.252  # mm Existing front opening axis, robot world frame
z_center = 298.805  # mm Existing screw/opening row, sensor offset unmeasured
seat_x = 104.085  # mm Existing face boss seating plane
ear_radius = (pcb_length-hole_pitch)/2  # mm Ends preserve measured length and pitch
model = Model()
model.box("pcb", pcb_thickness, hole_pitch, pcb_width,
          at=(seat_x-pcb_thickness/2,y_center,z_center),name="Measured PCB outline")
for i,yy in enumerate((y_center-hole_pitch/2,y_center+hole_pitch/2)):
    model.cylinder(f"ear_{i}",ear_radius,pcb_thickness,
                   at=(seat_x-pcb_thickness,yy,z_center),direction=(1,0,0),op="join")
for i,yy in enumerate((y_center-hole_pitch/2,y_center+hole_pitch/2)):
    model.cylinder(f"mount_hole_{i}",hole_radius,pcb_thickness,
                   at=(seat_x-pcb_thickness,yy,z_center),direction=(1,0,0),op="cut")
model.box("sensor",sensor_height,sensor_length,sensor_width,
          at=(seat_x+sensor_height/2,y_center,z_center),op="new",name="Bare sensor, nominal depth")
model.material("pla",color="#24875c",bodies=["pcb"])
model.material("pla",color="#232329",bodies=["sensor"])
result=model.result()

