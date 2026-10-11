# balancebot chassis (OpenSCAD prototype)

`balancebot_chassis.scad` is a parametric three-deck frame. All sizes in the first block of the
file are NOMINAL: measure the real parts and change them there.

View or export a part: set `part = "...";` near the bottom of the file (or pass
`-D 'part="rail_back"'` on the command line), press F5 to preview, F6 to render, then File > Export > STL.

| part | what | qty | print orientation in `stl/` |
| --- | --- | --- | --- |
| coupon | fit test: motor clamp half, IMU seat, Nucleo corner, nut pocket | 1 | flat |
| base_deck | deck 1: four clamp blocks, IMU seat, driver seat | 1 | flat |
| mid_deck | deck 2: Nucleo seat | 1 | flat |
| top_deck | deck 3: battery seat, perfboard seat, strap slots | 1 | flat |
| rail_front, rail_back | side rails (back has the switch cut-out) | 1 each | lying flat |
| cradle_cap | motor clamp cap | 4 | split face up |
| leg | outrigger leg, limits tilt to about 20 degrees | 4 | standing, layers follow the L |
| stand | test stand, wheels hang 30 mm clear | 1 | flat |

Hardware: clamp screws M3 x 40 with nuts (x8), leg screws M3 x 12 with nuts (x8), zip ties and a
hook-and-loop strap for the battery.

Checks run in OpenSCAD: `interference` (printed parts against the stand-ins for the motors, wheels,
Nucleo, battery, perfboard, IMU, driver), `interference_parts` (printed parts against each other) and
`interference_legs_stand` all report no overlapping volume. Faces that only touch are expected.
