# balancebot chassis (OpenSCAD prototype)

`balancebot_chassis.scad` is a parametric three-deck frame. Sizes marked `measured` in the first
block of the file come from calipers on the real parts (2026-10-21). Sizes marked NOMINAL (wheel
width, battery holder, perfboard, rocker switch) still need measuring: change them there and
everything else follows.

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

## Measured values (2026-10-21)

| part | measured (mm) | used in the model as |
| --- | --- | --- |
| motor body and gearbox diameter | 24.84 | `motor_dia` (clamp bore is this + 0.1) |
| motor, back of encoder cylinder to gearbox face | 57.6 | `motor_len` |
| motor can plus gearbox, encoder board to gearbox face | 52.03 (gearbox 18.8) | not used directly |
| shaft | 3.99 dia, 9.47 long, 3.10 across the flat | not used (wheel hub fits) |
| gearbox face ring | 6.97 dia, 2.4 high | not used |
| gearbox mounting holes | 2.17 dia, 17.3 apart, one each side of the shaft | not used (threaded; clamp holds the motor) |
| wheel, with tire | 68.84 dia (7.4 tire width, hub sticks out 13.6) | `wheel_dia` |
| Nucleo | 82.44 x 69.85 x 18.7 | `nucleo` |
| GY-521 IMU | 20.23 x 15.7 x 11.28 | `imu` |
| TB6612 driver | 21.16 x 18.1 x 11.2 | `driver` |
| buck converter | 33.04 x 18.06 x 5.36 | no seat modelled yet |

Changes made because of the measurements: the first clamp on each motor now sits 16 mm from the
rear end (was 12) so it stays clear of the encoder board; the outrigger legs moved to +/- 47 mm
(was 50) so they stay inside the shorter rails; the stand is 2 mm taller so the wheels still hang
about 30 mm clear. Deck length is now 125.2 mm.

Still to measure: total wheel width (`wheel_w`, now 27), the battery holder (`battery`, now
80 x 40 x 20), perfboard, rocker switch cut-out.

## Checking the coupon (print `coupon.stl` plus ONE `cradle_cap.stl`)

1. Print quality: corners flat on the bed, no stringing in the holes.
2. M3 screw slides through each hole with light resistance; an M3 nut presses into its pocket
   with a finger and does not spin when the screw is tightened.
3. Motor in the lower half: it drops in with light finger pressure and does not rock.
4. Cap on, two M3 x 40 screws snug: a small gap (about 0.4 mm) remains between cap and block, the
   motor cannot turn or slide by hand, and the shaft still spins freely.
5. GY-521 sits on its four pillars inside the corner locators with no force and the header
   pins have room underneath.
6. Nucleo corner: the board corner slides into the L and sits flat.

If the motor is loose, reduce `clamp_gap` or `bore_d` (0.1 mm steps). If you cannot close the
screws or the motor will not turn, increase them.
