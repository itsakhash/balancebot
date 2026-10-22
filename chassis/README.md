# balancebot chassis (OpenSCAD prototype)

`balancebot_chassis.scad` is a parametric three-deck frame. Sizes marked `measured` in the first
block of the file come from calipers on the real parts (2026-10-21). Sizes marked NOMINAL (wheel
width, breadboard, perfboard) still need measuring: change them there and everything else follows.

View or export a part: set `part = "...";` near the bottom of the file (or pass
`-D 'part="rail_back"'` on the command line), press F5 to preview, F6 to render, then File > Export > STL.

## What goes where

| deck | carries |
| --- | --- |
| base deck (z = -20.5) | the two motor clamps only |
| mid deck (z = 70) | the Nucleo and a half-size breadboard; the IMU and the TB6612 driver plug into the breadboard (their header pins point down 11.3 mm, so they cannot sit on the deck) |
| top deck (z = 140) | the 2S1P battery holder and the perfboard (buck converter, power wiring) |
| back rail | round 20 mm rocker switch and two zip-tie slots for the fuse holder |

The mid and top decks are longer than the rails (156 and 134 mm against 125.2 mm). They sit well above the wheels, so the overhang is clear.

| part | what | qty | print orientation in `stl/` |
| --- | --- | --- | --- |
| coupon | fit test: motor clamp half, breadboard corner, Nucleo corner, nut pocket (underside) | 1 | flat |
| base_deck | deck 1: four clamp blocks | 1 | flat |
| mid_deck | deck 2: Nucleo seat, breadboard seat, wire slot between them | 1 | flat |
| top_deck | deck 3: battery seat, perfboard seat, strap slots | 1 | flat |
| rail_front, rail_back | side rails (back has the switch hole) | 1 each | lying flat |
| cradle_cap | motor clamp cap | 4 | split face up |
| leg | outrigger leg, limits tilt to about 20 degrees | 4 | standing, layers follow the L |
| stand | test stand, wheels hang 30 mm clear | 1 | flat |

Hardware: clamp screws M3 x 40 with nuts (x8), leg screws M3 x 12 with nuts (x8), zip ties and a
hook-and-loop strap for the battery.

Checks run in OpenSCAD: `interference` (printed parts against the stand-ins for the motors, wheels,
Nucleo, breadboard, battery, perfboard and switch body), `interference_parts` (printed parts against
each other) and `interference_legs_stand` all report no overlapping volume beyond faces that only touch.

## Measured values (2026-10-21) and listing values

| part | size (mm) | used in the model as |
| --- | --- | --- |
| motor body and gearbox diameter | 24.84 (measured) | `motor_dia` (clamp bore is this + 0.1) |
| motor, back of encoder cylinder to gearbox face | 57.6 (measured) | `motor_len` |
| wheel, with tire | 68.84 dia (measured; 7.4 tire width, hub sticks out 13.6) | `wheel_dia` |
| Nucleo | 82.44 x 69.85 x 18.7 (measured) | `nucleo` |
| GY-521 IMU | 20.23 x 15.7 x 11.28 (measured); header pins point down 11.3 | `imu` (plugs into the breadboard) |
| TB6612 driver | 21.16 x 18.1 x 11.2 (measured); header pins point down | `driver` (plugs into the breadboard) |
| buck converter (Mini560 Pro 5 V) | 33.04 x 18.06 x 5.36 (measured) | no seat modelled yet |
| battery holder, 2S1P with protection | 80 x 42 x 18.6 (from the listing; measure on arrival) | `battery` |
| rocker switch, round | 20 dia, 25 high, 23 long, 5 sets of #187 terminal wires (from the listing) | `switch_d` hole 20.2 |
| breadboard | 82 x 54 x 9 (from the listing drawing): measure to confirm | `breadboard` |

Changes in this version:

- The IMU and driver moved off the base deck onto a breadboard on the mid deck. The IMU is now about 80 mm above the axle instead of on the axle line; the gyro is unaffected and the filter should lean on it, but expect a little more accelerometer noise from the lever arm.
- The mid deck is 156 mm long: Nucleo on one end, breadboard on the other, a 60 x 12 mm slot between them for the motor and encoder wires.
- Rail height is 4 mm taller (196.5 mm) and the back rail has a round 20.2 mm switch hole, with a 23.5 mm recess on the inner face that leaves a 1.8 mm web for the switch's snap tabs.
- The battery seat moved to y = 35 mm so the switch body clears it.
- The nut pocket is 6.8 mm across corners and 2.8 mm deep (was 6.4 and 2.6, only 0.05 mm clear of an M3 nut). It is on the underside of the part.

Still to measure: total wheel width (`wheel_w`, now 27), the breadboard, the battery holder, the
perfboard, the fuse holder body.

## Checking the coupon (print `coupon.stl` plus ONE `cradle_cap.stl`)

1. Print quality: corners flat on the bed, no stringing in the holes.
2. M3 screw slides through each hole with light resistance. Turn the coupon over: an M3 nut presses
   into its hexagonal pocket with a finger and does not spin when the screw is tightened.
3. Motor in the lower half: it drops in with light finger pressure and does not rock.
4. Cap on, two M3 x 40 screws snug: a small gap (about 0.4 mm) remains between cap and block, the
   motor cannot turn or slide by hand, and the shaft still spins freely.
5. Breadboard corner: a corner of the breadboard slides into the L and sits flat.
6. Nucleo corner: the board corner slides into the L and sits flat.

If the motor is loose, reduce `clamp_gap` or `bore_d` (0.1 mm steps). If you cannot close the
screws or the motor will not turn, increase them.
