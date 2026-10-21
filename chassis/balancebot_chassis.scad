// balancebot chassis prototype (OpenSCAD)
//
// Coordinates in the assembly: X = forward/back, Y = along the wheel axle,
// Z = up, origin = centre of the axle. Units are mm.
//
// Pick what to show or export with:  openscad -D 'part="rail_back"' -o rail_back.stl balancebot_chassis.scad
//   part = assembly | exploded | coupon | base_deck | mid_deck | top_deck | rail_front | rail_back
//          | cradle_cap | leg | stand | interference
//
// Numbers marked "measured" in the first block come from calipers on the real parts
// (2026-10-21). Numbers marked NOMINAL (wheel width, battery holder, perfboard, switch)
// are still to be measured. Change them here; everything else follows.

$fn = 64;

// ---- sizes of the purchased parts (NOMINAL: measure and update) -----------
motor_dia   = 24.84;   // measured: JGA25-370 body and gearbox diameter
motor_len   = 57.6;    // measured: back of the encoder cylinder to the gearbox face (shaft, ring and hub not included)
motor_gap   = 10;      // space between the two motors' inner ends
wheel_dia   = 68.84;   // measured: outer diameter including the tire
wheel_w     = 27;      // NOMINAL wheel width (measure)
wheel_clear = 3;       // gap between motor end and wheel
nucleo      = [82.44, 69.85];   // measured: X (long side), Y
imu         = [20.23, 15.7]; // measured GY-521 footprint; rotate with imu_rot if its Y arrow is on the long side
imu_rot     = 0;            // 0 or 90
driver      = [21.16, 18.1]; // measured TB6612 module
perfboard   = [70, 50];
battery     = [80, 40, 20]; // 2S 18650 holder with protection board
switch_cut  = [13, 19];     // rocker switch panel cut-out (Y, Z): match your switch
// ---- printing tolerances ---------------------------------------------------
hole_clear   = 0.2;
pocket_clear = 0.3;
// ---- frame -----------------------------------------------------------------
deck_t      = 4;
rail_t      = 4;
deck_x      = 96;                    // inner distance between the two rails
deck_y      = 2*(motor_gap/2 + motor_len);   // 125.2 with the measured motor_len
deck_z      = [-20.5, 70, 140];      // bottom of each deck, above the axle
tab_w       = 30;                    // deck tabs that go through the rail slots
tab_y       = 35;
rail_bot    = deck_z[0] - 6;
rail_top    = deck_z[2] + deck_t + 22;
win_pad     = 3;                     // wiring-window margin above/below each deck
tilt_limit  = 20;                    // outrigger legs touch the floor at this tilt
leg_reach   = 40;                    // how far the legs stick out of the rails
leg_w       = 30;
leg_h       = 28;
leg_y       = 47;                    // leg centres at +/- leg_y along the rail
// ---- derived ---------------------------------------------------------------
motor_r     = motor_dia/2;
clamp_gap   = 0.4;           // the cap stops this far above the block, so tightening the screws squeezes the motor
bore_d      = motor_dia + 0.1;
screw_x     = motor_r + 0.15 + 4.5;      // clamp screw offset from motor axis
cradle_x    = screw_x + 4.5;             // half width of a cradle block
cradle_w    = 10;
cap_top     = motor_r + 4;
cradle_y    = [for (s=[-1,1]) each [s*(motor_gap/2+16), s*(motor_gap/2+48)]];   // first clamp sits 16 mm from the rear, clear of the encoder board
wheel_y     = motor_gap/2 + motor_len + wheel_clear + wheel_w/2;
rail_x      = deck_x/2;                  // inner face of the rails
leg_x0      = rail_x + rail_t;
leg_z       = (leg_x0 + leg_reach) * tan(tilt_limit) - wheel_dia/2;   // arm underside
stand_h     = 38;
stand_floor = rail_bot - stand_h;

module hexpocket(h=2.6) { cylinder(d=6.4, h=h, $fn=6); }

// one L-shaped corner locator for a board of size sz centred at the origin (sx, sy = which corner)
module one_corner(sz, sx, sy, leg=8, t=2, h=3, c=pocket_clear) {
  a = sz[0]/2 + c;  b = sz[1]/2 + c;
  translate([sx>0 ? a-leg : -a-t, sy>0 ? b : -b-t, 0]) cube([leg+t, t, h]);
  translate([sx>0 ? a : -a-t, sy>0 ? b-leg : -b-t, 0]) cube([t, leg+t, h]);
}
// four corner locators around a board
module corner_locators(sz, leg=8, t=2, h=3, c=pocket_clear) {
  for (sx=[-1,1], sy=[-1,1]) one_corner(sz, sx, sy, leg, t, h, c);
}
// pillars under the corners of a board
module pillars(sz, h=5, w=5) {
  for (sx=[-1,1], sy=[-1,1])
    translate([sx*(sz[0]/2 - w/2 - 1) - w/2, sy*(sz[1]/2 - w/2 - 1) - w/2, 0]) cube([w, w, h]);
}
// zip-tie slots through a deck, just outside the long sides of a board
module tie_slots(sz, z0) {
  for (sx=[-1,1], sy=[-1,1])
    translate([sx*sz[0]/4 - 4, sy*(sz[1]/2 + 7) - 1.3, z0 - 1]) cube([8, 2.6, deck_t + 2]);
}

// ---- decks -----------------------------------------------------------------
module deck_blank(z0) {
  translate([0, 0, z0]) {
    translate([-deck_x/2, -deck_y/2, 0]) cube([deck_x, deck_y, deck_t]);
    for (sx=[-1,1], ty=[-tab_y, tab_y])
      translate([sx>0 ? deck_x/2 - 1 : -deck_x/2 - rail_t, ty - tab_w/2, 0]) cube([rail_t + 1, tab_w, deck_t]);
  }
}

// lower half of a motor clamp: from the plate top up to the split plane z = 0
module cradle_lower(yc) {
  translate([0, yc, 0]) difference() {
    translate([-cradle_x, -cradle_w/2, deck_z[0] + deck_t - 0.01]) cube([2*cradle_x, cradle_w, -(deck_z[0] + deck_t) + 0.01]);
    rotate([90, 0, 0]) cylinder(d=bore_d, h=cradle_w + 2, center=true);
  }
}

module base_deck() {
  difference() {
    union() {
      deck_blank(deck_z[0]);
      for (yc = cradle_y) cradle_lower(yc);
      // IMU on pillars, centred on the axle line, in front of the motors
      translate([34, 0, deck_z[0] + deck_t]) rotate([0, 0, imu_rot]) {
        pillars(imu, h=6, w=4);
        corner_locators(imu, leg=5, t=1.6, h=8);
      }
      // motor driver behind the motors
      translate([-34, 0, deck_z[0] + deck_t]) {
        pillars(driver, h=3, w=4);
        corner_locators(driver, leg=5, t=1.6, h=6);
      }
    }
    // clamp screws through the plate, nut pockets in the underside
    for (yc = cradle_y, sx=[-1,1]) {
      translate([sx*screw_x, yc, deck_z[0] - 1]) cylinder(d=3 + 2*hole_clear, h=40);
      translate([sx*screw_x, yc, deck_z[0] - 0.01]) hexpocket(2.6);
    }
    // zip-tie slots beside the IMU and the driver module
    translate([34, 0, 0]) rotate([0, 0, imu_rot]) tie_slots(imu, deck_z[0]);
    translate([-34, 0, 0]) tie_slots(driver, deck_z[0]);
  }
}

module mid_deck() {
  difference() {
    union() {
      deck_blank(deck_z[1]);
      translate([0, 0, deck_z[1] + deck_t]) {
        pillars(nucleo, h=5, w=6);
        corner_locators(nucleo, leg=10, t=2, h=7);
      }
    }
    tie_slots(nucleo, deck_z[1]);
  }
}

module top_deck() {
  by = 30;    // battery centre Y
  py = -37;   // perfboard centre Y
  difference() {
    union() {
      deck_blank(deck_z[2]);
      translate([0, by, deck_z[2] + deck_t]) corner_locators([battery[0], battery[1]], leg=12, t=2.4, h=10);
      translate([0, py, deck_z[2] + deck_t]) {
        pillars(perfboard, h=4, w=5);
        corner_locators(perfboard, leg=8, t=2, h=6);
      }
    }
    // two strap slots either side of the battery, at two X positions
    for (sx=[-22, 22], sy=[-1, 1])
      translate([sx - 5, by + sy*(battery[1]/2 + 5) - 1.3, deck_z[2] - 1]) cube([10, 2.6, deck_t + 2]);
    translate([0, py, 0]) tie_slots(perfboard, deck_z[2]);
  }
}

// ---- rails -----------------------------------------------------------------
module rail(back=false) {
  difference() {
    translate([0, -deck_y/2, rail_bot]) cube([rail_t, deck_y, rail_top - rail_bot]);
    // deck tab slots
    for (i=[0:2], ty=[-tab_y, tab_y])
      translate([-1, ty - tab_w/2 - 0.2, deck_z[i] - 0.2]) cube([rail_t + 2, tab_w + 0.4, deck_t + 0.4]);
    // wiring windows between the decks
    for (i=[0:1])
      translate([-1, -35, deck_z[i] + deck_t + win_pad]) cube([rail_t + 2, 70, deck_z[i+1] - deck_z[i] - deck_t - 2*win_pad]);
    // M3 holes for the outrigger legs
    for (ys=[-leg_y, leg_y], dz=[8, 20])
      translate([-1, ys, leg_z + dz]) rotate([0, 90, 0]) cylinder(d=3 + 2*hole_clear, h=rail_t + 2);
    // rocker switch cut-out and a fuse-holder tie slot, back rail only
    if (back) {
      translate([-1, 40 - switch_cut[0]/2, deck_z[2] + deck_t + 11 - switch_cut[1]/2]) cube([rail_t + 2, switch_cut[0], switch_cut[1]]);
      translate([-1, -50 - 1.3, deck_z[2] + deck_t + 11 - 4]) cube([rail_t + 2, 2.6, 8]);
      translate([-1, -40 - 1.3, deck_z[2] + deck_t + 11 - 4]) cube([rail_t + 2, 2.6, 8]);
    }
  }
}

// ---- cradle cap, leg, stand ------------------------------------------------
module cradle_cap() {   // modelled in place: yc = 0; its flat face sits clamp_gap above the split plane
  difference() {
    translate([-cradle_x, -cradle_w/2, clamp_gap]) cube([2*cradle_x, cradle_w, cap_top - clamp_gap]);
    rotate([90, 0, 0]) cylinder(d=bore_d, h=cradle_w + 2, center=true);
    for (sx=[-1,1]) translate([sx*screw_x, 0, -1]) cylinder(d=3 + 2*hole_clear, h=cap_top + 2);
  }
}

module leg() {   // local origin: floor of the arm, against the rail face; y 0..leg_w
  difference() {
    union() {
      hull() {                                        // arm with a rounded tip
        cube([leg_reach - leg_w/2, leg_w, 4]);
        translate([leg_reach - leg_w/2, leg_w/2, 0]) cylinder(d=leg_w, h=4);
      }
      cube([4, leg_w, leg_h]);                        // flange against the rail
      // gussets
      for (y=[0, leg_w - 3]) translate([0, y, 0]) rotate([90, 0, 0]) translate([0, 0, -3])
        linear_extrude(3) polygon([[0,0],[leg_reach*0.7,0],[0,leg_h*0.7]]);
    }
    for (dz=[8, 20]) translate([-1, leg_w/2, dz]) rotate([0, 90, 0]) cylinder(d=3 + 2*hole_clear, h=6);
  }
}

module stand() {
  difference() {
    union() {
      for (sx=[-1,1]) translate([sx*(rail_x + rail_t/2) - 7, -deck_y/2, stand_floor]) cube([14, deck_y, stand_h + 1]);
      for (sy=[-1,1]) translate([-(rail_x + rail_t/2) - 7, sy*(deck_y/2 - 6) - 6, stand_floor]) cube([2*(rail_x + rail_t/2) + 14, 12, 8]);
    }
    // seat for the rail bottoms (a 1 mm deep channel so the robot cannot slide); bar top is 1 mm above the rail bottom
    for (sx=[-1,1]) translate([sx*(rail_x + rail_t/2) - (rail_t + 0.6)/2, -deck_y/2 - 1, rail_bot]) cube([rail_t + 0.6, deck_y + 2, 2]);
  }
}

// ---- stand-ins for the bought parts (assembly view only) -------------------
// e shrinks every stand-in by e mm per side so that parts that only touch are not reported as overlapping
module ghosts(e=0) {
  color("dimgray", 0.9) for (s=[-1,1]) translate([0, s*(motor_gap/2 + motor_len/2), 0]) rotate([90, 0, 0]) cylinder(d=motor_dia - 2*e, h=motor_len - 2*e, center=true);
  color("black", 0.9) for (s=[-1,1]) translate([0, s*wheel_y, 0]) rotate([90, 0, 0]) cylinder(d=wheel_dia - 2*e, h=wheel_w - 2*e, center=true);
  color("seagreen", 0.9) translate([-nucleo[0]/2 + e, -nucleo[1]/2 + e, deck_z[1] + deck_t + 5 + e]) cube([nucleo[0] - 2*e, nucleo[1] - 2*e, 1.6 - 2*e]);
  color("royalblue", 0.9) translate([-battery[0]/2 + e, 30 - battery[1]/2 + e, deck_z[2] + deck_t + e]) cube([battery[0] - 2*e, battery[1] - 2*e, battery[2] - 2*e]);
  color("goldenrod", 0.9) translate([-perfboard[0]/2 + e, -37 - perfboard[1]/2 + e, deck_z[2] + deck_t + 4 + e]) cube([perfboard[0] - 2*e, perfboard[1] - 2*e, 1.6 - 2*e]);
  color("purple", 0.9) translate([34, 0, deck_z[0] + deck_t + 6 + e]) rotate([0,0,imu_rot]) translate([-imu[0]/2 + e, -imu[1]/2 + e, 0]) cube([imu[0] - 2*e, imu[1] - 2*e, 1.6 - 2*e]);
  color("red", 0.9) translate([-34, 0, deck_z[0] + deck_t + 3 + e]) translate([-driver[0]/2 + e, -driver[1]/2 + e, 0]) cube([driver[0] - 2*e, driver[1] - 2*e, 1.6 - 2*e]);
}

module printed_parts(explode=0) {
  color("lightsteelblue") translate([0, 0, -explode]) base_deck();
  color("lightsteelblue") mid_deck();
  color("lightsteelblue") translate([0, 0, explode]) top_deck();
  color("khaki") translate([rail_x, 0, 0]) translate([explode, 0, 0]) rail(false);
  color("khaki") translate([-rail_x, 0, 0]) translate([-explode, 0, 0]) mirror([1, 0, 0]) rail(true);
  color("orange") for (yc = cradle_y) translate([0, yc, explode*0.6]) cradle_cap();
  color("tomato") for (sx=[-1,1], sy=[-leg_y, leg_y]) translate([sx > 0 ? leg_x0 : -leg_x0, sy - leg_w/2, leg_z]) mirror([sx > 0 ? 0 : 1, 0, 0]) leg();
}

// ---- part selector ---------------------------------------------------------
part = "assembly";

if (part == "assembly") { printed_parts(); ghosts(); }
else if (part == "exploded") { printed_parts(explode=25); ghosts(); }
else if (part == "base_deck") translate([0, 0, -deck_z[0]]) base_deck();
else if (part == "mid_deck") translate([0, 0, -deck_z[1]]) mid_deck();
else if (part == "top_deck") translate([0, 0, -deck_z[2]]) top_deck();
else if (part == "rail_front") rotate([0, -90, 0]) rail(false);
else if (part == "rail_back") rotate([0, -90, 0]) rail(true);
else if (part == "cradle_cap") translate([0, 0, cap_top]) rotate([180, 0, 0]) cradle_cap();
else if (part == "leg") rotate([90, 0, 0]) leg();
else if (part == "stand") translate([0, 0, -stand_floor]) stand();
else if (part == "interference") {
  intersection() {
    union() {
      base_deck(); mid_deck(); top_deck();
      translate([rail_x, 0, 0]) rail(false);
      translate([-rail_x, 0, 0]) mirror([1, 0, 0]) rail(true);
      for (yc = cradle_y) translate([0, yc, 0]) cradle_cap();
    }
    ghosts(0.05);
  }
}
else if (part == "interference_parts") {   // printed part against printed part: should be empty
  union() {
    intersection() { union() { base_deck(); mid_deck(); top_deck(); }
                     union() { translate([rail_x, 0, 0]) rail(false); translate([-rail_x, 0, 0]) mirror([1, 0, 0]) rail(true); } }
    intersection() { for (sx=[-1,1], sy=[-leg_y, leg_y]) translate([sx > 0 ? leg_x0 : -leg_x0, sy - leg_w/2, leg_z]) mirror([sx > 0 ? 0 : 1, 0, 0]) leg();
                     union() { translate([rail_x, 0, 0]) rail(false); translate([-rail_x, 0, 0]) mirror([1, 0, 0]) rail(true); } }
    intersection() { stand(); union() { translate([rail_x, 0, 0]) rail(false); translate([-rail_x, 0, 0]) mirror([1, 0, 0]) rail(true); } }
  }
}
else if (part == "interference_legs_stand") {   // legs and stand against the wheels and motors: should be empty
  intersection() {
    union() { stand(); for (sx=[-1,1], sy=[-leg_y, leg_y]) translate([sx > 0 ? leg_x0 : -leg_x0, sy - leg_w/2, leg_z]) mirror([sx > 0 ? 0 : 1, 0, 0]) leg(); }
    ghosts(0.05);
  }
}
else if (part == "coupon") {
  // one hour of plastic: a motor clamp, an IMU seat, a Nucleo corner and a nut pocket
  difference() {
    union() {
      cube([100, 60, deck_t]);
      translate([28, 30, deck_t - 0.01]) {
        difference() {
          translate([-cradle_x, -cradle_w/2, 0]) cube([2*cradle_x, cradle_w, motor_r + 4]);
          translate([0, 0, motor_r + 4]) rotate([90, 0, 0]) cylinder(d=bore_d, h=cradle_w + 2, center=true);
        }
      }
      translate([78, 40, deck_t]) rotate([0,0,imu_rot]) { pillars(imu, h=6, w=4); corner_locators(imu, leg=5, t=1.6, h=8); }
      translate([82 - nucleo[0]/2, 12 - nucleo[1]/2, deck_t]) one_corner(nucleo, 1, 1, leg=10, t=2, h=7);
    }
    for (sx=[-1,1]) translate([28 + sx*screw_x, 30, -1]) cylinder(d=3 + 2*hole_clear, h=40);
    for (sx=[-1,1]) translate([28 + sx*screw_x, 30, -0.01]) hexpocket(2.6);
  }
}
