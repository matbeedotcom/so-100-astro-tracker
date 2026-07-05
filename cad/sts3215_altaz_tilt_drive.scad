// STS3215 floating sled for an Alt-AZ tilt lead screw.
// Units: millimeters.
//
// Design intent:
// - The Alt-AZ mount already has threaded hardware for the tilt lead screw.
// - When the servo rotates the screw, the screw advances/retracts through that
//   hardware, so the motor must float with the screw instead of being hard
//   mounted to the Alt-AZ frame.
// - A fixed base carries a printed-in-place captured slide. A moving sled holds
//   the STS3215, the stock-horn coupler, and the lead screw. The sled translates
//   along X as the screw thread changes position.
//
// Coordinate system:
// - X is the lead-screw travel axis.
// - +X points from the motor toward the existing threaded Alt-AZ hardware.
// - Z is up from the Alt-AZ mounting plate.

$fn = 72;

part = "assembly";      // assembly, sled_base_print, sled_base_print_xup, base, sled, strap, coupler, hardware, motion, clearance_debug
travel = 16;            // sled extension in mm, clamped visually to stroke
print_travel = 16;      // parked sled position for the print-in-place base/sled export
cutaway = false;        // false, true
cut_axis = "y";         // "x", "y", or "z"
cut_side = "positive"; // remove "positive" or "negative" half along cut_axis
cut_offset = 0;

// Measured tilt screw.
lead_screw_d = 7.6;
lead_screw_pitch = 1.0;
lead_screw_bore = lead_screw_d + 0.55;
lead_screw_len = 122;
stroke = 34;

// Published STS3215 envelope is approximately 45.2 x 24.7 x 35 mm.
servo_width_y = 45.2;
servo_depth_x = 24.7;
servo_height_z = 35.0;
servo_clearance = 0.65;
servo_axis_z = 36;
servo_home_x = -33;

// Fixed base and guide hardware.
printed_capture = true; // true prints the base and sled as a trapped FDM slide.
show_guide_rods = false;
base_len_x = 150;
base_width_y = 100;
base_thick = 5;
wall = 4;
mount_surface_d = 70;
mount_surface_clearance = 0.8;
saddle_len_x = 120;
saddle_width_y = 82;
saddle_drop_z = 12;
saddle_strap_slot_x = 10;
saddle_strap_slot_y = 5.2;
saddle_strap_x = [-45, 45];
saddle_strap_slot_bottom_z = -13;
saddle_strap_slot_top_z = 20;
saddle_strap_recess_y = 64;
saddle_strap_recess_z = 10.2;
saddle_strap_recess_depth = 2.2;
rail_d = 8;
rail_clearance = 0.55;
rail_y = 37;
rail_z = 14;
rail_len = 124;
rail_end_block_x = 50;
receiver_x = 58;

// Print-in-place slide geometry. The sled runners are trapped by open C-channel
// lips with enough XY/Z clearance to break free on a tuned FDM printer.
slide_clearance_xy = 0.85;
slide_clearance_z = 1.00;
slide_floor_thick = 2.4;
slide_runner_w = 11.0;
slide_runner_h = 7.2;
slide_runner_len = 52;
slide_channel_wall = 3.0;
slide_lip_w = 3.2;
slide_lip_thick = 2.6;
slide_post_w = 3.2;
slide_floor_top_z = base_thick + slide_floor_thick;
slide_runner_z = slide_floor_top_z + slide_clearance_z + slide_runner_h / 2;
slide_lip_top_z = slide_floor_top_z + slide_clearance_z + slide_runner_h + slide_clearance_z + slide_lip_thick;

// Moving sled.
sled_len_x = 52;
sled_width_y = 86;
sled_plate_thick = 5;
sled_z = printed_capture
  ? slide_lip_top_z + 0.85 + sled_plate_thick / 2
  : rail_z + rail_d / 2 + sled_plate_thick / 2 + 1.2;
strap_gap_z = servo_axis_z + servo_height_z / 2 + servo_clearance;

// Coupler and stock horn interface.
coupler_outer_d = 18;
coupler_len = 35;
horn_flange_d = 30;
horn_flange_w = 5.5;
horn_pcd = 16;
horn_bolt_d = 2.4;      // M2 clearance, tune to your horn screws.
horn_center_d = 6.4;    // STS3215 output shaft/horn boss clearance.
clamp_bolt_d = 3.3;     // M3 clearance.
clamp_bolt_spacing = 13;

function clamp_value(v, lo, hi) = min(max(v, lo), hi);
function sled_offset(t = travel) = clamp_value(t, 0, stroke);
function servo_x(t = travel) = servo_home_x + sled_offset(t);
function sled_x(t = travel) = servo_x(t);
function servo_front_x(t = travel) = servo_x(t) + servo_depth_x / 2;
function coupler_center_x(t = travel) = servo_front_x(t) + 14;
function screw_start_x(t = travel) = servo_front_x(t) + 3;
function screw_end_x(t = travel) = screw_start_x(t) + lead_screw_len;
function screw_center_x(t = travel) = screw_start_x(t) + lead_screw_len / 2;

module cyl_x(d, l) {
  rotate([0, 90, 0]) cylinder(d = d, h = l, center = true);
}

module cone_x(d1, d2, l) {
  rotate([0, 90, 0]) cylinder(d1 = d1, d2 = d2, h = l, center = true);
}

module rounded_box(size, r = 2) {
  sx = size[0];
  sy = size[1];
  sz = size[2];
  hull() {
    for (x = [-sx / 2 + r, sx / 2 - r])
      for (y = [-sy / 2 + r, sy / 2 - r])
        for (z = [-sz / 2 + r, sz / 2 - r])
          translate([x, y, z]) sphere(r = r);
  }
}

module rib_y(points, thickness) {
  y = thickness / 2;
  polyhedron(
    points = [
      [points[0][0], -y, points[0][1]],
      [points[1][0], -y, points[1][1]],
      [points[2][0], -y, points[2][1]],
      [points[0][0],  y, points[0][1]],
      [points[1][0],  y, points[1][1]],
      [points[2][0],  y, points[2][1]]
    ],
    faces = [
      [0, 1, 2],
      [5, 4, 3],
      [0, 3, 4, 1],
      [1, 4, 5, 2],
      [2, 5, 3, 0]
    ]
  );
}

module prism_x(points_yz, length) {
  n = len(points_yz);
  polyhedron(
    points = concat(
      [for (p = points_yz) [-length / 2, p[0], p[1]]],
      [for (p = points_yz) [ length / 2, p[0], p[1]]]
    ),
    faces = concat(
      [[for (i = [n - 1 : -1 : 0]) i]],
      [[for (i = [0 : n - 1]) i + n]],
      [for (i = [0 : n - 1]) [i, (i + 1) % n, (i + 1) % n + n, i + n]]
    )
  );
}

module teardrop_runner_x(size) {
  sx = size[0];
  sy = size[1];
  sz = size[2];
  shoulder_z = -sz / 2 + min(2.7, sz * 0.42);
  lower_z = -sz / 2 + min(1.2, sz * 0.22);

  prism_x([
    [-sy / 2,  sz / 2],
    [ sy / 2,  sz / 2],
    [ sy / 2,  shoulder_z],
    [ sy * 0.34, lower_z],
    [0,        -sz / 2],
    [-sy * 0.34, lower_z],
    [-sy / 2,  shoulder_z]
  ], sx);
}

module chamfered_capture_lip_x(size, inward_sign = -1) {
  sx = size[0];
  sy = size[1];
  sz = size[2];
  rise = min(1.45, sz * 0.58);
  top = sz / 2;
  bottom = -sz / 2;

  points_yz = inward_sign < 0
    ? [[-sy / 2, bottom + rise], [ sy / 2, bottom], [ sy / 2, top], [-sy / 2, top]]
    : [[-sy / 2, bottom], [ sy / 2, bottom + rise], [ sy / 2, top], [-sy / 2, top]];

  prism_x(points_yz, sx);
}

module cutaway_remove_box(axis = cut_axis, side = cut_side, offset = cut_offset, size = 260) {
  s = size;
  if (axis == "x")
    translate(side == "positive" ? [offset, -s / 2, -s / 2] : [-s + offset, -s / 2, -s / 2]) cube([s, s, s]);
  else if (axis == "y")
    translate(side == "positive" ? [-s / 2, offset, -s / 2] : [-s / 2, -s + offset, -s / 2]) cube([s, s, s]);
  else
    translate(side == "positive" ? [-s / 2, -s / 2, offset] : [-s / 2, -s / 2, -s + offset]) cube([s, s, s]);
}

module rail_rod(y = rail_y) {
  translate([0, y, rail_z])
    cyl_x(d = rail_d, l = rail_len);
}

module rod_end_block(x, y) {
  difference() {
    translate([x, y, base_thick + 9])
      rounded_box([12, 18, 18], 1.8);
    translate([x, y, rail_z])
      cyl_x(d = rail_d + 0.35, l = 14);
    translate([x, y, base_thick + 3])
      cylinder(d = 3.2, h = 10, center = true);
  }
}

module capture_channel(y = rail_y) {
  channel_w = slide_runner_w + 2 * slide_clearance_xy + 2 * slide_channel_wall;
  wall_z = slide_floor_top_z + (slide_clearance_z + slide_runner_h + slide_clearance_z + slide_lip_thick) / 2;
  wall_h = slide_clearance_z + slide_runner_h + slide_clearance_z + slide_lip_thick;
  side_y = slide_runner_w / 2 + slide_clearance_xy + slide_channel_wall / 2;
  lip_y = slide_runner_w / 2 + slide_clearance_xy - slide_lip_w / 2;
  lip_z = slide_floor_top_z + slide_clearance_z + slide_runner_h + slide_clearance_z + slide_lip_thick / 2;

  translate([0, y, base_thick + slide_floor_thick / 2])
    rounded_box([rail_len - 12, channel_w, slide_floor_thick], 1.0);

  for (s = [-1, 1]) {
    translate([0, y + s * side_y, wall_z])
      rounded_box([rail_len - 12, slide_channel_wall, wall_h], 0.8);

    translate([0, y + s * lip_y, lip_z])
      chamfered_capture_lip_x([rail_len - 12, slide_lip_w, slide_lip_thick], -s);
  }
}

module capture_runner(t = travel, y = rail_y) {
  runner_top = slide_runner_z + slide_runner_h / 2;
  plate_bottom = sled_z - sled_plate_thick / 2;
  post_h = max(0.1, plate_bottom - runner_top);

  translate([sled_x(t), y, slide_runner_z])
    teardrop_runner_x([slide_runner_len, slide_runner_w, slide_runner_h]);

  translate([sled_x(t), y, runner_top + post_h / 2])
    rounded_box([slide_runner_len - 4, slide_post_w, post_h], 0.8);
}

module curved_mount_surface_ghost() {
  color([0.10, 0.10, 0.11, 0.30])
    translate([0, 0, -mount_surface_d / 2])
      cyl_x(d = mount_surface_d, l = saddle_len_x + 18);
}

module curved_mount_band_ghost() {
  for (x = saddle_strap_x)
    color([0.95, 0.58, 0.16, 0.34])
      difference() {
        translate([x, 0, -mount_surface_d / 2])
          cyl_x(d = mount_surface_d + 7, l = saddle_strap_slot_x * 0.72);
        translate([x, 0, -mount_surface_d / 2])
          cyl_x(d = mount_surface_d + 1.8, l = saddle_strap_slot_x * 0.9);
      }
}

module curved_saddle_blank() {
  translate([0, 0, (base_thick - saddle_drop_z) / 2])
    rounded_box([saddle_len_x, saddle_width_y, base_thick + saddle_drop_z], 2.4);
}

module curved_saddle_cut() {
  translate([0, 0, -mount_surface_d / 2])
    cyl_x(d = mount_surface_d + mount_surface_clearance, l = saddle_len_x + 10);
}

module saddle_strap_slots() {
  slot_h = saddle_strap_slot_top_z - saddle_strap_slot_bottom_z;
  slot_z = (saddle_strap_slot_top_z + saddle_strap_slot_bottom_z) / 2;
  for (x = saddle_strap_x)
    for (y = [-saddle_width_y / 2 + 7, saddle_width_y / 2 - 7])
      translate([x, y, slot_z])
        rounded_box([saddle_strap_slot_x, saddle_strap_slot_y, slot_h], 1.0);
}

module saddle_strap_recesses() {
  for (x = saddle_strap_x)
    translate([x, 0, saddle_strap_recess_z])
      rounded_box([saddle_strap_slot_x + 1.6, saddle_strap_recess_y, saddle_strap_recess_depth], 1.2);
}

module fixed_thread_receiver_ghost() {
  color([0.62, 0.86, 0.95, 0.28])
    translate([receiver_x + 20, 0, servo_axis_z])
      rounded_box([8, 36, 46], 1.8);

  color([0.54, 0.55, 0.58, 0.6])
    translate([receiver_x + 17, 0, servo_axis_z])
      cyl_x(d = 15, l = 8);
}

module base_part() {
  difference() {
    union() {
      curved_saddle_blank();

      translate([0, 0, base_thick / 2])
        rounded_box([base_len_x, base_width_y, base_thick], 2.2);

      if (printed_capture)
        for (y = [-rail_y, rail_y])
          capture_channel(y);
      else
        for (y = [-rail_y, rail_y])
          for (x = [-rail_len / 2, rail_len / 2])
            rod_end_block(x, y);

      // Low cable guard and rear travel stop.
      translate([-base_len_x / 2 + 10, 0, base_thick + 8])
        rounded_box([8, base_width_y - 12, 16], 1.6);

      // Front stop, leaving the real threaded hardware as the axial reference.
      translate([base_len_x / 2 - 10, 0, base_thick + 8])
        rounded_box([8, base_width_y - 12, 16], 1.6);

      if (!printed_capture)
        for (y = [-rail_y, rail_y])
          translate([0, y, base_thick + 2.5])
            rounded_box([rail_len - 12, 12, 5], 1.4);
    }

    curved_saddle_cut();
    saddle_strap_recesses();
    saddle_strap_slots();
  }
}

module rail_hardware_ghost() {
  if (show_guide_rods)
    color([0.72, 0.72, 0.70, 0.75])
      for (y = [-rail_y, rail_y])
        rail_rod(y);
}

module strap_bolt_pattern(t = travel, h = 80) {
  side_y = servo_width_y / 2 + servo_clearance + wall / 2;
  top_z = servo_axis_z + servo_height_z / 2 + servo_clearance + wall / 2;
  for (x = [servo_x(t) - 7, servo_x(t) + 7])
    for (y = [-side_y, side_y])
      translate([x, y, top_z])
        cylinder(d = clamp_bolt_d, h = h, center = true);
}

module sled_part(t = travel) {
  side_y = servo_width_y / 2 + servo_clearance + wall / 2;
  side_wall_h = servo_height_z + 14;
  side_wall_z = base_thick + side_wall_h / 2 + 6.5;
  side_wall_len = servo_depth_x + 12;
  rear_x = servo_x(t) - servo_depth_x / 2 - wall / 2 - 1.0;
  face_x = servo_front_x(t) + wall / 2 + 1.4;
  face_h = servo_height_z + 8;
  face_z = servo_axis_z + 4.5;
  face_w = servo_width_y + 12;

  difference() {
    union() {
      translate([sled_x(t), 0, sled_z])
        rounded_box([sled_len_x, sled_width_y, sled_plate_thick], 1.8);

      for (y = [-rail_y, rail_y])
        if (printed_capture)
          capture_runner(t, y);
        else
          translate([sled_x(t), y, rail_z])
            rounded_box([sled_len_x, 14, 14], 1.8);

      translate([servo_x(t), -side_y, side_wall_z])
        rounded_box([side_wall_len, wall, side_wall_h], 1.8);
      translate([servo_x(t), side_y, side_wall_z])
        rounded_box([side_wall_len, wall, side_wall_h], 1.8);

      translate([servo_x(t), 0, sled_z + sled_plate_thick / 2 + wall / 2])
        rounded_box([side_wall_len, servo_width_y + 2 * servo_clearance + 2 * wall, wall], 1.4);

      translate([rear_x, 0, side_wall_z])
        rounded_box([wall, servo_width_y + 2 * servo_clearance + 2 * wall, side_wall_h], 1.8);

      translate([face_x, 0, face_z])
        rounded_box([wall + 1.4, face_w, face_h], 2.0);

      for (y = [-side_y + 2.5, side_y - 2.5])
        translate([0, y, 0])
          rib_y([[face_x, sled_z + sled_plate_thick / 2],
                 [face_x, servo_axis_z - 14],
                 [face_x + 23, sled_z + sled_plate_thick / 2]], 4.5);
    }

    // Linear rail clearances through the sled shoes when using metal rods.
    if (!printed_capture)
      for (y = [-rail_y, rail_y])
        translate([sled_x(t), y, rail_z])
          cyl_x(d = rail_d + rail_clearance, l = sled_len_x + 2);

    // Servo body clearance through the cradle floor and side walls.
    translate([servo_x(t), 0, servo_axis_z])
      rounded_box([servo_depth_x + 2 * servo_clearance,
                   servo_width_y + 2 * servo_clearance,
                   servo_height_z + 2 * servo_clearance], 1.2);

    // Servo output and horn clearance.
    translate([face_x, 0, servo_axis_z])
      cyl_x(d = horn_flange_d + 8, l = wall + 10);

    // Strap screws.
    strap_bolt_pattern(t, 80);

    // Rear cable relief.
    translate([rear_x, 0, servo_axis_z - servo_height_z / 2 + 9])
      rounded_box([wall + 2, 16, 12], 1.2);

    // Servo bottom relief; leaves the captured runners and rear stop.
    translate([servo_x(t), 0, sled_z + sled_plate_thick / 2 + 1.4])
      rounded_box([servo_depth_x + 1.4, servo_width_y + 2 * servo_clearance, 5], 1.0);
  }
}

module strap_part(t = travel) {
  side_y = servo_width_y / 2 + servo_clearance + wall / 2;
  top_z = servo_axis_z + servo_height_z / 2 + servo_clearance + wall / 2;
  difference() {
    translate([servo_x(t), 0, top_z])
      rounded_box([servo_depth_x + 8, servo_width_y + 2 * servo_clearance + 2 * wall, wall], 1.8);

    translate([servo_x(t), 0, top_z - wall / 2 - 0.1])
      rounded_box([servo_depth_x + 3, servo_width_y + 2 * servo_clearance, wall + 1], 1.1);

    strap_bolt_pattern(t, 18);

    for (x = [servo_x(t) - 7, servo_x(t) + 7])
      for (y = [-side_y, side_y])
        translate([x, y, top_z + wall / 2 - 1.1])
          cylinder(d = 6.4, h = 2.4, center = true);
  }
}

module coupler_part(t = travel) {
  center_x = coupler_center_x(t);
  clamp_start_x = center_x - coupler_len / 2 + horn_flange_w;
  clamp_center_x = clamp_start_x + (coupler_len - horn_flange_w) / 2;
  horn_x = center_x - coupler_len / 2 + horn_flange_w / 2;

  difference() {
    union() {
      translate([clamp_center_x, 0, servo_axis_z])
        cyl_x(d = coupler_outer_d, l = coupler_len - horn_flange_w);
      translate([horn_x, 0, servo_axis_z])
        cyl_x(d = horn_flange_d, l = horn_flange_w);
      translate([horn_x + 3.3, 0, servo_axis_z])
        cone_x(d1 = horn_flange_d, d2 = coupler_outer_d, l = 6.5);
    }

    translate([center_x + 2, 0, servo_axis_z])
      cyl_x(d = lead_screw_bore, l = coupler_len + 4);

    translate([horn_x - 0.2, 0, servo_axis_z])
      cyl_x(d = horn_center_d, l = horn_flange_w + 1.4);

    for (a = [0, 90, 180, 270])
      translate([horn_x, cos(a) * horn_pcd / 2, servo_axis_z + sin(a) * horn_pcd / 2])
        cyl_x(d = horn_bolt_d, l = horn_flange_w + 2);

    // Horizontal split and two M3 pinch screws for the lead-screw clamp.
    translate([clamp_center_x + 3, 0, servo_axis_z])
      cube([coupler_len - horn_flange_w + 3, coupler_outer_d + 4, 1.5], center = true);

    for (x = [clamp_center_x + 1 - clamp_bolt_spacing / 2, clamp_center_x + 1 + clamp_bolt_spacing / 2]) {
      translate([x, coupler_outer_d * 0.27, servo_axis_z])
        cylinder(d = clamp_bolt_d, h = coupler_outer_d + 6, center = true);
      translate([x, coupler_outer_d * 0.27, servo_axis_z - coupler_outer_d / 2 + 1.7])
        cylinder(d = 6.4, h = 3.4, center = true, $fn = 6);
    }
  }
}

module servo_ghost(t = travel, alpha = 0.42) {
  color([0.08, 0.11, 0.14, alpha])
    translate([servo_x(t), 0, servo_axis_z])
      rounded_box([servo_depth_x, servo_width_y, servo_height_z], 2.0);

  color([0.16, 0.20, 0.24, min(alpha + 0.12, 1)])
    translate([servo_front_x(t) + 2.2, 0, servo_axis_z])
      cyl_x(d = 19, l = 4.4);

  color([0.7, 0.7, 0.72, min(alpha + 0.38, 1)])
    translate([servo_front_x(t) + 5.3, 0, servo_axis_z])
      cyl_x(d = 6.0, l = 2.8);
}

module lead_screw_ghost(t = travel, alpha = 0.82) {
  len = lead_screw_len;
  center_x = screw_center_x(t);

  color([0.65, 0.65, 0.62, alpha])
    translate([center_x, 0, servo_axis_z])
      cyl_x(d = lead_screw_d, l = len);

  // Visual 1 mm pitch marker. This is not a modeled thread profile.
  color([0.9, 0.9, 0.86, 0.40])
    for (i = [0 : 1 : floor(len / lead_screw_pitch)])
      translate([center_x - len / 2 + i * lead_screw_pitch, 0, servo_axis_z])
        cyl_x(d = lead_screw_d + 0.42, l = 0.16);
}

module sled_assembly(t = travel, alpha = 1.0) {
  color([0.10, 0.32, 0.42, alpha]) sled_part(t);
  color([0.95, 0.52, 0.16, alpha]) strap_part(t);
  color([0.86, 0.76, 0.44, alpha]) coupler_part(t);
  servo_ghost(t, 0.42 * alpha);
  lead_screw_ghost(t, 0.82 * alpha);
}

module hardware_preview(t = travel) {
  curved_mount_surface_ghost();
  curved_mount_band_ghost();
  rail_hardware_ghost();
  fixed_thread_receiver_ghost();
  servo_ghost(t);
  lead_screw_ghost(t);
}

module assembly_model(t = travel) {
  curved_mount_surface_ghost();
  curved_mount_band_ghost();
  color([0.08, 0.25, 0.34, 1.0]) base_part();
  rail_hardware_ghost();
  fixed_thread_receiver_ghost();
  sled_assembly(t);
}

module sled_base_print_model(t = print_travel) {
  color([0.08, 0.25, 0.34, 1.0]) base_part();
  color([0.10, 0.32, 0.42, 1.0]) sled_part(t);
}

module sled_base_print_xup_model(t = print_travel) {
  rotate([0, -90, 0])
    sled_base_print_model(t);
}

module motion_preview() {
  curved_mount_surface_ghost();
  curved_mount_band_ghost();
  color([0.08, 0.25, 0.34, 1.0]) base_part();
  rail_hardware_ghost();
  fixed_thread_receiver_ghost();
  sled_assembly(0, 0.36);
  sled_assembly(stroke, 0.82);
}

module clearance_debug() {
  curved_mount_surface_ghost();
  curved_mount_band_ghost();
  color([0.08, 0.25, 0.34, 0.45]) base_part();
  color([1.0, 0.1, 0.1, 0.5])
    for (y = [-rail_y, rail_y])
      if (printed_capture) {
        translate([sled_x(travel), y, slide_runner_z])
          rounded_box([slide_runner_len + 2 * slide_clearance_xy,
                       slide_runner_w + 2 * slide_clearance_xy,
                       slide_runner_h + 2 * slide_clearance_z], 0.7);
      } else {
        translate([sled_x(travel), y, rail_z])
          cyl_x(d = rail_d + rail_clearance, l = sled_len_x + 2);
      }
  rail_hardware_ghost();
  sled_assembly(travel, 0.62);
}

module selected_model() {
  if (part == "sled_base_print" || part == "print_in_place") sled_base_print_model(print_travel);
  else if (part == "sled_base_print_xup" || part == "print_in_place_xup") sled_base_print_xup_model(print_travel);
  else if (part == "base" || part == "mount") base_part();
  else if (part == "sled") sled_part();
  else if (part == "coupler") translate([-coupler_center_x(travel), 0, -servo_axis_z]) coupler_part();
  else if (part == "strap") translate([-servo_x(travel), 0, -strap_gap_z - wall / 2]) strap_part();
  else if (part == "hardware") hardware_preview();
  else if (part == "motion") motion_preview();
  else if (part == "clearance_debug") clearance_debug();
  else assembly_model();
}

module cutaway_model() {
  difference() {
    selected_model();
    if (cutaway) cutaway_remove_box();
  }
}

cutaway_model();
