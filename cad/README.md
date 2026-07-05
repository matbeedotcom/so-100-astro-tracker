# STS3215 Alt-AZ Floating Sled CAD

`sts3215_altaz_tilt_drive.scad` is a parametric OpenSCAD concept for driving an Alt-AZ tilt lead screw with an STS3215 servo.

The current mechanical assumption is that the existing Alt-AZ hardware already engages the lead screw with threads. Because the screw translates as it rotates, the motor cannot be hard-mounted to the Alt-AZ frame. The fixed printed base now carries a print-in-place captured slide, and the STS3215 rides on a moving sled so the motor and full lead screw travel together through the fixed threaded receiver.

The fixed base is shaped for the rounded Alt-AZ housing shown in the reference photos. It has a concave saddle for a `70 mm` diameter curved surface and four strap slots for a hose clamp, metal band, or heavy zip ties. The opposing strap slots include shallow top recesses so the bands route across the rail support strips instead of floating on their edges. There is no separate flat-base screw slot pattern in the current model. Tune `mount_surface_d`, `mount_surface_clearance`, and `saddle_drop_z` after measuring the actual housing with calipers.

The default slide interface is `printed_capture = true`. In that mode, the base prints two open C-channel rails and the sled prints separated runners parked inside them. The STL contains separate shells with `slide_clearance_xy = 0.85` and `slide_clearance_z = 1.00`, so a calibrated FDM printer should leave the sled trapped but breakable after printing. Inspect the slicer preview to make sure the shells do not merge; increase those clearances for PETG, rough first layers, or a printer that tends to elephant-foot internal gaps.

The useful printable parts are:

- `part="sled_base_print"`: base and sled as one print-in-place trapped slide, in source/model orientation.
- `part="sled_base_print_xup"`: same trapped slide, pre-rotated to the mesh diagnostic's best orientation for the current curved-saddle base.
- `part="base"` or `part="mount"`: fixed Alt-AZ base with curved saddle, strap slots, captured slide rails, and travel stops.
- `part="sled"`: moving STS3215 cradle with captured runners.
- `part="strap"`: removable top strap for the servo cradle.
- `part="coupler"`: stock-servo-horn bolt flange plus split clamp for the measured lead screw.
- `part="assembly"`: preview with base, captured rails, moving sled, screw, and existing threaded receiver.
- `part="motion"`: preview of the sled at minimum and maximum screw travel.
- `part="clearance_debug"`: rail clearances highlighted against the sled shoes.

Key assumptions:

- STS3215 body envelope defaults to `45.2 x 24.7 x 35 mm`.
- Tilt screw defaults to `lead_screw_d = 7.6` and `lead_screw_pitch = 1.0`.
- Curved mounting surface defaults to `mount_surface_d = 70`.
- Travel preview is controlled by `travel`, clamped by `stroke`.
- The print-in-place slide defaults to `print_travel = 16`, which parks the sled near mid-stroke during export.
- The older 8 mm guide-rod concept is still available by setting `printed_capture = false` and `show_guide_rods = true`, but the default model is the FDM captured slide.
- The coupler bolts to a stock servo horn using a 4-hole `horn_pcd = 16` pattern; measure your horn and tune this before printing the final coupler.
- Mesh printability diagnostics on the combined trapped assembly reported `x-up` as the best orientation. Use `part="sled_base_print_xup"` for that pre-rotated STL, then verify supports and shell separation in the slicer. Printing `sled_base_print` flat in source orientation makes the slide easiest to understand, but the curved saddle creates much more underside/support risk.

Example exports:

```bash
OPENSCAD=/tmp/openscad-snapshot/OpenSCAD-2026.06.19-x86_64.AppImage
$OPENSCAD --export-format asciistl -D 'part="sled_base_print_xup"' -o renders/sts3215_altaz_sled_base_print_in_place_xup.stl cad/sts3215_altaz_tilt_drive.scad
$OPENSCAD --export-format asciistl -D 'part="sled_base_print"' -o renders/sts3215_altaz_sled_base_print_in_place.stl cad/sts3215_altaz_tilt_drive.scad
$OPENSCAD --export-format asciistl -D 'part="base"' -o renders/sts3215_altaz_floating_base.stl cad/sts3215_altaz_tilt_drive.scad
$OPENSCAD --export-format asciistl -D 'part="sled"' -o renders/sts3215_altaz_motor_sled.stl cad/sts3215_altaz_tilt_drive.scad
$OPENSCAD --export-format asciistl -D 'part="coupler"' -o renders/sts3215_leadscrew_coupler.stl cad/sts3215_altaz_tilt_drive.scad
$OPENSCAD --export-format asciistl -D 'part="strap"' -o renders/sts3215_cradle_strap.stl cad/sts3215_altaz_tilt_drive.scad
$OPENSCAD --export-format png --imgsize 1600,1000 --autocenter --viewall -D 'part="sled_base_print"' -o renders/sts3215_altaz_sled_base_print_in_place.png cad/sts3215_altaz_tilt_drive.scad
$OPENSCAD --export-format png --imgsize 1600,1000 --autocenter --viewall -D 'part="motion"' -o renders/sts3215_altaz_floating_sled_motion.png cad/sts3215_altaz_tilt_drive.scad
```

Validation artifacts from the latest pass:

- `renders/sts3215_altaz_sled_base_print_in_place.stl`
- `renders/sts3215_altaz_sled_base_print_in_place_xup.stl`
- `renders/sts3215_altaz_sled_base_clearance_debug.png`
- `renders/printability/sts3215_altaz_sled_base_print_in_place/printability/report.md`
- `renders/printability/sts3215_altaz_sled_base_print_in_place/printability/risk_visualization.png`
