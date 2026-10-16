# SunramSim

A browser-based simulator for the Stormram/Sunram family of MRI-guided biopsy robots. It currently models **Stormram 1**: a 7-DOF robot made of a pneumatic hexapod plus a needle-insertion stage.

What it does:

- **Joint control**: the six leg lengths and the insertion depth, optionally snapped to the motor step size.
- **End-effector control**: inverse kinematics for the needle-tip pose, using sliders or a 3D handle that rotates about the needle tip. If a pose can't be reached, it names the limiting joints.
- **Image data**: load a DICOM series (files, folder, or drag-and-drop) or use the built-in synthetic prone breast phantom. The scan shows as axial, coronal, sagittal and needle-aligned slices, and as planes in the 3D view.
- **Robot placement**: position the robot base relative to the patient, either with the handle, by typing values, or with *Place robot for target*.
- **Biopsy workflow**: pick a target (and optionally an entry point), plan the trajectory, then **Align → Insert → Fire → Retract**. Moves run at the motor step rate. After firing, it reports how well the specimen notch lines up with the target.

## Running

The app has no build step and no external network dependencies (three.js and dicom-parser are in `vendor/`). Serve the folder with any static web server:

```sh
python3 -m http.server 8000      # then open http://localhost:8000
```

It also works on GitHub Pages. Opening `index.html` from `file://` won't work, because ES modules need HTTP.

Tests (Node ≥ 18):

```sh
npm test                                  # kinematics: IK/FK round trip, needle planning
node tests/make-test-dicom.mjs out-dir    # writes an oblique test DICOM series from the phantom
```

## Coordinate frames

| Frame | Definition |
|---|---|
| World | DICOM patient frame **LPS**, mm. Room "up" depends on patient position (prone ⇒ +Y). |
| Robot base | Origin at the centre of the base plate's top surface. +z is up, +x is the default needle direction. |
| Platform | Moving hexapod platform. The pose at home is `homePlatform` in the robot file. |
| Needle | Tip at zero insertion (`needle.origin`, platform frame), moving along `needle.dir`. |

Joint vector `q = [L1 … L6, d]`: L1–L6 are the leg lengths (mm, base ball-joint centre to platform ball-joint centre) and `d` is the insertion depth (mm).

## Stormram 1 geometry: status

`js/robots/stormram1.js` documents where each number comes from.

**From CAD:**
- The six base ball joints. They sit on a 100 mm radius every 60°, as in the base plate drawing (`Delrin_6mm_base_plate_round.cdr`), and the SolidWorks frame model (`Stormram1_frame_step.STEP`) agrees exactly.
- The 10° tilted base cups.
- The rack, motor and ball dimensions, and which platform ball each leg connects to.
- The base plate outline and holes, in `js/robots/stormram1BasePlate.js`.

**Still provisional:**
- The platform ball positions (the STEP mounts are known to be wrong).
- The home pose.
- The leg stroke limits (estimated as 90–185 mm from the rack and motor geometry).
- The ball-joint angle limits.
- The needle axis and stroke.
- The motor step size and step frequency.
- The biopsy gun's throw and notch dimensions.

`tools/cdr_contours.py` converts CorelDRAW laser-cutting drawings into contours in mm, with holes detected as circles. It needs `cdr2xhtml` from libcdr (`apt install libcdr-tools`).

### CAD meshes

Export each part as STL or GLB, in **mm**, in the **CAD assembly frame at the home configuration**. Use one file per body: `base`, `platform`, `motor1…6`, `rack1…6`, `needle` (the insertion carriage with the gun and needle).

The simulator converts each part into its body's frame. Set `cad.cadToBase` if the CAD origin isn't the robot base frame.

You can try parts interactively in the *Robot* tab. To make them permanent, add them to `cad.parts`.

## Layout

```
js/core/la.js              small linear-algebra library (plain arrays; runs in browser and Node)
js/robots/stewart.js       generic Gough-Stewart IK / Newton FK / Jacobian / joint limits
js/robots/hexapodRobot.js  hexapod + insertion axis: needle kinematics, trajectory planning, quantisation
js/robots/hexapodVisual.js three.js rendering, CAD part loading
js/robots/stormram1.js     Stormram 1 parameters (sources documented in the file)
js/robots/stormram1BasePlate.js  base plate outline and holes, generated from the CorelDRAW file
tools/cdr_contours.py      CorelDRAW (.cdr) drawing → contours in mm
js/robots/registry.js      list of robots (Stormram 2/3, Sunram 5/7 etc. are placeholders)
js/imaging/                volume model, DICOM loader, synthetic phantom
js/ui/sliceViews.js        2D reformatted views with overlays
js/main.js                 application
```

## Limitations

- **DICOM**: only uncompressed transfer syntaxes are supported. Convert JPEG/JPEG 2000 data first, e.g. with `gdcmconv --raw`. Single-frame series and enhanced multi-frame series are supported.
- **Collisions**: leg-to-leg, robot-to-patient and robot-to-scanner collisions are not checked.
- **Tissue**: the needle is modelled as rigid, with no tissue deformation or needle deflection.
