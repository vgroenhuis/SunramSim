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

## Entering the real Stormram 1 geometry

The geometry in `js/robots/stormram1.js` is **provisional**: it was estimated from photographs. To make it exact, enter:

1. `hexapod.baseJoints`: the 6 base ball-joint centres in the robot base frame.
2. `hexapod.platformJoints`: the 6 platform ball-joint centres in the platform frame, plus `homePlatform` (where the platform sits at home).
3. `legMin` / `legMax`: the leg lengths at each end of the rack stroke.
4. The ball-joint cone axes and maximum angles.
5. `needle.origin` / `needle.dir` / `needle.max`: the insertion axis and stroke.
6. `motors`: step size (mm) and maximum step frequency.
7. `biopsy`: needle throw and notch dimensions of the biopsy gun used.

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
js/robots/stormram1.js     Stormram 1 parameters
js/robots/registry.js      list of robots (Stormram 2/3, Sunram 5/7 etc. are placeholders)
js/imaging/                volume model, DICOM loader, synthetic phantom
js/ui/sliceViews.js        2D reformatted views with overlays
js/main.js                 application
```

## Limitations

- **DICOM**: only uncompressed transfer syntaxes are supported. Convert JPEG/JPEG 2000 data first, e.g. with `gdcmconv --raw`. Single-frame series and enhanced multi-frame series are supported.
- **Collisions**: leg-to-leg, robot-to-patient and robot-to-scanner collisions are not checked.
- **Tissue**: the needle is modelled as rigid, with no tissue deformation or needle deflection.
