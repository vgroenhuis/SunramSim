// Stormram 1: MRI-compatible 7-DOF breast biopsy robot. A hexapod of six pneumatic linear
// stepper motors (each pivoting in a ball joint on the base, its rack ending in a ball joint under
// the platform) plus a seventh stepper on the platform that drives the biopsy needle forward.
//
// Robot base frame (mm): origin = centre of the top surface of the base plate, +z up, x/y as in the
// laser-cutting drawing of the base plate (Delrin_6mm_base_plate_round.cdr).
//
// Sources:
//   - Base ball joints: the six Ø46 housing holes of the base plate drawing (r = 100 mm, every 60°)
//     and the SolidWorks frame model (Stormram1_frame_step.STEP), which agree exactly. Ball centre
//     height above the plate and the 10° tilted cups are taken from the STEP model.
//   - Leg hardware (rack 130.1 mm, rack end 11.1 mm below the platform ball, motor 34–64 mm from the
//     base ball, R20 / R7.5 balls), the 100 × 150 × 6 mm platform grid and the leg-to-platform
//     connectivity: STEP model.
// PROVISIONAL (to be confirmed): platform ball positions (the STEP mounts are known to be wrong),
// home pose, stroke limits, ball-joint angle limits, needle axis and motor step size/frequency.

import { DEG, I3, rotZ, mulMV, sub, normalize } from '../core/la.js';
import { HexapodNeedleRobot } from './hexapodRobot.js';
import { STORMRAM1_BASE_PLATE } from './stormram1BasePlate.js';

const BASE_BALL_Z = 1.0; // base ball centres above the plate top (STEP model; plate is 10 mm there)

// Base ball centres, in leg order (leg i of the STEP model).
const BASE_JOINTS = [-150, -30, -90, 30, 90, 150].map((a) => [
  100 * Math.cos(a * DEG), 100 * Math.sin(a * DEG), BASE_BALL_Z,
]);

// Platform ball centres in the platform frame (origin = underside centre of the grid plate, +x along
// its 150 mm side, +z up). From the STEP model, which places them on the 2.5 mm mounting grid.
const PLATFORM_JOINTS = [
  [60, -17.5, -16.5],
  [32.5, 35, -16.5],
  [57.5, 25, -16.5],
  [-60, 32.5, -16.5],
  [-60, 7.5, -16.5],
  [32.5, -35, -16.5],
];

// Home: platform level, at the yaw of the STEP model, its ball centroid above the base centre.
const HOME_YAW = -150;
const HOME_HEIGHT = 136;

const CUP_TILT = 10; // base cups are tilted 10° from vertical (STEP model)

export const stormram1Config = (() => {
  const R = rotZ(HOME_YAW * DEG);
  const c = PLATFORM_JOINTS.reduce((s, a) => [s[0] + a[0] / 6, s[1] + a[1] / 6, 0], [0, 0, 0]);
  const rc = mulMV(R, c);
  const homeP = [-rc[0], -rc[1], HOME_HEIGHT];
  // Tilt each base cup towards its leg's home direction.
  const cupAxes = BASE_JOINTS.map((b, i) => {
    const P = sub([...mulMV(R, PLATFORM_JOINTS[i])].map((v, k) => v + homeP[k]), b);
    const az = Math.atan2(P[1], P[0]);
    const t = CUP_TILT * DEG;
    return normalize([Math.sin(t) * Math.cos(az), Math.sin(t) * Math.sin(az), Math.cos(t)]);
  });
  return {
    id: 'stormram1',
    name: 'Stormram 1',
    year: 2016,
    description: '7-DOF hexapod with needle insertion, actuated by seven pneumatic linear stepper motors.',
    provisional: true,
    provisionalNote: 'Base geometry is from the CAD files; platform ball positions, home pose, stroke limits and the needle axis are still provisional.',
    hexapod: {
      baseJoints: BASE_JOINTS,
      platformJoints: PLATFORM_JOINTS,
      // Estimated: shortest when the rack cup reaches the motor (~85 mm), longest when the rack end
      // approaches the pinion inside the motor (STEP pose reaches 180.6 mm).
      legMin: 90,
      legMax: 185,
      baseJointAxes: cupAxes,
      platformJointAxes: PLATFORM_JOINTS.map(() => [0, 0, -1]),
      baseJointMaxAngle: 40,
      platformJointMaxAngle: 50,
    },
    homePlatform: { p: homeP, R },
    needle: {
      origin: [150, 0, 95], // needle tip at zero insertion, platform frame (estimated)
      dir: [1, 0, 0],
      min: 0,
      max: 70,
    },
    motors: {
      legStep: 0.25,          // mm per step (assumed)
      insertionStep: 0.25,
      stepFrequency: 8,       // Hz, limited by ~5 m pneumatic tubes (assumed)
    },
    biopsy: {
      gauge: '14G',
      diameter: 2.1,          // mm
      throw: 22,              // stroke of the biopsy gun when fired (mm)
      tipLength: 5,           // trocar tip, distal of the specimen notch (mm)
      notchLength: 18,        // specimen notch length (mm)
    },
    visual: {
      basePlate: STORMRAM1_BASE_PLATE,
      basePlateRadius: 133,
      basePlateThickness: 6,
      footHeight: 120,
      feet: { azimuths: [0, 120, 240], radius: 80, width: 50, thickness: 6, height: 120 },
      platformRect: [-75, 75, -50, 50],
      platformThickness: 6,
      baseBallRadius: 20,
      baseCupRadius: 23,
      platformBallRadius: 7.5,
      platformCupRadius: 10.5,
      motorSize: [80, 40, 28], // across, across, along leg
      motorOffset: 48.8,       // motor body centre from base ball centre, along the leg
      rackLength: 130.1,
      rackTopOffset: 11.1,     // rack end below the platform ball centre, along the leg
      rackSection: [4.1, 12.5],
      gunLength: 150,
      gunRadius: 12,
    },
    // Optional CAD meshes (STL/GLB) per body, exported in the CAD assembly frame at the home
    // configuration. `cadToBase` maps CAD assembly coordinates to the robot base frame.
    cad: {
      cadToBase: { p: [0, 0, 0], R: I3() },
      parts: {},
    },
  };
})();

export function createStormram1() {
  return new HexapodNeedleRobot(stormram1Config);
}
