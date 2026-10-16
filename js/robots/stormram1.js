// Stormram 1: MRI-compatible 7-DOF breast biopsy robot. A hexapod of six pneumatic linear
// stepper motors (each pivoting in a ball joint on the base, its rack ending in a ball joint under
// the platform) plus a seventh stepper on the platform that drives the biopsy needle forward.
//
// GEOMETRY IS PROVISIONAL: the joint coordinates below are estimated from photographs and must be
// replaced by the CAD values. Everything is in millimetres in the robot base frame:
//   origin = centre of the top surface of the base plate, +z up, +x = default needle direction.

import { DEG, sub, normalize, I3 } from '../core/la.js';
import { HexapodNeedleRobot } from './hexapodRobot.js';

const BASE_RADIUS = 115;      // radius of the circle through the base ball-joint centres
const BASE_PAIR_HALF = 12;    // half the angular spacing within a base joint pair (deg)
const BASE_Z = 30;            // base ball-joint centres above the base plate
const PLAT_RADIUS = 75;       // radius of the circle through the platform ball-joint centres
const PLAT_PAIR_HALF = 12;
const PLAT_Z = -25;           // platform ball-joint centres below the platform frame origin
const HOME_HEIGHT = 230;      // platform frame origin above the base frame origin at home
const PAIR_OFFSET = 0;        // rotation of the whole pattern about z (deg)

function circlePoint(r, angDeg, z) {
  const a = angDeg * DEG;
  return [r * Math.cos(a), r * Math.sin(a), z];
}

function buildJoints() {
  const base = [], plat = [];
  for (let k = 0; k < 3; k++) {
    const c = 120 * k + PAIR_OFFSET + 60; // base pairs at 60, 180, 300 deg: needle side (+x) stays open
    // Leg 2k connects base (c + half) to platform (c + 60 - half); leg 2k+1 connects base
    // (c - half) to platform (c - 60 + half): the classic crossed 6-6 arrangement.
    base.push(circlePoint(BASE_RADIUS, c + BASE_PAIR_HALF, BASE_Z));
    plat.push(circlePoint(PLAT_RADIUS, c + 60 - PLAT_PAIR_HALF, PLAT_Z));
    base.push(circlePoint(BASE_RADIUS, c - BASE_PAIR_HALF, BASE_Z));
    plat.push(circlePoint(PLAT_RADIUS, c - 60 + PLAT_PAIR_HALF, PLAT_Z));
  }
  return { base, plat };
}

export const stormram1Config = (() => {
  const { base, plat } = buildJoints();
  const homeP = [0, 0, HOME_HEIGHT];
  // Ball-joint cones are centred on the home leg direction (the sockets are mounted tilted).
  const homeDirs = base.map((b, i) => normalize(sub([plat[i][0] + homeP[0], plat[i][1] + homeP[1], plat[i][2] + homeP[2]], b)));
  return {
    id: 'stormram1',
    name: 'Stormram 1',
    year: 2016,
    description: '7-DOF hexapod with needle insertion, actuated by seven pneumatic linear stepper motors.',
    provisional: true,
    hexapod: {
      baseJoints: base,
      platformJoints: plat,
      legMin: 150,
      legMax: 245,
      baseJointAxes: homeDirs,
      platformJointAxes: homeDirs.map((d) => [-d[0], -d[1], -d[2]]),
      baseJointMaxAngle: 35,
      platformJointMaxAngle: 35,
    },
    homePlatform: { p: homeP, R: I3() },
    needle: {
      origin: [150, 0, 95], // needle tip at zero insertion, platform frame
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
      basePlateRadius: 160,
      basePlateThickness: 10,
      footHeight: 160,
      platformThickness: 10,
      platformJointCupHeight: 22,
      motorSize: [62, 62, 48], // across, across, along leg
      motorOffset: 22,         // motor body centre from base ball centre, along the leg
      rackLength: 300,
      rackSection: [9, 14],
      gunLength: 150,
      gunRadius: 12,
    },
    // Optional CAD meshes (STL/GLB) per body, exported in the CAD assembly frame at the home
    // configuration. `cadToBase` maps CAD assembly coordinates to the robot base frame.
    cad: {
      cadToBase: { p: [0, 0, 0], R: I3() },
      parts: {
        // base: 'models/stormram1/base.stl',
        // platform: 'models/stormram1/platform.stl',
        // motor1..motor6, rack1..rack6, needle: ...
      },
    },
  };
})();

export function createStormram1() {
  return new HexapodNeedleRobot(stormram1Config);
}
