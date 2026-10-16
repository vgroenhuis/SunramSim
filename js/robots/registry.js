// Robots known to the simulator. Each available entry provides `create()` returning
// { robot, Visual } where robot exposes the kinematic interface used by the app:
//   joints[], homeJoints(), fk(q, guess), ikPlatform(pose, d), platformFromTip(tip, R, d),
//   planNeedle(target, dir, opts), quantize(q), needle {dir, min, max}, homePlatform, cfg.biopsy

import { createStormram1 } from './stormram1.js';
import { HexapodVisual } from './hexapodVisual.js';

export const ROBOTS = [
  { id: 'stormram1', name: 'Stormram 1', kind: 'parallel (hexapod + insertion)', create: () => ({ robot: createStormram1(), Visual: HexapodVisual }) },
  { id: 'stormram2', name: 'Stormram 2', kind: 'parallel', create: null },
  { id: 'stormram3', name: 'Stormram 3', kind: 'parallel', create: null },
  { id: 'stormram4', name: 'Stormram 4', kind: 'serial', create: null },
  { id: 'sunram5', name: 'Sunram 5', kind: 'serial', create: null },
  { id: 'sunram7', name: 'Sunram 7', kind: 'serial', create: null },
];
