// Run with: node --test tests/*.test.mjs
import test from 'node:test';
import assert from 'node:assert/strict';
import { createStormram1 } from '../js/robots/stormram1.js';
import { fromRPY, mulMM, dist, DEG, norm, sub, dot } from '../js/core/la.js';
import { HexapodNeedleRobot } from '../js/robots/hexapodRobot.js';

const robot = createStormram1();

test('home configuration is feasible', () => {
  const ik = robot.ikPlatform(robot.homePlatform, 0);
  assert.ok(ik.ok, JSON.stringify(ik.violations));
  assert.ok(ik.margin > 0.2, `margin ${ik.margin}`);
});

test('FK inverts IK over random poses', () => {
  let tested = 0;
  for (let n = 0; n < 200; n++) {
    const R = mulMM(fromRPY((Math.random() - 0.5) * 20 * DEG, (Math.random() - 0.5) * 20 * DEG, (Math.random() - 0.5) * 30 * DEG), robot.homePlatform.R);
    const p = robot.homePlatform.p.map((v) => v + (Math.random() - 0.5) * 60);
    const ik = robot.ikPlatform({ p, R }, 10);
    if (!ik.ok) continue;
    tested++;
    const fk = robot.fk(ik.q, robot.homePlatform);
    assert.ok(fk.ok);
    assert.ok(dist(fk.platform.p, p) < 1e-6);
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) assert.ok(Math.abs(fk.platform.R[i][j] - R[i][j]) < 1e-8);
  }
  assert.ok(tested > 20, `only ${tested} feasible samples`);
});

test('needle planning reaches the target along the requested direction', () => {
  const home = robot.fk(robot.homeJoints());
  const target = [home.tip0[0] + 40, home.tip0[1] + 15, home.tip0[2] - 10];
  const dir = [Math.cos(10 * DEG), Math.sin(10 * DEG), -0.1];
  const plan = robot.planNeedle(target, dir);
  assert.ok(plan.ok, JSON.stringify(plan.violations));
  const fk = robot.fk(plan.q, robot.homePlatform);
  assert.ok(dist(fk.tip, target) < 1e-6);
  assert.ok(dot(fk.dir, plan.dir) > 1 - 1e-9);
  // Quantised joints give a small but non-zero error.
  const fq = robot.fk(robot.quantize(plan.q), plan.platform);
  const err = HexapodNeedleRobot.pointLineDistance(target, fq.tip, fq.dir);
  assert.ok(err < 1, `quantisation error ${err}`);
  assert.ok(norm(sub(fq.tip, target)) < 2);
});
