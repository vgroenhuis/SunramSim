// A hexapod (Gough-Stewart) positioning stage carrying a needle-insertion axis.
// Joint vector q = [L1..L6, d]: six leg lengths (mm) and needle insertion depth d (mm).
// Frames: base frame B (robot base), platform frame P, needle frame N rigidly attached to P whose
// origin is the needle tip at d = 0 and whose direction is the insertion axis.

import {
  add, sub, scale, dot, norm, normalize, mulMV, mulMM, rotBetween, axisAngle, DEG,
} from '../core/la.js';
import { StewartPlatform } from './stewart.js';

export class HexapodNeedleRobot {
  constructor(cfg) {
    this.cfg = cfg;
    this.kin = new StewartPlatform(cfg.hexapod);
    this.homePlatform = { p: [...cfg.homePlatform.p], R: cfg.homePlatform.R.map((r) => [...r]) };
    this.needle = {
      origin: cfg.needle.origin,          // tip at d = 0, platform frame
      dir: normalize(cfg.needle.dir),     // insertion direction, platform frame
      min: cfg.needle.min ?? 0,
      max: cfg.needle.max,
    };
    const legStep = cfg.motors.legStep, insStep = cfg.motors.insertionStep;
    this.joints = [
      ...this.kin.legMin.map((mn, i) => ({
        name: `Leg ${i + 1}`, unit: 'mm', min: mn, max: this.kin.legMax[i], step: legStep,
      })),
      { name: 'Insertion', unit: 'mm', min: this.needle.min, max: this.needle.max, step: insStep },
    ];
    this.stepFrequency = cfg.motors.stepFrequency; // Hz per motor
  }

  get dof() { return this.joints.length; }

  homeJoints() {
    const { lengths } = this.kin.ik(this.homePlatform);
    return [...lengths, this.needle.min];
  }

  /** Needle tip and direction (base frame) for a platform pose and insertion depth. */
  needleInBase(platform, d) {
    const dir = mulMV(platform.R, this.needle.dir);
    const tip0 = add(platform.p, mulMV(platform.R, this.needle.origin));
    return { tip0, tip: add(tip0, scale(dir, d)), dir };
  }

  /** Forward kinematics. `guess` is the previous platform pose (Newton seed). */
  fk(q, guess = this.homePlatform) {
    const res = this.kin.fk(q.slice(0, 6), guess);
    const needle = this.needleInBase(res.pose, q[6]);
    return { platform: res.pose, ...needle, ok: res.ok, residual: res.residual };
  }

  /** Inverse kinematics for a platform pose and insertion depth. */
  ikPlatform(platform, d) {
    const r = this.kin.ik(platform);
    const violations = [...r.violations];
    if (d < this.needle.min - 1e-9 || d > this.needle.max + 1e-9) {
      violations.push({ leg: 6, type: 'insertion-range', value: d });
    }
    return {
      q: [...r.lengths, d], ok: violations.length === 0, violations, margin: r.margin,
      legDirs: r.dirs, platformJoints: r.joints,
    };
  }

  /** Platform pose that puts the needle tip (at depth d) at `tip` with platform orientation R. */
  platformFromTip(tip, R, d) {
    const dir = mulMV(R, this.needle.dir);
    const tip0 = sub(tip, scale(dir, d));
    return { p: sub(tip0, mulMV(R, this.needle.origin)), R };
  }

  /**
   * Orientation that points the needle along `dir` (base frame): the smallest rotation from the
   * home orientation, followed by a roll `roll` (rad) about the needle axis.
   */
  orientationForDirection(dir, roll = 0) {
    const homeDir = mulMV(this.homePlatform.R, this.needle.dir);
    const Ralign = mulMM(rotBetween(homeDir, dir), this.homePlatform.R);
    return roll ? mulMM(axisAngle(dir, roll), Ralign) : Ralign;
  }

  /**
   * Plan a needle insertion: the needle must travel along the line through `target` with
   * direction `dir` and end with its tip at `target` (all in base frame).
   * The free parameters are the insertion depth used to reach the target and the roll about the
   * needle axis; they are chosen to maximise the worst-case joint margin (or depth is fixed).
   * @param {object} opt
   * @param {number|null} opt.depth  fixed insertion depth, or null to prefer the deepest feasible
   * @param {number} opt.rollRange   search ±range (deg) for roll about the needle axis
   */
  planNeedle(target, dir, { depth = null, rollRange = 30 } = {}) {
    dir = normalize(dir);
    const depths = [];
    if (depth != null) depths.push(depth);
    else for (let d = this.needle.max; d >= this.needle.min - 1e-9; d -= (this.needle.max - this.needle.min) / 28) depths.push(d);
    const rolls = [0];
    for (let r = 5; r <= rollRange; r += 5) rolls.push(r * DEG, -r * DEG);

    let best = null;
    for (const d of depths) {
      for (const roll of rolls) {
        const R = this.orientationForDirection(dir, roll);
        const platform = this.platformFromTip(target, R, d);
        const ik = this.ikPlatform(platform, d);
        // Prefer feasible, then deeper insertion (platform stays further from the patient), then margin.
        const score = (ik.ok ? 1000 : 0) + ik.margin * 10 + (depth == null ? d / this.needle.max : 0) - Math.abs(roll);
        if (!best || score > best.score) best = { score, q: ik.q, platform, depth: d, roll, ...ik };
      }
    }
    // Retracted configuration: same platform pose, needle at minimum depth.
    best.qRetracted = [...best.q.slice(0, 6), this.needle.min];
    best.dir = dir;
    best.target = target;
    return best;
  }

  /** Round each joint to its motor step grid (relative to the home configuration). */
  quantize(q, home = this.homeJoints()) {
    return q.map((v, i) => {
      const s = this.joints[i].step;
      return s ? home[i] + Math.round((v - home[i]) / s) * s : v;
    });
  }

  /** Distance from point x to the infinite line through `p` with unit direction `u`. */
  static pointLineDistance(x, p, u) {
    const v = sub(x, p);
    return norm(sub(v, scale(u, dot(v, u))));
  }
}
