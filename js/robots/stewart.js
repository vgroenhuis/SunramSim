// Generic Gough-Stewart (6-UPS / 6-SPS) platform kinematics.
// Each leg i connects base joint b_i (base frame) to platform joint a_i (platform frame).
// The actuated variable is the leg length L_i = |p + R a_i - b_i|.

import {
  add, sub, scale, dot, cross, norm, normalize, mulMV, mulMM, expSO3, orthonormalize, solve, DEG,
} from '../core/la.js';

export class StewartPlatform {
  /**
   * @param {object} cfg
   * @param {number[][]} cfg.baseJoints      6 base joint centres, base frame (mm)
   * @param {number[][]} cfg.platformJoints  6 platform joint centres, platform frame (mm)
   * @param {number[]|number} cfg.legMin     minimum leg length(s) (mm)
   * @param {number[]|number} cfg.legMax     maximum leg length(s) (mm)
   * @param {number[][]} [cfg.baseJointAxes]     cone axis per base joint (base frame), default +z
   * @param {number[][]} [cfg.platformJointAxes] cone axis per platform joint (platform frame), default -z
   * @param {number} [cfg.baseJointMaxAngle]     max angle between leg and base cone axis (deg)
   * @param {number} [cfg.platformJointMaxAngle] max angle between reversed leg and platform cone axis (deg)
   */
  constructor(cfg) {
    this.b = cfg.baseJoints;
    this.a = cfg.platformJoints;
    this.n = this.b.length;
    const arr = (v) => (Array.isArray(v) ? v : new Array(this.n).fill(v));
    this.legMin = arr(cfg.legMin);
    this.legMax = arr(cfg.legMax);
    this.baseAxes = (cfg.baseJointAxes || this.b.map(() => [0, 0, 1])).map(normalize);
    this.platAxes = (cfg.platformJointAxes || this.a.map(() => [0, 0, -1])).map(normalize);
    this.baseMax = (cfg.baseJointMaxAngle ?? 90) * DEG;
    this.platMax = (cfg.platformJointMaxAngle ?? 90) * DEG;
  }

  /** World (base-frame) platform joint positions for a platform pose. */
  platformJointsInBase(pose) {
    return this.a.map((ai) => add(pose.p, mulMV(pose.R, ai)));
  }

  /** Inverse kinematics: leg lengths plus a feasibility report. */
  ik(pose) {
    const P = this.platformJointsInBase(pose);
    const lengths = [], dirs = [], violations = [];
    let margin = Infinity;
    for (let i = 0; i < this.n; i++) {
      const v = sub(P[i], this.b[i]);
      const L = norm(v);
      const d = scale(v, 1 / L);
      lengths.push(L);
      dirs.push(d);
      const range = this.legMax[i] - this.legMin[i];
      const mLen = Math.min(L - this.legMin[i], this.legMax[i] - L) / (0.5 * range);
      if (L < this.legMin[i] - 1e-6) violations.push({ leg: i, type: 'length-min', value: L });
      if (L > this.legMax[i] + 1e-6) violations.push({ leg: i, type: 'length-max', value: L });
      const angB = Math.acos(Math.max(-1, Math.min(1, dot(d, this.baseAxes[i]))));
      const platAxisBase = mulMV(pose.R, this.platAxes[i]);
      const angP = Math.acos(Math.max(-1, Math.min(1, dot(scale(d, -1), platAxisBase))));
      if (angB > this.baseMax + 1e-9) violations.push({ leg: i, type: 'base-joint-angle', value: angB / DEG });
      if (angP > this.platMax + 1e-9) violations.push({ leg: i, type: 'platform-joint-angle', value: angP / DEG });
      const mAng = Math.min(1 - angB / this.baseMax, 1 - angP / this.platMax);
      margin = Math.min(margin, mLen, mAng);
    }
    return { lengths, dirs, joints: P, ok: violations.length === 0, violations, margin };
  }

  /** Analytic Jacobian dL/d[p, w] (6x6) where w is a small rotation applied as R <- exp(w) R. */
  jacobian(pose) {
    const P = this.platformJointsInBase(pose);
    return P.map((Pi, i) => {
      const d = normalize(sub(Pi, this.b[i]));
      const r = sub(Pi, pose.p);
      return [...d, ...cross(r, d)];
    });
  }

  /**
   * Forward kinematics by Newton-Raphson from an initial guess.
   * @returns {{pose, ok, residual, iterations}}
   */
  fk(lengths, guess, { tol = 1e-9, maxIter = 50 } = {}) {
    let p = [...guess.p];
    let R = guess.R.map((r) => [...r]);
    let residual = Infinity, it = 0;
    for (; it < maxIter; it++) {
      const cur = this.ik({ p, R }).lengths;
      const f = cur.map((L, i) => L - lengths[i]);
      residual = Math.max(...f.map(Math.abs));
      if (residual < tol) break;
      const dx = solve(this.jacobian({ p, R }), f.map((x) => -x));
      if (!dx) return { pose: { p, R }, ok: false, residual, iterations: it };
      // Damp large steps to keep Newton in the basin of the current assembly mode.
      const stepNorm = Math.hypot(...dx.slice(0, 3)) + 100 * Math.hypot(...dx.slice(3));
      const k = stepNorm > 20 ? 20 / stepNorm : 1;
      p = add(p, scale(dx.slice(0, 3), k));
      R = orthonormalize(mulMM(expSO3(scale(dx.slice(3), k)), R));
    }
    return { pose: { p, R }, ok: residual < 1e-6, residual, iterations: it };
  }
}
