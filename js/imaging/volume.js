// A scalar image volume with an affine voxel-to-patient (DICOM LPS, mm) transform.

import { add, mulMV, transpose, solve } from '../core/la.js';

export class Volume {
  /**
   * @param {object} o
   * @param {number[]} o.dims       [nx, ny, nz] (i = column, j = row, k = slice)
   * @param {Float32Array} o.data   voxel values, index = i + nx * (j + ny * k)
   * @param {number[]} o.origin     LPS position of voxel (0,0,0) centre
   * @param {number[][]} o.axes     LPS step vectors for +i, +j, +k (mm, include spacing)
   */
  constructor({ dims, data, origin, axes, name = 'volume', meta = {} }) {
    this.dims = dims;
    this.data = data;
    this.origin = origin;
    this.axes = axes;
    this.name = name;
    this.meta = meta;
    // M maps ijk -> LPS offset; columns are the axis vectors.
    this.M = transpose(axes);
    this.Minv = invert3(this.M);
    this.spacing = axes.map((a) => Math.hypot(...a));
    let mn = Infinity, mx = -Infinity;
    for (let n = 0; n < data.length; n++) { const v = data[n]; if (v < mn) mn = v; if (v > mx) mx = v; }
    this.min = mn; this.max = mx;
    this.autoWindow = this.computeAutoWindow();
  }

  ijkToLps(ijk) { return add(this.origin, mulMV(this.M, ijk)); }
  lpsToIjk(x) { return mulMV(this.Minv, [x[0] - this.origin[0], x[1] - this.origin[1], x[2] - this.origin[2]]); }

  get center() { return this.ijkToLps(this.dims.map((d) => (d - 1) / 2)); }

  /** LPS corners of the voxel-centre bounding box. */
  corners() {
    const [nx, ny, nz] = this.dims, out = [];
    for (const i of [0, nx - 1]) for (const j of [0, ny - 1]) for (const k of [0, nz - 1]) out.push(this.ijkToLps([i, j, k]));
    return out;
  }

  /** Axis-aligned LPS bounds [[min], [max]]. */
  bounds() {
    const c = this.corners();
    return [0, 1].map((m) => [0, 1, 2].map((a) => (m ? Math.max : Math.min)(...c.map((p) => p[a]))));
  }

  /** Trilinear sample at continuous voxel coordinates; NaN outside. */
  sampleIjk(i, j, k) {
    const [nx, ny, nz] = this.dims;
    if (i < -0.5 || j < -0.5 || k < -0.5 || i > nx - 0.5 || j > ny - 0.5 || k > nz - 0.5) return NaN;
    i = Math.min(Math.max(i, 0), nx - 1); j = Math.min(Math.max(j, 0), ny - 1); k = Math.min(Math.max(k, 0), nz - 1);
    const i0 = Math.min(Math.floor(i), nx - 2 < 0 ? 0 : nx - 2), j0 = Math.min(Math.floor(j), ny - 2 < 0 ? 0 : ny - 2), k0 = Math.min(Math.floor(k), nz - 2 < 0 ? 0 : nz - 2);
    const fi = nx > 1 ? i - i0 : 0, fj = ny > 1 ? j - j0 : 0, fk = nz > 1 ? k - k0 : 0;
    const di = nx > 1 ? 1 : 0, dj = ny > 1 ? nx : 0, dk = nz > 1 ? nx * ny : 0;
    const d = this.data, b = i0 + nx * (j0 + ny * k0);
    const c00 = d[b] * (1 - fi) + d[b + di] * fi;
    const c10 = d[b + dj] * (1 - fi) + d[b + dj + di] * fi;
    const c01 = d[b + dk] * (1 - fi) + d[b + dk + di] * fi;
    const c11 = d[b + dk + dj] * (1 - fi) + d[b + dk + dj + di] * fi;
    return (c00 * (1 - fj) + c10 * fj) * (1 - fk) + (c01 * (1 - fj) + c11 * fj) * fk;
  }

  sample(x) { const [i, j, k] = this.lpsToIjk(x); return this.sampleIjk(i, j, k); }

  /** Window from the 1st-99.5th percentile of a subsample of voxel values. */
  computeAutoWindow() {
    const n = this.data.length, step = Math.max(1, Math.floor(n / 200000)), vals = [];
    for (let s = 0; s < n; s += step) vals.push(this.data[s]);
    vals.sort((a, b) => a - b);
    const lo = vals[Math.floor(vals.length * 0.01)], hi = vals[Math.floor(vals.length * 0.995)];
    const width = Math.max(hi - lo, 1e-6);
    return { center: lo + width / 2, width };
  }
}

function invert3(M) {
  const cols = [0, 1, 2].map((c) => solve(M, [0, 1, 2].map((r) => (r === c ? 1 : 0))));
  if (cols.some((c) => !c)) throw new Error('Singular voxel-to-patient transform');
  return transpose(cols);
}

