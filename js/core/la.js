// Minimal 3D linear algebra on plain arrays, so kinematics run in both browser and Node.
// Vectors: [x, y, z]. Matrices: row-major 3x3 as [[r00,r01,r02],[r10,..],[r20,..]].
// Poses: { p: vec3, R: mat3 } mapping child coordinates to parent: x_parent = R x_child + p.

export const DEG = Math.PI / 180;

export const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
export const norm = (a) => Math.hypot(a[0], a[1], a[2]);
export const dist = (a, b) => norm(sub(a, b));
export const normalize = (a) => {
  const n = norm(a);
  return n > 1e-12 ? scale(a, 1 / n) : [0, 0, 0];
};
export const lerp = (a, b, t) => add(a, scale(sub(b, a), t));

export const I3 = () => [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
export const mulMV = (M, v) => [dot(M[0], v), dot(M[1], v), dot(M[2], v)];
export const transpose = (M) => [
  [M[0][0], M[1][0], M[2][0]],
  [M[0][1], M[1][1], M[2][1]],
  [M[0][2], M[1][2], M[2][2]],
];
export const mulMM = (A, B) => {
  const Bt = transpose(B);
  return A.map((row) => Bt.map((col) => dot(row, col)));
};

export const rotX = (a) => { const c = Math.cos(a), s = Math.sin(a); return [[1, 0, 0], [0, c, -s], [0, s, c]]; };
export const rotY = (a) => { const c = Math.cos(a), s = Math.sin(a); return [[c, 0, s], [0, 1, 0], [-s, 0, c]]; };
export const rotZ = (a) => { const c = Math.cos(a), s = Math.sin(a); return [[c, -s, 0], [s, c, 0], [0, 0, 1]]; };

/** Fixed-axis roll/pitch/yaw (radians): R = Rz(yaw) Ry(pitch) Rx(roll). */
export const fromRPY = (roll, pitch, yaw) => mulMM(rotZ(yaw), mulMM(rotY(pitch), rotX(roll)));
export const toRPY = (R) => {
  const pitch = Math.asin(Math.max(-1, Math.min(1, -R[2][0])));
  if (Math.abs(Math.cos(pitch)) < 1e-9) return [Math.atan2(-R[1][2], R[1][1]), pitch, 0];
  return [Math.atan2(R[2][1], R[2][2]), pitch, Math.atan2(R[1][0], R[0][0])];
};

/** Rodrigues: rotation of angle |w| about w. */
export const expSO3 = (w) => {
  const th = norm(w);
  if (th < 1e-12) return I3();
  const k = scale(w, 1 / th), c = Math.cos(th), s = Math.sin(th), t = 1 - c;
  const [x, y, z] = k;
  return [
    [c + x * x * t, x * y * t - z * s, x * z * t + y * s],
    [y * x * t + z * s, c + y * y * t, y * z * t - x * s],
    [z * x * t - y * s, z * y * t + x * s, c + z * z * t],
  ];
};
export const axisAngle = (axis, angle) => expSO3(scale(normalize(axis), angle));

/** Smallest rotation taking unit vector a onto unit vector b. */
export const rotBetween = (a, b) => {
  a = normalize(a); b = normalize(b);
  const c = dot(a, b);
  if (c > 1 - 1e-12) return I3();
  if (c < -1 + 1e-12) {
    const perp = Math.abs(a[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
    return axisAngle(cross(a, perp), Math.PI);
  }
  return axisAngle(cross(a, b), Math.acos(c));
};

/** Re-orthonormalise a rotation matrix (Gram-Schmidt on columns). */
export const orthonormalize = (R) => {
  const Rt = transpose(R);
  const x = normalize(Rt[0]);
  const y = normalize(sub(Rt[1], scale(x, dot(x, Rt[1]))));
  const z = cross(x, y);
  return transpose([x, y, z]);
};

export const pose = (p = [0, 0, 0], R = I3()) => ({ p: [...p], R: R.map((r) => [...r]) });
export const applyPose = (T, x) => add(mulMV(T.R, x), T.p);
export const applyRot = (T, v) => mulMV(T.R, v);
export const composePose = (A, B) => ({ p: applyPose(A, B.p), R: mulMM(A.R, B.R) });
export const invertPose = (T) => {
  const Rt = transpose(T.R);
  return { p: scale(mulMV(Rt, T.p), -1), R: Rt };
};

/** Solve A x = b for small dense systems (Gaussian elimination, partial pivoting). Returns null if singular. */
export function solve(A, b) {
  const n = b.length;
  const M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let piv = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
    if (Math.abs(M[piv][c]) < 1e-12) return null;
    [M[c], M[piv]] = [M[piv], M[c]];
    for (let r = c + 1; r < n; r++) {
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  const x = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    let s = M[r][n];
    for (let k = r + 1; k < n; k++) s -= M[r][k] * x[k];
    x[r] = s / M[r][r];
  }
  return x;
}

/** Column-major 4x4 array (three.js Matrix4.fromArray order) for a pose. */
export const poseToMatrix4Array = (T) => [
  T.R[0][0], T.R[1][0], T.R[2][0], 0,
  T.R[0][1], T.R[1][1], T.R[2][1], 0,
  T.R[0][2], T.R[1][2], T.R[2][2], 0,
  T.p[0], T.p[1], T.p[2], 1,
];
