// Synthetic prone breast phantom (fat-suppressed, contrast-enhanced T1-like contrast) so the
// simulator is usable without patient data. Coordinates are DICOM LPS (mm); the patient lies prone
// (head first), so posterior (+Y) points up in the room and the breast hangs towards -Y.

import { Volume } from './volume.js';

function hash3(x, y, z, seed) {
  let h = (x * 374761393 + y * 668265263 + z * 2147483647 + seed * 144665) | 0;
  h = Math.imul(h ^ (h >>> 13), 1274126177);
  return ((h ^ (h >>> 16)) >>> 0) / 4294967295;
}

function valueNoise(x, y, z, seed) {
  const xi = Math.floor(x), yi = Math.floor(y), zi = Math.floor(z);
  const fx = x - xi, fy = y - yi, fz = z - zi;
  const sx = fx * fx * (3 - 2 * fx), sy = fy * fy * (3 - 2 * fy), sz = fz * fz * (3 - 2 * fz);
  let v = 0;
  for (let c = 0; c < 8; c++) {
    const dx = c & 1, dy = (c >> 1) & 1, dz = (c >> 2) & 1;
    const w = (dx ? sx : 1 - sx) * (dy ? sy : 1 - sy) * (dz ? sz : 1 - sz);
    v += w * hash3(xi + dx, yi + dy, zi + dz, seed);
  }
  return v;
}

export const PHANTOM_LESIONS = [
  { name: 'Lesion A (8 mm)', center: [12, -28, 8], radius: 4 },
  { name: 'Lesion B (12 mm)', center: [-22, -50, -14], radius: 6 },
  { name: 'Lesion C (16 mm)', center: [26, -8, -24], radius: 8 },
];

export function createBreastPhantom({ spacing = 1.25 } = {}) {
  const ext = { x: [-110, 110], y: [-100, 70], z: [-80, 80] };
  const dims = [ext.x, ext.y, ext.z].map(([a, b]) => Math.round((b - a) / spacing) + 1);
  const [nx, ny, nz] = dims;
  const data = new Float32Array(nx * ny * nz);
  const chest = 28; // chest wall (Y) where the breast attaches
  const R = [68, 98, 62]; // breast semi-axes (x, depth along -y, z)
  let n = 0;
  for (let k = 0; k < nz; k++) {
    const z = ext.z[0] + k * spacing;
    for (let j = 0; j < ny; j++) {
      const y = ext.y[0] + j * spacing;
      for (let i = 0; i < nx; i++, n++) {
        const x = ext.x[0] + i * spacing;
        let v = 0;
        if (y >= chest) {
          // Thorax: muscle layer then darker lung region.
          const body = Math.abs(x) < 105;
          if (body) v = y < chest + 14 ? 330 : (y < chest + 22 ? 120 : 60);
        } else {
          const e = (x / R[0]) ** 2 + ((chest - y) / R[1]) ** 2 + (z / R[2]) ** 2;
          if (e < 1) {
            const depth = 1 - Math.sqrt(e);
            if (depth < 0.035) v = 430; // skin
            else {
              // Fibroglandular tissue concentrated around the central ducts, in fat (suppressed).
              const radial = Math.sqrt((x / R[0]) ** 2 + (z / R[2]) ** 2);
              const g = valueNoise(x / 9, y / 9, z / 9, 1) * 0.65 + valueNoise(x / 4, y / 4, z / 4, 2) * 0.35;
              const t = g + 0.45 * (1 - radial) - 0.25 * (y < chest - 70 ? 1 : 0);
              const w = Math.min(1, Math.max(0, (t - 0.62) / 0.2));
              v = 140 + 250 * w * w * (3 - 2 * w) + 40 * g;
            }
          }
        }
        for (const L of PHANTOM_LESIONS) {
          const d = Math.hypot(x - L.center[0], y - L.center[1], z - L.center[2]);
          if (d < L.radius + 1.5) {
            const t = Math.min(1, Math.max(0, (L.radius + 1.5 - d) / 3));
            v = v * (1 - t) + (1050 - 150 * (d / L.radius)) * t;
          }
        }
        data[n] = Math.max(0, v + (hash3(i, j, k, 7) - 0.5) * 50);
      }
    }
  }
  return new Volume({
    dims,
    data,
    origin: [ext.x[0], ext.y[0], ext.z[0]],
    axes: [[spacing, 0, 0], [0, spacing, 0], [0, 0, spacing]],
    name: 'Synthetic breast phantom (prone)',
    meta: { patientPosition: 'HFP', modality: 'MR (synthetic)', lesions: PHANTOM_LESIONS },
  });
}
