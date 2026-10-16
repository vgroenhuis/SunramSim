// Writes an oblique, uncompressed (explicit VR little endian) MR series resampled from the
// synthetic phantom, for testing the DICOM loader:  node tests/make-test-dicom.mjs <outDir>
import { mkdirSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { createBreastPhantom } from '../js/imaging/phantom.js';
import { add, scale, rotZ, mulMV, cross } from '../js/core/la.js';

const out = process.argv[2] || 'test-dicom';
mkdirSync(out, { recursive: true });

const LONG = new Set(['OB', 'OW', 'OF', 'SQ', 'UT', 'UN']);
function element(group, elem, vr, value) {
  let bytes;
  if (vr === 'US') { bytes = Buffer.alloc(2); bytes.writeUInt16LE(value); }
  else if (vr === 'UL') { bytes = Buffer.alloc(4); bytes.writeUInt32LE(value); }
  else if (Buffer.isBuffer(value)) bytes = value;
  else {
    bytes = Buffer.from(String(value), 'latin1');
    if (bytes.length % 2) bytes = Buffer.concat([bytes, Buffer.from(vr === 'UI' ? '\0' : ' ')]);
  }
  const head = Buffer.alloc(LONG.has(vr) ? 12 : 8);
  head.writeUInt16LE(group, 0); head.writeUInt16LE(elem, 2); head.write(vr, 4, 'latin1');
  if (LONG.has(vr)) head.writeUInt32LE(bytes.length, 8); else head.writeUInt16LE(bytes.length, 6);
  return Buffer.concat([head, bytes]);
}

const vol = createBreastPhantom({ spacing: 2 });
const R = rotZ(10 * Math.PI / 180); // oblique acquisition
const rowDir = mulMV(R, [1, 0, 0]), colDir = mulMV(R, [0, 1, 0]);
const nrm = cross(rowDir, colDir);
const cols = 110, rows = 86, nSlices = 60, ps = 2, dz = 2.5;
const c = vol.center;
const origin = add(c, add(add(scale(rowDir, -cols / 2 * ps), scale(colDir, -rows / 2 * ps)), scale(nrm, -nSlices / 2 * dz)));
const series = '1.2.826.0.1.3680043.2.1125.99.1';
const ds = (v) => v.map((x) => x.toFixed(4)).join('\\');

for (let k = 0; k < nSlices; k++) {
  const ipp = add(origin, scale(nrm, k * dz));
  const px = Buffer.alloc(rows * cols * 2);
  for (let j = 0; j < rows; j++) for (let i = 0; i < cols; i++) {
    const v = vol.sample(add(ipp, add(scale(rowDir, i * ps), scale(colDir, j * ps))));
    px.writeInt16LE(Math.round((Number.isNaN(v) ? 0 : v) / 2) - 100, 2 * (i + cols * j)); // slope 2, intercept 200
  }
  const sop = `${series}.${k + 1}`;
  const meta = Buffer.concat([
    element(0x0002, 0x0001, 'OB', Buffer.from([0, 1])),
    element(0x0002, 0x0002, 'UI', '1.2.840.10008.5.1.4.1.1.4'),
    element(0x0002, 0x0003, 'UI', sop),
    element(0x0002, 0x0010, 'UI', '1.2.840.10008.1.2.1'),
  ]);
  const body = Buffer.concat([
    element(0x0008, 0x0016, 'UI', '1.2.840.10008.5.1.4.1.1.4'),
    element(0x0008, 0x0018, 'UI', sop),
    element(0x0008, 0x0060, 'CS', 'MR'),
    element(0x0008, 0x103e, 'LO', 'Test oblique phantom'),
    element(0x0018, 0x5100, 'CS', 'HFP'),
    element(0x0020, 0x000e, 'UI', series),
    element(0x0020, 0x0011, 'IS', '3'),
    element(0x0020, 0x0013, 'IS', String(nSlices - k)),
    element(0x0020, 0x0032, 'DS', ds(ipp)),
    element(0x0020, 0x0037, 'DS', ds([...rowDir, ...colDir])),
    element(0x0028, 0x0002, 'US', 1),
    element(0x0028, 0x0004, 'CS', 'MONOCHROME2'),
    element(0x0028, 0x0010, 'US', rows),
    element(0x0028, 0x0011, 'US', cols),
    element(0x0028, 0x0030, 'DS', `${ps}\\${ps}`),
    element(0x0028, 0x0100, 'US', 16),
    element(0x0028, 0x0101, 'US', 16),
    element(0x0028, 0x0102, 'US', 15),
    element(0x0028, 0x0103, 'US', 1),
    element(0x0028, 0x1052, 'DS', '200'),
    element(0x0028, 0x1053, 'DS', '2'),
    element(0x7fe0, 0x0010, 'OW', px),
  ]);
  const metaHead = Buffer.concat([element(0x0002, 0x0000, 'UL', meta.length)]);
  // Shuffle file order to exercise sorting by position.
  writeFileSync(join(out, `IM${String((k * 37) % nSlices).padStart(4, '0')}.dcm`), Buffer.concat([Buffer.alloc(128), Buffer.from('DICM'), metaHead, meta, body]));
}
console.log(`Wrote ${nSlices} slices to ${out}`);
