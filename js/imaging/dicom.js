// DICOM series loader (uncompressed transfer syntaxes; single-frame series and enhanced
// multi-frame MR/CT). Uses dicomParser, loaded as a global by index.html.

import { Volume } from './volume.js';
import { cross, dot, sub, scale, norm, normalize } from '../core/la.js';

const UNCOMPRESSED = new Set(['1.2.840.10008.1.2', '1.2.840.10008.1.2.1', '1.2.840.10008.1.2.2']);
const BIG_ENDIAN = '1.2.840.10008.1.2.2';

const nums = (ds, tag) => {
  const s = ds.string(tag);
  return s ? s.split('\\').map(Number) : null;
};
const num = (ds, tag, def) => {
  const v = ds.floatString(tag);
  return v === undefined || Number.isNaN(v) ? def : v;
};

/** Recursively collect File objects from a drag-and-drop DataTransfer (folders allowed). */
export async function filesFromDataTransfer(dt) {
  const entries = [...dt.items].map((it) => it.webkitGetAsEntry?.()).filter(Boolean);
  if (!entries.length) return [...dt.files];
  const out = [];
  const walk = async (entry) => {
    if (entry.isFile) out.push(await new Promise((res, rej) => entry.file(res, rej)));
    else if (entry.isDirectory) {
      const reader = entry.createReader();
      let batch;
      do {
        batch = await new Promise((res, rej) => reader.readEntries(res, rej));
        for (const e of batch) await walk(e);
      } while (batch.length);
    }
  };
  for (const e of entries) await walk(e);
  return out;
}

function readPixels(ds, byteArray, frameIndex, rows, cols, tsBig) {
  const el = ds.elements.x7fe00010;
  const bits = ds.uint16('x00280100');
  const signed = ds.uint16('x00280103') === 1;
  const spp = ds.uint16('x00280002') || 1;
  if (spp !== 1) throw new Error('Colour (multi-sample) images are not supported');
  const n = rows * cols;
  const bytes = bits / 8;
  const offset = el.dataOffset + frameIndex * n * bytes;
  const dv = new DataView(byteArray.buffer, byteArray.byteOffset + offset, n * bytes);
  const out = new Float32Array(n);
  const le = !tsBig;
  for (let p = 0; p < n; p++) {
    if (bits === 16) out[p] = signed ? dv.getInt16(p * 2, le) : dv.getUint16(p * 2, le);
    else if (bits === 8) out[p] = signed ? dv.getInt8(p) : dv.getUint8(p);
    else if (bits === 32) out[p] = signed ? dv.getInt32(p * 4, le) : dv.getUint32(p * 4, le);
    else throw new Error(`Unsupported BitsAllocated ${bits}`);
  }
  return out;
}

/** Parse one file into zero or more slices {series, ipp, iop, spacing, rows, cols, pixels(), ...}. */
function parseFile(byteArray, fileName) {
  const dp = globalThis.dicomParser;
  if (!dp) throw new Error('dicomParser not loaded');
  const ds = dp.parseDicom(byteArray);
  if (!ds.elements.x7fe00010) return [];
  const ts = ds.string('x00020010') || '1.2.840.10008.1.2';
  if (!UNCOMPRESSED.has(ts)) throw new Error(`${fileName}: compressed transfer syntax ${ts} is not supported (decompress e.g. with "gdcmconv --raw")`);
  const tsBig = ts === BIG_ENDIAN;
  const rows = ds.uint16('x00280010'), cols = ds.uint16('x00280011');
  const frames = parseInt(ds.string('x00280008') || '1', 10);
  const common = {
    series: ds.string('x0020000e') || 'unknown',
    seriesDescription: ds.string('x0008103e') || '',
    seriesNumber: ds.string('x00200011') || '',
    modality: ds.string('x00080060') || '',
    patientPosition: ds.string('x00185100') || '',
    rows, cols,
  };
  const slices = [];
  if (frames > 1 && ds.elements.x52009230) {
    // Enhanced multi-frame: geometry in shared / per-frame functional groups.
    const shared = ds.elements.x52009229?.items?.[0]?.dataSet;
    const per = ds.elements.x52009230.items;
    const grp = (fg, seqTag) => fg?.elements[seqTag]?.items?.[0]?.dataSet;
    for (let f = 0; f < frames; f++) {
      const pf = per[f]?.dataSet;
      const pick = (seqTag) => grp(pf, seqTag) || grp(shared, seqTag);
      const pos = pick('x00209113'), ori = pick('x00209116'), meas = pick('x00289110'), pvt = pick('x00289145');
      if (!pos || !ori || !meas) throw new Error(`${fileName}: missing plane geometry in functional groups`);
      slices.push({
        ...common,
        ipp: nums(pos, 'x00200032'), iop: nums(ori, 'x00200037'), spacing: nums(meas, 'x00280030'),
        slope: pvt ? num(pvt, 'x00281053', 1) : 1, intercept: pvt ? num(pvt, 'x00281052', 0) : 0,
        pixels: () => readPixels(ds, byteArray, f, rows, cols, tsBig),
      });
    }
  } else {
    const ipp = nums(ds, 'x00200032'), iop = nums(ds, 'x00200037');
    const spacing = nums(ds, 'x00280030') || nums(ds, 'x00181164') || [1, 1];
    if (!ipp || !iop) throw new Error(`${fileName}: no ImagePositionPatient / ImageOrientationPatient`);
    for (let f = 0; f < frames; f++) {
      // Legacy multi-frame without functional groups: assume contiguous frames along the normal.
      const nrm = cross(iop.slice(0, 3), iop.slice(3, 6));
      const dz = num(ds, 'x00180088', num(ds, 'x00180050', 1));
      slices.push({
        ...common,
        ipp: [0, 1, 2].map((a) => ipp[a] + nrm[a] * dz * f), iop, spacing,
        slope: num(ds, 'x00281053', 1), intercept: num(ds, 'x00281052', 0),
        pixels: () => readPixels(ds, byteArray, f, rows, cols, tsBig),
      });
    }
  }
  return slices;
}

/**
 * Read files and group slices by series.
 * @returns {Promise<{series: Array<{uid, description, count, build: () => Volume}>, errors: string[]}>}
 */
export async function readDicomFiles(files, onProgress) {
  const all = [], errors = [];
  let done = 0;
  for (const file of files) {
    try {
      const buf = new Uint8Array(await file.arrayBuffer());
      // Quick reject of non-DICOM files (no 'DICM' preamble and not plausibly implicit VR).
      const hasMagic = buf.length > 132 && buf[128] === 68 && buf[129] === 73 && buf[130] === 67 && buf[131] === 77;
      if (hasMagic || /\.dcm$/i.test(file.name) || !/\.[a-z]{2,4}$/i.test(file.name)) {
        all.push(...parseFile(buf, file.name));
      }
    } catch (e) {
      errors.push(String(e.message || e));
    }
    if (onProgress && ++done % 20 === 0) onProgress(done, files.length);
  }
  const groups = new Map();
  for (const s of all) {
    const key = `${s.series}|${s.rows}x${s.cols}|${s.iop.map((v) => v.toFixed(3)).join(',')}`;
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push(s);
  }
  const series = [...groups.entries()]
    .map(([uid, slices]) => ({
      uid,
      description: `${slices[0].seriesNumber ? `#${slices[0].seriesNumber} ` : ''}${slices[0].seriesDescription || slices[0].modality} (${slices.length} slices, ${slices[0].cols}×${slices[0].rows})`,
      count: slices.length,
      build: () => buildVolume(slices),
    }))
    .sort((a, b) => b.count - a.count);
  return { series, errors };
}

function buildVolume(slices) {
  const iop = slices[0].iop;
  const rowDir = normalize(iop.slice(0, 3)); // direction of increasing column index i
  const colDir = normalize(iop.slice(3, 6)); // direction of increasing row index j
  const nrm = normalize(cross(rowDir, colDir));
  slices = [...slices].sort((a, b) => dot(a.ipp, nrm) - dot(b.ipp, nrm));
  // Drop duplicate positions (e.g. repeated acquisitions).
  slices = slices.filter((s, i) => i === 0 || Math.abs(dot(sub(s.ipp, slices[i - 1].ipp), nrm)) > 1e-3);
  const { rows, cols } = slices[0];
  const [rowSpacing, colSpacing] = slices[0].spacing;
  const nz = slices.length;
  let sliceStep = scale(nrm, 1);
  if (nz > 1) {
    sliceStep = scale(sub(slices[nz - 1].ipp, slices[0].ipp), 1 / (nz - 1));
    const gaps = slices.slice(1).map((s, i) => norm(sub(s.ipp, slices[i].ipp)));
    const mean = norm(sliceStep);
    if (gaps.some((g) => Math.abs(g - mean) > 0.05 * mean + 0.01)) console.warn('Non-uniform slice spacing; using mean spacing', gaps);
  }
  const data = new Float32Array(rows * cols * nz);
  slices.forEach((s, k) => {
    const px = s.pixels();
    const off = k * rows * cols;
    for (let p = 0; p < px.length; p++) data[off + p] = px[p] * s.slope + s.intercept;
  });
  return new Volume({
    dims: [cols, rows, nz],
    data,
    origin: slices[0].ipp,
    axes: [scale(rowDir, colSpacing), scale(colDir, rowSpacing), sliceStep],
    name: slices[0].seriesDescription || 'DICOM series',
    meta: { patientPosition: slices[0].patientPosition, modality: slices[0].modality },
  });
}
