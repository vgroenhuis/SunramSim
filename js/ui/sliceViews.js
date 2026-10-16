// 2D reformatted slice views (axial / coronal / sagittal / needle-aligned) with overlays.
// All geometry is in DICOM LPS (mm). Screen x follows `ex`, screen y (down) follows `ey`.

import { add, sub, scale, dot, cross, normalize, axisAngle, mulMV, norm } from '../core/la.js';

const RES = 256; // sample grid of the reformatted image

const ORIENTATIONS = {
  axial: { ex: [1, 0, 0], ey: [0, 1, 0], label: 'Axial' },
  coronal: { ex: [1, 0, 0], ey: [0, 0, -1], label: 'Coronal' },
  sagittal: { ex: [0, 1, 0], ey: [0, 0, -1], label: 'Sagittal' },
};

function dirLetters(v) {
  const L = [['L', 'R'], ['P', 'A'], ['S', 'I']];
  const order = [0, 1, 2].sort((a, b) => Math.abs(v[b]) - Math.abs(v[a]));
  let s = '';
  for (const a of order) if (Math.abs(v[a]) > 0.25) s += L[a][v[a] > 0 ? 0 : 1];
  return s;
}

export class SliceView {
  constructor(el, kind, app) {
    this.el = el;
    this.kind = kind;
    this.app = app;
    this.fov = 220;
    this.pan = [0, 0];
    this.obliqueAngle = 0; // rotation of the needle-view plane about the needle axis
    this.offset = 0;       // oblique plane offset along its normal
    el.classList.add('slice-view');
    el.innerHTML = `<canvas class="sv-canvas"></canvas><div class="sv-head"><span class="sv-title"></span><span class="sv-tools"></span></div><div class="sv-info"></div>`;
    this.canvas = el.querySelector('canvas');
    this.ctx = this.canvas.getContext('2d');
    this.title = el.querySelector('.sv-title');
    this.info = el.querySelector('.sv-info');
    this.title.textContent = kind === 'oblique' ? 'Needle view' : ORIENTATIONS[kind].label;
    if (kind === 'oblique') {
      const tools = el.querySelector('.sv-tools');
      tools.innerHTML = '<button title="Rotate plane about needle −15°">⟲</button><button title="Rotate plane about needle +15°">⟳</button>';
      const [a, b] = tools.querySelectorAll('button');
      a.onclick = () => { this.obliqueAngle -= 15; this.app.requestRender(); };
      b.onclick = () => { this.obliqueAngle += 15; this.app.requestRender(); };
    }
    // `image` is shown in 2D; `texture` is the same slice with low intensities made transparent
    // for the 3D scene.
    this.image = document.createElement('canvas');
    this.texture = document.createElement('canvas');
    this.image.width = this.image.height = this.texture.width = this.texture.height = RES;
    this.imageCtx = this.image.getContext('2d');
    this.textureCtx = this.texture.getContext('2d');
    this.imageData = this.imageCtx.createImageData(RES, RES);
    this.textureData = this.textureCtx.createImageData(RES, RES);
    this.version = 0;
    this.bindEvents();
  }

  /** Current plane: center c, in-plane axes ex (screen right), ey (screen down), normal n. */
  plane() {
    const s = this.app.state, vol = s.volume;
    if (this.kind === 'oblique') {
      const line = this.app.needleViewLine();
      const ex = line.dir;
      // Reference: plane containing the needle and the room's vertical, then rotated by the user.
      let ref = normalize(cross(ex, s.roomUp));
      if (norm(ref) < 0.5) ref = normalize(cross(ex, [0, 0, 1]));
      const nrm0 = ref;
      const n = mulMV(axisAngle(ex, this.obliqueAngle * Math.PI / 180), nrm0);
      let ey = normalize(cross(n, ex));
      // Keep "up" on screen pointing roughly to the room's up.
      if (dot(ey, s.roomUp) > 0) ey = scale(ey, -1);
      const nn = cross(ex, ey);
      const c0 = add(line.center, scale(nn, this.offset));
      return { c: add(c0, add(scale(ex, this.pan[0]), scale(ey, this.pan[1]))), ex, ey, n: nn };
    }
    const o = ORIENTATIONS[this.kind];
    const n = cross(o.ex, o.ey);
    const base = vol ? vol.center : [0, 0, 0];
    const cur = s.cursor;
    const c = add(add(base, scale(n, dot(sub(cur, base), n))), add(scale(o.ex, this.pan[0]), scale(o.ey, this.pan[1])));
    return { c, ex: o.ex, ey: o.ey, n };
  }

  layout() {
    const r = this.el.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    const w = Math.max(1, Math.round(r.width * dpr)), h = Math.max(1, Math.round(r.height * dpr));
    if (this.canvas.width !== w || this.canvas.height !== h) { this.canvas.width = w; this.canvas.height = h; }
    const S = Math.min(w, h);
    this.screen = { w, h, S, cx: w / 2, cy: h / 2, dpr };
  }

  toScreen(x, P = this.plane()) {
    const d = sub(x, P.c), { S, cx, cy } = this.screen;
    return [cx + dot(d, P.ex) / this.fov * S, cy + dot(d, P.ey) / this.fov * S, dot(d, P.n)];
  }

  fromScreen(sx, sy, P = this.plane()) {
    const { S, cx, cy } = this.screen;
    return add(P.c, add(scale(P.ex, (sx - cx) / S * this.fov), scale(P.ey, (sy - cy) / S * this.fov)));
  }

  /** Resample the volume on the plane into the RES x RES image (alpha 0 outside the volume). */
  renderImage(P) {
    const vol = this.app.state.volume, px = this.imageData.data, tx = this.textureData.data;
    if (!vol) {
      px.fill(0); tx.fill(0);
      this.imageCtx.putImageData(this.imageData, 0, 0);
      this.textureCtx.putImageData(this.textureData, 0, 0);
      this.version++;
      return;
    }
    const { center, width } = this.app.state.window;
    const lo = center - width / 2, inv = 255 / width;
    const thr = this.app.state.threshold3D;
    const step = this.fov / RES;
    const p0 = add(P.c, add(scale(P.ex, -this.fov / 2 + step / 2), scale(P.ey, -this.fov / 2 + step / 2)));
    const i0 = vol.lpsToIjk(p0);
    const di = mulMV(vol.Minv, scale(P.ex, step)), dj = mulMV(vol.Minv, scale(P.ey, step));
    let o = 0;
    for (let r = 0; r < RES; r++) {
      let a = i0[0] + dj[0] * r, b = i0[1] + dj[1] * r, c = i0[2] + dj[2] * r;
      for (let q = 0; q < RES; q++, o += 4, a += di[0], b += di[1], c += di[2]) {
        const v = vol.sampleIjk(a, b, c);
        if (Number.isNaN(v)) { px[o + 3] = 0; tx[o + 3] = 0; continue; }
        const g = Math.max(0, Math.min(255, (v - lo) * inv));
        px[o] = px[o + 1] = px[o + 2] = tx[o] = tx[o + 1] = tx[o + 2] = g;
        px[o + 3] = 255;
        tx[o + 3] = g < thr ? Math.round(255 * 0.35 * (g / Math.max(thr, 1))) : 255;
      }
    }
    this.imageCtx.putImageData(this.imageData, 0, 0);
    this.textureCtx.putImageData(this.textureData, 0, 0);
    this.version++;
  }

  render() {
    this.layout();
    const P = this.plane();
    this.lastPlane = P;
    this.renderImage(P);
    const { ctx } = this;
    const { w, h, S, cx, cy, dpr } = this.screen;
    ctx.fillStyle = '#000';
    ctx.fillRect(0, 0, w, h);
    ctx.imageSmoothingEnabled = true;
    ctx.drawImage(this.image, cx - S / 2, cy - S / 2, S, S);
    this.drawOverlay(P, dpr);
  }

  drawOverlay(P, dpr) {
    const { ctx } = this, s = this.app.state;
    const { w, h } = this.screen;
    const lw = 1.2 * dpr;
    const seg = (a, b, color, width = lw, dash = []) => {
      const A = this.toScreen(a, P), B = this.toScreen(b, P);
      ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash.map((d) => d * dpr));
      ctx.beginPath(); ctx.moveTo(A[0], A[1]); ctx.lineTo(B[0], B[1]); ctx.stroke();
      ctx.setLineDash([]);
    };
    // Crosshair (orthogonal views only).
    if (this.kind !== 'oblique') {
      const C = this.toScreen(s.cursor, P);
      ctx.strokeStyle = 'rgba(80,200,255,0.45)'; ctx.lineWidth = dpr;
      ctx.beginPath(); ctx.moveTo(C[0], 0); ctx.lineTo(C[0], h); ctx.moveTo(0, C[1]); ctx.lineTo(w, C[1]); ctx.stroke();
    }
    // Planned trajectory.
    if (s.plan) {
      const back = sub(s.plan.target, scale(s.plan.dir, 160));
      seg(back, s.plan.target, 'rgba(0,230,200,0.9)', lw, [6, 4]);
    }
    // Entry point.
    if (s.entry) this.marker(s.entry, P, '#3ddc84', 'E');
    // Needle (cannula) and biopsy core.
    const nd = s.needle;
    if (nd) {
      const tail = sub(nd.tip, scale(nd.dir, 120));
      const tipS = this.toScreen(nd.tip, P);
      const near = Math.abs(tipS[2]) < 3;
      seg(tail, nd.tip, near ? 'rgba(255,225,60,0.95)' : 'rgba(255,225,60,0.55)', 2.4 * dpr);
      ctx.fillStyle = '#ffe13c';
      ctx.beginPath(); ctx.arc(tipS[0], tipS[1], 3 * dpr, 0, 7); ctx.fill();
      if (nd.notch) seg(nd.notch[0], nd.notch[1], '#ff9a2e', 4 * dpr);
    }
    if (s.core) seg(s.core[0], s.core[1], 'rgba(255,120,40,0.9)', 5 * dpr);
    // Target.
    if (s.target) this.marker(s.target, P, '#ff3b6b', 'T', true);
    // Orientation letters and scale bar.
    ctx.fillStyle = 'rgba(255,255,255,0.75)';
    ctx.font = `${11 * dpr}px system-ui, sans-serif`;
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(dirLetters(P.ex), w - 10 * dpr, h / 2);
    ctx.fillText(dirLetters(scale(P.ex, -1)), 10 * dpr, h / 2);
    ctx.fillText(dirLetters(scale(P.ey, -1)), w / 2, 28 * dpr);
    ctx.fillText(dirLetters(P.ey), w / 2, h - 10 * dpr);
    const barMm = this.fov > 150 ? 50 : (this.fov > 60 ? 20 : 10);
    const barPx = barMm / this.fov * this.screen.S;
    ctx.strokeStyle = 'rgba(255,255,255,0.75)'; ctx.lineWidth = dpr;
    ctx.beginPath(); ctx.moveTo(w - 12 * dpr - barPx, h - 14 * dpr); ctx.lineTo(w - 12 * dpr, h - 14 * dpr); ctx.stroke();
    ctx.textAlign = 'right';
    ctx.fillText(`${barMm} mm`, w - 12 * dpr, h - 24 * dpr);
    // Info line.
    this.info.textContent = this.kind === 'oblique'
      ? `rot ${this.obliqueAngle}° · offset ${this.offset.toFixed(1)} mm`
      : `${['x', 'y', 'z'][this.kind === 'axial' ? 2 : this.kind === 'coronal' ? 1 : 0]} = ${(this.kind === 'axial' ? s.cursor[2] : this.kind === 'coronal' ? s.cursor[1] : s.cursor[0]).toFixed(1)} mm`;
  }

  marker(x, P, color, label, ring = false) {
    const { ctx } = this, dpr = this.screen.dpr;
    const [sx, sy, dz] = this.toScreen(x, P);
    const a = Math.abs(dz);
    if (a > 25) return;
    const alpha = a < 2 ? 1 : Math.max(0.25, 1 - a / 25);
    ctx.globalAlpha = alpha;
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 1.6 * dpr;
    const r = 6 * dpr;
    ctx.beginPath(); ctx.moveTo(sx - r, sy); ctx.lineTo(sx + r, sy); ctx.moveTo(sx, sy - r); ctx.lineTo(sx, sy + r); ctx.stroke();
    if (ring) { ctx.beginPath(); ctx.arc(sx, sy, r * 1.6, 0, 7); ctx.stroke(); }
    ctx.font = `bold ${10 * dpr}px system-ui, sans-serif`;
    ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
    ctx.fillText(a < 2 ? label : `${label} ${dz > 0 ? '+' : ''}${dz.toFixed(0)}`, sx + r + 2, sy - 2);
    ctx.globalAlpha = 1;
  }

  bindEvents() {
    const cv = this.canvas;
    let drag = null;
    const pos = (e) => {
      const r = cv.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
      return [(e.clientX - r.left) * dpr, (e.clientY - r.top) * dpr];
    };
    cv.addEventListener('contextmenu', (e) => e.preventDefault());
    cv.addEventListener('pointerdown', (e) => {
      cv.setPointerCapture(e.pointerId);
      const p = pos(e);
      if (e.button === 2) drag = { kind: 'wl', p, w0: { ...this.app.state.window } };
      else if (e.button === 1 || e.shiftKey) drag = { kind: 'pan', p, pan0: [...this.pan] };
      else {
        drag = { kind: 'pick' };
        this.app.onViewPick(this.fromScreen(p[0], p[1], this.lastPlane));
      }
    });
    cv.addEventListener('pointermove', (e) => {
      if (!drag) return;
      const p = pos(e);
      if (drag.kind === 'pick' && this.app.pickMode === 'cursor') this.app.onViewPick(this.fromScreen(p[0], p[1], this.lastPlane));
      if (drag.kind === 'pan') {
        const k = this.fov / this.screen.S;
        this.pan = [drag.pan0[0] - (p[0] - drag.p[0]) * k, drag.pan0[1] - (p[1] - drag.p[1]) * k];
        this.app.requestRender();
      }
      if (drag.kind === 'wl') {
        const s = this.app.state, range = (s.volume ? s.volume.max - s.volume.min : 1000) / 400;
        s.window.width = Math.max(1, drag.w0.width + (p[0] - drag.p[0]) * range);
        s.window.center = drag.w0.center - (p[1] - drag.p[1]) * range;
        this.app.onWindowChanged();
      }
    });
    cv.addEventListener('pointerup', () => { drag = null; });
    cv.addEventListener('wheel', (e) => {
      e.preventDefault();
      if (e.ctrlKey || e.metaKey) {
        this.fov = Math.min(600, Math.max(20, this.fov * Math.exp(e.deltaY * 0.0015)));
        this.app.requestRender();
        return;
      }
      const step = (e.deltaY > 0 ? 1 : -1) * (e.altKey ? 5 : 1) * (this.app.state.volume ? Math.min(...this.app.state.volume.spacing) : 1);
      if (this.kind === 'oblique') this.offset += step;
      else {
        const n = this.lastPlane.n;
        this.app.state.cursor = add(this.app.state.cursor, scale(n, step));
      }
      this.app.requestRender();
    }, { passive: false });
  }
}
