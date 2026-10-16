// SunramSim application: 3D scene, robot control, image views and the biopsy workflow.
// World coordinates are the DICOM patient frame (LPS, mm). The robot base pose is expressed in it.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import {
  add, sub, scale, dot, cross, norm, normalize, dist, mulMV, mulMM, transpose, fromRPY, toRPY, rotZ, rotY,
  poseToMatrix4Array, applyPose, invertPose, composePose, DEG,
} from './core/la.js';
import { ROBOTS } from './robots/registry.js';
import { HexapodNeedleRobot } from './robots/hexapodRobot.js';
import { createBreastPhantom } from './imaging/phantom.js';
import { readDicomFiles, filesFromDataTransfer } from './imaging/dicom.js';
import { SliceView } from './ui/sliceViews.js';

const $ = (id) => document.getElementById(id);
const fmt = (v, n = 1) => (Math.abs(v) < 0.5 * 10 ** -n ? 0 : v).toFixed(n);
const fmtV = (v, n = 1) => `(${v.map((x) => fmt(x, n).padStart(7)).join(',')})`;

function setStatus(msg, bad = false) {
  const el = $('status');
  el.textContent = msg;
  el.classList.toggle('bad', bad);
}

function describeViolations(vs, robot) {
  const names = { 'length-min': 'leg too short', 'length-max': 'leg too long', 'base-joint-angle': 'base ball joint at limit', 'platform-joint-angle': 'platform ball joint at limit', 'insertion-range': 'insertion out of range' };
  return vs.map((v) => `${robot.joints[v.leg].name}: ${names[v.type] || v.type}`).filter((s, i, a) => a.indexOf(s) === i).join('; ');
}

/** three.js Object3D world matrix -> pose. */
function objectPose(obj) {
  obj.updateMatrixWorld(true);
  const e = obj.matrixWorld.elements;
  return { p: [e[12], e[13], e[14]], R: [[e[0], e[4], e[8]], [e[1], e[5], e[9]], [e[2], e[6], e[10]]] };
}
function setObjectPose(obj, T) {
  const m = new THREE.Matrix4().fromArray(poseToMatrix4Array(T));
  m.decompose(obj.position, obj.quaternion, obj.scale);
  obj.updateMatrixWorld(true);
}

class App {
  constructor() {
    this.state = {
      volume: null,
      window: { center: 500, width: 1000 },
      threshold3D: 60,
      cursor: [0, 0, 0],
      target: null,
      entry: null,
      plan: null,
      needle: null,
      core: null,
      roomUp: [0, 1, 0],
      fire: 0,
    };
    this.pickMode = 'cursor';
    this.dirty = true;
    this.motion = null;
    this.busy = false;
    this.initScene();
    this.initViews();
    this.initUI();
    this.loadRobot('stormram1');
    this.loadPhantom();
    this.loop();
  }

  // ---------------------------------------------------------------- scene ---------------------
  initScene() {
    const el = $('view3d');
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setClearColor(0x22262c);
    el.appendChild(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(38, 1, 5, 20000);
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x3a3f46, 1.6));
    const sun = new THREE.DirectionalLight(0xffffff, 1.8);
    sun.position.set(400, 900, 600);
    this.scene.add(sun);
    this.sun = sun;
    const fill = new THREE.DirectionalLight(0xffffff, 0.6);
    fill.position.set(-600, -200, -400);
    this.scene.add(fill);

    this.robotRoot = new THREE.Group();
    this.robotRoot.matrixAutoUpdate = false;
    this.scene.add(this.robotRoot);

    this.volumeGroup = new THREE.Group();
    this.scene.add(this.volumeGroup);

    const sphere = (color, r) => new THREE.Mesh(new THREE.SphereGeometry(r, 20, 14), new THREE.MeshBasicMaterial({ color, depthTest: false, transparent: true, opacity: 0.9 }));
    this.targetMarker = sphere(0xff3b6b, 2.5);
    this.entryMarker = sphere(0x3ddc84, 2);
    this.targetMarker.renderOrder = this.entryMarker.renderOrder = 10;
    this.targetMarker.visible = this.entryMarker.visible = false;
    this.scene.add(this.targetMarker, this.entryMarker);
    this.planLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), new THREE.LineDashedMaterial({ color: 0x00e6c8, dashSize: 6, gapSize: 4, depthTest: false, transparent: true }));
    this.planLine.renderOrder = 9;
    this.planLine.visible = false;
    this.scene.add(this.planLine);
    this.coreMesh = new THREE.Mesh(new THREE.CylinderGeometry(1.4, 1.4, 1, 12), new THREE.MeshBasicMaterial({ color: 0xff7a28, depthTest: false, transparent: true, opacity: 0.85 }));
    this.coreMesh.renderOrder = 11;
    this.coreMesh.visible = false;
    this.scene.add(this.coreMesh);

    // Handles for the 3D manipulator.
    this.tipHandle = new THREE.Object3D();
    this.baseHandle = new THREE.Object3D();
    this.scene.add(this.tipHandle, this.baseHandle);

    this.resize3D = () => {
      const r = el.getBoundingClientRect();
      this.renderer.setSize(r.width, r.height);
      this.camera.aspect = r.width / Math.max(1, r.height);
      this.camera.updateProjectionMatrix();
    };
    new ResizeObserver(() => { this.resize3D(); this.requestRender(); }).observe(el);
    this.resize3D();
    this.makeControls();
  }

  makeControls() {
    const target = this.orbit ? this.orbit.target.clone() : new THREE.Vector3();
    this.orbit?.dispose();
    this.camera.up.set(...this.state.roomUp);
    this.orbit = new OrbitControls(this.camera, this.renderer.domElement);
    this.orbit.target.copy(target);
    this.orbit.enableDamping = true;
    this.orbit.dampingFactor = 0.15;
    if (!this.gizmo) {
      this.gizmo = new TransformControls(this.camera, this.renderer.domElement);
      this.gizmo.setSize(0.8);
      this.scene.add(this.gizmo.getHelper());
      this.gizmo.addEventListener('objectChange', () => this.onGizmoChange());
      this.gizmo.addEventListener('mouseUp', () => this.onGizmoRelease());
    }
    this.gizmo.addEventListener('dragging-changed', (e) => { this.orbit.enabled = !e.value; });
  }

  frameCamera() {
    const pts = [];
    const v = this.state.volume;
    if (v) pts.push(...v.corners());
    const base = this.basePose;
    const vis = this.robot.cfg.visual;
    for (const z of [-vis.footHeight, 380]) for (const a of [0, 90, 180, 270]) {
      pts.push(applyPose(base, [vis.basePlateRadius * Math.cos(a * DEG), vis.basePlateRadius * Math.sin(a * DEG), z]));
    }
    const c = scale(pts.reduce((a, b) => add(a, b), [0, 0, 0]), 1 / pts.length);
    const r = Math.max(...pts.map((p) => dist(p, c)));
    const up = this.state.roomUp;
    // View from the robot's side, slightly from above.
    const side = normalize(sub(mulMV(base.R, [0, -1, 0]), scale(up, dot(mulMV(base.R, [0, -1, 0]), up))));
    const along = normalize(cross(up, side));
    const eye = add(c, scale(add(add(scale(side, 1.0), scale(up, 0.45)), scale(along, -0.35)), r * 1.9));
    this.camera.position.set(...eye);
    this.orbit.target.set(...c);
    this.orbit.update();
  }

  // ---------------------------------------------------------------- slice views ---------------
  initViews() {
    this.views = {
      axial: new SliceView($('svAxial'), 'axial', this),
      coronal: new SliceView($('svCoronal'), 'coronal', this),
      sagittal: new SliceView($('svSagittal'), 'sagittal', this),
      oblique: new SliceView($('svOblique'), 'oblique', this),
    };
    this.slicePlanes = {};
    for (const [k, view] of Object.entries(this.views)) {
      const tex = new THREE.CanvasTexture(view.texture);
      tex.colorSpace = THREE.SRGBColorSpace;
      const m = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.MeshBasicMaterial({ map: tex, transparent: true, side: THREE.DoubleSide, depthWrite: false, alphaTest: 0.02 }));
      m.matrixAutoUpdate = false;
      m.renderOrder = 1;
      m.userData = { tex, version: -1 };
      this.volumeGroup.add(m);
      this.slicePlanes[k] = m;
    }
    for (const v of Object.values(this.views)) new ResizeObserver(() => this.requestRender()).observe(v.el);
  }

  requestRender() { this.dirty = true; }

  needleViewLine() {
    const s = this.state;
    if (s.plan) return { center: s.plan.target, dir: s.plan.dir };
    if (s.needle) return { center: s.needle.tip, dir: s.needle.dir };
    return { center: s.cursor, dir: [1, 0, 0] };
  }

  onViewPick(p) {
    if (this.pickMode === 'target') {
      this.setTarget(p);
      this.setPickMode('cursor');
    } else if (this.pickMode === 'entry') {
      this.state.entry = p;
      this.setPickMode('cursor');
      this.updatePlanPreview();
    } else {
      this.state.cursor = p;
    }
    this.requestRender();
  }

  onWindowChanged() {
    const w = this.state.window;
    $('wlCenter').value = w.center; $('wlWidth').value = w.width;
    $('wlCenterOut').textContent = fmt(w.center, 0); $('wlWidthOut').textContent = fmt(w.width, 0);
    this.requestRender();
  }

  // ---------------------------------------------------------------- images -------------------
  setVolume(vol) {
    const s = this.state;
    s.volume = vol;
    s.window = { ...vol.autoWindow };
    s.cursor = vol.center;
    const range = vol.max - vol.min;
    for (const id of ['wlCenter', 'wlWidth']) {
      $(id).min = id === 'wlWidth' ? 1 : vol.min;
      $(id).max = id === 'wlWidth' ? range * 2 : vol.max;
      $(id).step = range / 500;
    }
    this.onWindowChanged();
    const pp = (vol.meta.patientPosition || '').toUpperCase();
    const up = pp.endsWith('P') ? [0, 1, 0] : pp.endsWith('S') ? [0, -1, 0] : pp.includes('DL') ? [-1, 0, 0] : pp.includes('DR') ? [1, 0, 0] : s.roomUp;
    $('roomUp').value = up.join(',');
    this.setRoomUp(up, false);
    const b = vol.bounds();
    const fov = Math.max(...[0, 1, 2].map((a) => b[1][a] - b[0][a])) * 1.05;
    for (const v of Object.values(this.views)) { v.fov = v.kind === 'oblique' ? Math.min(fov, 200) : fov; v.pan = [0, 0]; v.offset = 0; }
    $('volumeInfo').innerHTML = `<b>${vol.name}</b><br>${vol.dims.join(' × ')} voxels, ${vol.spacing.map((x) => x.toFixed(2)).join(' × ')} mm<br>${vol.meta.modality || ''} ${vol.meta.patientPosition ? `· position ${vol.meta.patientPosition}` : ''}`;
    // Volume outline.
    if (this.volumeBox) this.volumeGroup.remove(this.volumeBox);
    const box = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1)), new THREE.LineBasicMaterial({ color: 0x6f7c8a }));
    const ax = vol.axes.map((a, i) => scale(a, vol.dims[i] - 1 || 1));
    const c = vol.center;
    box.matrixAutoUpdate = false;
    box.matrix.set(ax[0][0], ax[1][0], ax[2][0], c[0], ax[0][1], ax[1][1], ax[2][1], c[1], ax[0][2], ax[1][2], ax[2][2], c[2], 0, 0, 0, 1);
    this.volumeBox = box;
    this.volumeGroup.add(box);
    // Lesion shortcuts.
    const lb = $('lesionButtons');
    lb.innerHTML = '';
    for (const L of vol.meta.lesions || []) {
      const btn = document.createElement('button');
      btn.textContent = L.name;
      btn.onclick = () => this.setTarget(L.center);
      lb.appendChild(btn);
    }
    this.clearProcedure();
    s.target = null; s.entry = null;
    this.updateTargetInfo();
    this.resetBasePlacement();
    this.frameCamera();
    this.requestRender();
  }

  setRoomUp(up, reframe = true) {
    this.state.roomUp = up;
    this.makeControls();
    this.sun.position.set(...add(scale(up, 900), [300, 0, 300]));
    if (reframe) this.frameCamera();
    this.requestRender();
  }

  loadPhantom() {
    setStatus('Generating synthetic breast phantom…');
    setTimeout(() => {
      const t0 = performance.now();
      this.setVolume(createBreastPhantom());
      setStatus(`Synthetic phantom ready (${(performance.now() - t0).toFixed(0)} ms). Pick a lesion as target in the Biopsy tab.`);
    }, 10);
  }

  async loadDicom(files) {
    files = [...files];
    if (!files.length) return;
    setStatus(`Reading ${files.length} files…`);
    try {
      const { series, errors } = await readDicomFiles(files, (n, N) => setStatus(`Reading DICOM ${n}/${N}…`));
      if (!series.length) throw new Error(errors[0] || 'No DICOM images found');
      this.series = series;
      const sel = $('seriesSelect');
      sel.innerHTML = series.map((s, i) => `<option value="${i}">${s.description}</option>`).join('');
      sel.disabled = series.length < 2;
      this.loadSeries(0);
      if (errors.length) setStatus(`Loaded with ${errors.length} file(s) skipped: ${errors[0]}`, true);
    } catch (e) {
      console.error(e);
      setStatus(`DICOM load failed: ${e.message}`, true);
    }
  }

  loadSeries(i) {
    try {
      const vol = this.series[i].build();
      this.setVolume(vol);
      setStatus(`Loaded ${this.series[i].description}`);
    } catch (e) {
      console.error(e);
      setStatus(`Could not build volume: ${e.message}`, true);
    }
  }

  // ---------------------------------------------------------------- robot --------------------
  loadRobot(id) {
    const def = ROBOTS.find((r) => r.id === id);
    if (!def?.create) { setStatus(`${def?.name ?? id} is not available yet.`, true); return; }
    const { robot, Visual } = def.create();
    this.robot = robot;
    if (this.visual) this.robotRoot.remove(this.visual.root);
    this.visual = new Visual(robot);
    this.robotRoot.add(this.visual.root);
    // Floor grid under the robot feet.
    if (this.grid) this.robotRoot.remove(this.grid);
    this.grid = new THREE.GridHelper(1000, 20, 0x4a525c, 0x343a42);
    this.grid.rotation.x = Math.PI / 2;
    this.grid.position.z = -(robot.cfg.visual.footHeight + robot.cfg.visual.basePlateThickness);
    this.robotRoot.add(this.grid);
    $('robotTitle').textContent = robot.cfg.name;
    $('robotDesc').innerHTML = `${robot.cfg.description}${robot.cfg.provisional ? '<br><span class="warn">Geometry is provisional (estimated from photographs) until CAD joint coordinates are entered in <code>js/robots/stormram1.js</code>.</span>' : ''}`;
    $('cadBody').innerHTML = this.visual.bodyNames.map((n) => `<option>${n}</option>`).join('');
    this.buildJointUI();
    this.buildCartesianUI();
    this.q = robot.homeJoints();
    this.platform = robot.homePlatform;
    this.resetBasePlacement();
    this.applyJoints(this.q);
  }

  /** Default placement: robot upright in the room, needle pointing towards the volume centre. */
  resetBasePlacement() {
    if (!this.robot) return;
    const up = this.state.roomUp;
    const vol = this.state.volume;
    const center = vol ? vol.center : [0, 0, 0];
    // The home needle direction points horizontally; prefer the patient's left-right axis.
    let xr = [1, 0, 0];
    if (Math.abs(dot(xr, up)) > 0.9) xr = [0, 0, 1];
    xr = normalize(sub(xr, scale(up, dot(xr, up))));
    const zr = up, yr = cross(zr, xr);
    const home = this.robot.needleInBase(this.robot.homePlatform, 0);
    const az = Math.atan2(home.dir[1], home.dir[0]);
    const R = mulMM(transpose([xr, yr, zr]), rotZ(-az));
    const tip0 = home.tip0;
    let standoff = 40;
    if (vol) {
      const b = vol.bounds();
      // Half-extent of the volume along the needle direction.
      standoff = 0.5 * Math.abs(dot(sub(b[1], b[0]), xr.map(Math.abs))) * 0.55 + 10;
    }
    const tipWorld = sub(center, scale(xr, standoff));
    this.setBasePose({ p: sub(tipWorld, mulMV(R, tip0)), R });
  }

  setBasePose(T) {
    this.basePose = { p: [...T.p], R: T.R.map((r) => [...r]) };
    this.robotRoot.matrix.fromArray(poseToMatrix4Array(this.basePose));
    this.robotRoot.matrixWorldNeedsUpdate = true;
    this.syncBaseFields();
    this.updateNeedleWorld();
    if (this.gizmoMode?.startsWith('base') && !this.gizmo.dragging) setObjectPose(this.baseHandle, this.basePose);
    this.requestRender();
  }

  placeForTarget() {
    const s = this.state;
    if (!s.target) { setStatus('Set a target first (Biopsy tab).', true); return; }
    // Keep the orientation, move the base so the home needle axis passes through the target with
    // the target reached at mid-stroke.
    const dirW = mulMV(this.basePose.R, mulMV(this.robot.homePlatform.R, this.robot.needle.dir));
    const tip0 = this.robot.needleInBase(this.robot.homePlatform, 0).tip0;
    const reach = (this.robot.needle.min + this.robot.needle.max) / 2 + this.preFireAdvance();
    const tipWorld = sub(s.target, scale(dirW, reach));
    this.setBasePose({ p: sub(tipWorld, mulMV(this.basePose.R, tip0)), R: this.basePose.R });
    this.clearProcedure();
    setStatus('Robot base moved so that the target lies on the home needle axis.');
  }

  /** Distance between the pre-fire tip and the target for the selected targeting mode. */
  preFireAdvance() {
    const b = this.robot.cfg.biopsy;
    return $('targetAt').value === 'notch' ? b.throw - b.tipLength - b.notchLength / 2 : 0;
  }

  /** Apply a joint vector immediately (forward kinematics, visuals, readouts). */
  applyJoints(q, { fromCartesian = false } = {}) {
    const fk = this.robot.fk(q, this.platform);
    if (!fk.ok) { setStatus('Forward kinematics did not converge for these leg lengths.', true); return false; }
    this.q = [...q];
    this.platform = fk.platform;
    const ik = this.robot.ikPlatform(fk.platform, q[6]);
    this.lastIK = ik;
    this.visual.update({ platform: fk.platform, q, legDirs: ik.legDirs, violations: ik.violations });
    this.updateNeedleWorld();
    this.syncJointUI();
    if (!fromCartesian) this.syncCartesianUI();
    this.updateReadouts();
    if (this.gizmoMode?.startsWith('tip') && !this.gizmo.dragging) this.placeTipHandle();
    this.requestRender();
    return true;
  }

  updateNeedleWorld() {
    if (!this.robot || !this.platform) return;
    const nb = this.robot.needleInBase(this.platform, this.q[6]);
    const T = this.basePose;
    const dir = mulMV(T.R, nb.dir);
    const s = this.state;
    const tip = add(applyPose(T, nb.tip), scale(dir, s.fire));
    const b = this.robot.cfg.biopsy;
    s.needle = {
      tip, dir, tip0: applyPose(T, nb.tip0),
      notch: s.fire > 0 ? [sub(tip, scale(dir, b.tipLength + b.notchLength)), sub(tip, scale(dir, b.tipLength))] : null,
    };
  }

  /** Animate to joint goal q (synchronised joint interpolation at the motor step rate). */
  moveJoints(qGoal, { label = 'Moving' } = {}) {
    if (this.motion) this.motion.cancel();
    const speed = Number($('speed').value);
    const q0 = [...this.q];
    const quant = $('quantize').checked;
    if (quant) qGoal = this.robot.quantize(qGoal);
    const steps = qGoal.map((g, i) => Math.abs(g - q0[i]) / this.robot.joints[i].step);
    const maxSteps = Math.max(...steps);
    const T = speed === 0 ? 0 : maxSteps / this.robot.stepFrequency / speed;
    return new Promise((resolve) => {
      const t0 = performance.now();
      let cancelled = false;
      const motion = { cancel: () => { cancelled = true; resolve(false); } };
      this.motion = motion;
      const tick = () => {
        if (cancelled) return;
        const t = T === 0 ? 1 : Math.min(1, (performance.now() - t0) / 1000 / T);
        let q = q0.map((a, i) => a + (qGoal[i] - a) * t);
        if (quant) q = this.robot.quantize(q);
        this.applyJoints(t >= 1 ? qGoal : q);
        setStatus(`${label}… ${(t * 100).toFixed(0)}% (${(T * (1 - t)).toFixed(1)} s left, ${maxSteps.toFixed(0)} steps at ${this.robot.stepFrequency} Hz${speed > 1 ? ` ×${speed}` : ''})`);
        if (t < 1) requestAnimationFrame(tick);
        else { this.motion = null; resolve(true); }
      };
      tick();
    });
  }

  // ---------------------------------------------------------------- joint / cartesian UI -----
  buildJointUI() {
    const box = $('jointSliders');
    box.innerHTML = '';
    this.jointRows = this.robot.joints.map((j, i) => {
      const row = document.createElement('div');
      row.className = 'joint';
      row.innerHTML = `<span class="name">${j.name}</span><input type="range" min="${j.min}" max="${j.max}" step="${j.step}"><input type="number" step="${j.step}"><span class="steps"></span>`;
      const [range, numIn] = row.querySelectorAll('input');
      const set = (v) => {
        if (this.motion) this.motion.cancel();
        const q = [...this.q];
        // The rack stroke is a hard limit; ball-joint angle violations are reported but allowed.
        q[i] = Math.min(j.max, Math.max(j.min, Number(v)));
        if ($('quantize').checked) q[i] = this.robot.quantize(q)[i];
        this.applyJoints(q);
        const ik = this.lastIK;
        setStatus(ik.ok ? `${j.name} = ${fmt(q[i], 2)} mm` : `Joint limits violated: ${describeViolations(ik.violations, this.robot)}`, !ik.ok);
      };
      range.addEventListener('input', () => set(range.value));
      numIn.addEventListener('change', () => set(numIn.value));
      box.appendChild(row);
      return { row, range, numIn, steps: row.querySelector('.steps') };
    });
  }

  syncJointUI() {
    if (!this.jointRows) return;
    const home = this.robot.homeJoints();
    const bad = new Set((this.lastIK?.violations || []).map((v) => v.leg));
    this.jointRows.forEach((r, i) => {
      const j = this.robot.joints[i];
      if (document.activeElement !== r.range) r.range.value = this.q[i];
      if (document.activeElement !== r.numIn) r.numIn.value = this.q[i].toFixed(2);
      const steps = Math.round((this.q[i] - home[i]) / j.step);
      r.steps.textContent = `${steps >= 0 ? '+' : ''}${steps} steps from home · range ${j.min}–${j.max} ${j.unit}`;
      r.row.classList.toggle('bad', bad.has(i));
    });
  }

  buildCartesianUI() {
    const defs = [
      ['x', 'Tip x', -150, 150, 0.5], ['y', 'Tip y', -150, 150, 0.5], ['z', 'Tip z', -120, 120, 0.5],
      ['roll', 'Roll', -40, 40, 0.5], ['pitch', 'Pitch', -40, 40, 0.5], ['yaw', 'Yaw', -40, 40, 0.5],
    ];
    const home = this.robot.needleInBase(this.robot.homePlatform, 0).tip0;
    const box = $('cartSliders');
    box.innerHTML = '';
    this.cartRows = {};
    for (const [k, label, lo, hi, step] of defs) {
      const pos = 'xyz'.indexOf(k);
      const min = pos >= 0 ? home[pos] + lo : lo, max = pos >= 0 ? home[pos] + hi : hi;
      const row = document.createElement('div');
      row.className = 'joint';
      row.innerHTML = `<span class="name">${label}</span><input type="range" min="${min}" max="${max}" step="${step}"><input type="number" step="${step}">`;
      const [range, numIn] = row.querySelectorAll('input');
      range.addEventListener('input', () => { numIn.value = range.value; this.onCartesianInput(); });
      numIn.addEventListener('change', () => { range.value = numIn.value; this.onCartesianInput(); });
      box.appendChild(row);
      this.cartRows[k] = { range, numIn };
    }
  }

  cartesianFromState() {
    const nb = this.robot.needleInBase(this.platform, this.q[6]);
    const Rrel = mulMM(this.platform.R, transpose(this.robot.homePlatform.R));
    const rpy = toRPY(Rrel).map((a) => a / DEG);
    return { x: nb.tip[0], y: nb.tip[1], z: nb.tip[2], roll: rpy[0], pitch: rpy[1], yaw: rpy[2] };
  }

  syncCartesianUI() {
    if (!this.cartRows) return;
    const c = this.cartesianFromState();
    for (const [k, r] of Object.entries(this.cartRows)) {
      r.range.value = c[k];
      r.numIn.value = c[k].toFixed(2);
    }
  }

  onCartesianInput() {
    const v = Object.fromEntries(Object.entries(this.cartRows).map(([k, r]) => [k, Number(r.numIn.value)]));
    const R = mulMM(fromRPY(v.roll * DEG, v.pitch * DEG, v.yaw * DEG), this.robot.homePlatform.R);
    this.commandTipPose([v.x, v.y, v.z], R, true);
  }

  /** Inverse kinematics for a needle-tip pose in the base frame; applies it if feasible. */
  commandTipPose(tip, R, fromCartesian = false) {
    if (this.motion) this.motion.cancel();
    const d = this.q[6];
    const platform = this.robot.platformFromTip(tip, R, d);
    const ik = this.robot.ikPlatform(platform, d);
    const st = $('ikStatus');
    if (!ik.ok) {
      st.innerHTML = `<span class="bad">Unreachable:</span> ${describeViolations(ik.violations, this.robot)}`;
      setStatus(`IK: pose not reachable — ${describeViolations(ik.violations, this.robot)}`, true);
      if (fromCartesian) this.syncCartesianUI();
      return false;
    }
    let q = ik.q;
    if ($('quantize').checked) q = this.robot.quantize(q);
    this.applyJoints(q, { fromCartesian });
    st.innerHTML = `<span class="ok">Reachable</span> · joint margin ${(ik.margin * 100).toFixed(0)}%`;
    setStatus('IK solution applied.');
    return true;
  }

  updateReadouts() {
    const nb = this.robot.needleInBase(this.platform, this.q[6]);
    const p = this.platform;
    const rpy = toRPY(mulMM(p.R, transpose(this.robot.homePlatform.R))).map((a) => a / DEG);
    const ik = this.lastIK;
    const nw = this.state.needle;
    const txt = [
      `Platform position   ${fmtV(p.p)} mm`,
      `Platform roll/pitch/yaw ${fmtV(rpy)} °`,
      `Needle tip (base)   ${fmtV(nb.tip)} mm`,
      `Needle dir (base)   ${fmtV(nb.dir, 3)}`,
      `Needle tip (LPS)    ${fmtV(nw.tip)} mm`,
      `Leg lengths ${this.q.slice(0, 6).map((v) => fmt(v, 1)).join(' ')}`,
      `Joint margin ${(ik.margin * 100).toFixed(0)}%  ${ik.ok ? 'OK' : 'LIMIT VIOLATED'}`,
    ].join('\n');
    $('fkReadout').textContent = txt;
    $('cartReadout').textContent = txt;
  }

  // ---------------------------------------------------------------- base placement UI --------
  buildBaseFields() {
    const box = $('basePoseFields');
    const keys = ['x', 'y', 'z', 'rx', 'ry', 'rz'];
    box.innerHTML = keys.map((k) => `<label>${k}<input type="number" data-k="${k}" step="${k.startsWith('r') ? 1 : 1}"></label>`).join('');
    box.querySelectorAll('input').forEach((inp) => inp.addEventListener('change', () => {
      const v = Object.fromEntries([...box.querySelectorAll('input')].map((i) => [i.dataset.k, Number(i.value)]));
      this.setBasePose({ p: [v.x, v.y, v.z], R: fromRPY(v.rx * DEG, v.ry * DEG, v.rz * DEG) });
      this.clearProcedure();
    }));
  }

  syncBaseFields() {
    const box = $('basePoseFields');
    if (!box.children.length) return;
    const rpy = toRPY(this.basePose.R).map((a) => a / DEG);
    const vals = { x: this.basePose.p[0], y: this.basePose.p[1], z: this.basePose.p[2], rx: rpy[0], ry: rpy[1], rz: rpy[2] };
    box.querySelectorAll('input').forEach((i) => { if (document.activeElement !== i) i.value = vals[i.dataset.k].toFixed(1); });
  }

  // ---------------------------------------------------------------- 3D manipulator -----------
  setGizmoMode(mode) {
    this.gizmoMode = mode;
    document.querySelectorAll('[data-gizmo]').forEach((b) => b.classList.toggle('active', b.dataset.gizmo === mode));
    this.gizmo.detach();
    if (mode === 'none') return;
    const [obj, kind] = mode.split('-');
    if (obj === 'tip') { this.placeTipHandle(); this.gizmo.attach(this.tipHandle); this.gizmo.setSpace('local'); }
    else { setObjectPose(this.baseHandle, this.basePose); this.gizmo.attach(this.baseHandle); this.gizmo.setSpace('world'); }
    this.gizmo.setMode(kind);
  }

  placeTipHandle() {
    const nb = this.robot.needleInBase(this.platform, this.q[6]);
    setObjectPose(this.tipHandle, composePose(this.basePose, { p: nb.tip, R: this.platform.R }));
  }

  onGizmoChange() {
    if (this.gizmoMode.startsWith('base')) {
      this.setBasePose(objectPose(this.baseHandle));
      this.clearProcedure();
    } else {
      const T = composePose(invertPose(this.basePose), objectPose(this.tipHandle));
      this.commandTipPose(T.p, T.R);
    }
  }

  onGizmoRelease() {
    if (this.gizmoMode?.startsWith('tip')) this.placeTipHandle();
  }

  // ---------------------------------------------------------------- biopsy workflow ----------
  setPickMode(mode) {
    this.pickMode = mode;
    $('pickTarget').classList.toggle('active', mode === 'target');
    $('pickEntry').classList.toggle('active', mode === 'entry');
    if (mode !== 'cursor') setStatus(`Click in a slice view to set the ${mode} point.`);
  }

  setTarget(p) {
    this.state.target = [...p];
    this.state.cursor = [...p];
    this.clearProcedure();
    this.updateTargetInfo();
    this.updatePlanPreview();
    this.requestRender();
  }

  updateTargetInfo() {
    const t = this.state.target;
    $('targetInfo').innerHTML = t ? `Target (LPS): ${fmtV(t)} mm` : 'No target set.';
  }

  /** Requested needle direction in world coordinates, or null if incomplete. */
  requestedDirection() {
    const s = this.state, mode = $('dirMode').value;
    const homeDirBase = mulMV(this.robot.homePlatform.R, this.robot.needle.dir);
    if (mode === 'entry') {
      if (!s.entry || !s.target) return null;
      const d = sub(s.target, s.entry);
      return norm(d) > 1 ? normalize(d) : null;
    }
    let db = homeDirBase;
    if (mode === 'angles') {
      const az = Number($('azimuth').value) * DEG, el = Number($('elevation').value) * DEG;
      db = mulMV(rotZ(az), mulMV(rotY(-el), homeDirBase));
    }
    return mulMV(this.basePose.R, db);
  }

  updatePlanPreview() {
    const s = this.state;
    const dir = this.requestedDirection();
    if (s.target && dir) {
      const back = sub(s.target, scale(dir, 200));
      this.planLine.geometry.setFromPoints([new THREE.Vector3(...back), new THREE.Vector3(...s.target)]);
      this.planLine.computeLineDistances();
      this.planLine.visible = true;
    } else this.planLine.visible = false;
    this.requestRender();
  }

  plan() {
    const s = this.state;
    if (!s.target) { setStatus('Set a target first.', true); return null; }
    const dirW = this.requestedDirection();
    if (!dirW) { setStatus('Pick an entry point to define the trajectory.', true); return null; }
    const toBase = invertPose(this.basePose);
    const preFire = this.preFireAdvance();
    const tipGoalW = sub(s.target, scale(dirW, preFire));
    const dirB = mulMV(toBase.R, dirW);
    const depth = $('depthMode').value === 'fixed' ? Number($('depthValue').value) : null;
    const res = this.robot.planNeedle(applyPose(toBase, tipGoalW), dirB, { depth });
    const q = $('quantize').checked ? this.robot.quantize(res.q) : res.q;
    const fq = this.robot.fk(q, res.platform);
    const nb = this.robot.needleInBase(fq.platform, q[6]);
    const tipW = applyPose(this.basePose, nb.tip);
    const dirWa = mulMV(this.basePose.R, nb.dir);
    const lineErr = HexapodNeedleRobot.pointLineDistance(tipGoalW, tipW, dirWa);
    s.plan = { ...res, q, target: s.target, tipGoal: tipGoalW, dir: dirW, preFire, predictedTipError: dist(tipW, tipGoalW), predictedLineError: lineErr };
    const info = $('planInfo');
    if (res.ok) {
      info.innerHTML = `<span class="ok">Feasible.</span> Insertion ${fmt(res.depth)} mm, roll ${fmt(res.roll / DEG, 0)}°, joint margin ${(res.margin * 100).toFixed(0)}%.<br>`
        + `Step quantisation: tip error ${fmt(s.plan.predictedTipError, 2)} mm, off-axis ${fmt(lineErr, 2)} mm.`;
    } else {
      info.innerHTML = `<span class="bad">Not reachable</span> from this robot placement: ${describeViolations(res.violations, this.robot)}.<br>Try “Place robot for target” (Robot tab), another direction, or move the base.`;
    }
    this.updatePlanPreview();
    this.updateProcedureButtons();
    setStatus(res.ok ? 'Plan ready: Align → Insert → Fire → Retract.' : 'Plan infeasible.', !res.ok);
    return s.plan;
  }

  /** Discard plan and biopsy state (the plan is tied to the current target and base placement). */
  clearProcedure() {
    const s = this.state;
    s.plan = null;
    s.core = null;
    s.fire = 0;
    this.procStage = 0;
    if (this.visual) this.visual.setFire(0, false);
    this.updateNeedleWorld();
    $('planInfo').innerHTML = '';
    $('procInfo').innerHTML = '';
    this.updateProcedureButtons();
    this.updatePlanPreview();
  }

  updateProcedureButtons() {
    const ok = !!this.state.plan?.ok && !this.busy;
    const st = this.procStage || 0;
    $('alignBtn').disabled = !ok;
    $('insertBtn').disabled = !ok || st < 1;
    $('fireBtn').disabled = !ok || st < 2 || st >= 3;
    $('retractBtn').disabled = this.busy || (this.q?.[6] <= this.robot?.needle.min + 1e-6 && !this.state.fire);
    $('runAllBtn').disabled = !ok;
    ['alignBtn', 'insertBtn', 'fireBtn', 'retractBtn'].forEach((id, i) => {
      $(id).classList.toggle('done', st > i);
    });
  }

  async runStep(name, fn) {
    this.busy = true;
    this.updateProcedureButtons();
    try { return await fn(); } finally { this.busy = false; this.updateProcedureButtons(); }
  }

  async align() {
    const p = this.state.plan;
    if (!p?.ok) return;
    await this.runStep('align', async () => {
      if (this.q[6] > this.robot.needle.min + 1e-6 || this.state.fire) await this.retract(true);
      const goal = [...p.q.slice(0, 6), this.robot.needle.min];
      if (!(await this.moveJoints(goal, { label: 'Aligning needle' }))) return;
      const nw = this.state.needle;
      const err = HexapodNeedleRobot.pointLineDistance(p.target, nw.tip, nw.dir);
      this.procStage = 1;
      $('procInfo').innerHTML = `Aligned. Target is ${fmt(err, 2)} mm from the needle axis; ${fmt(dot(sub(p.target, nw.tip), nw.dir))} mm ahead of the tip.`;
      setStatus('Needle aligned. Next: Insert.');
    });
  }

  async insert() {
    const p = this.state.plan;
    if (!p?.ok) return;
    await this.runStep('insert', async () => {
      const goal = [...this.q.slice(0, 6), p.q[6]];
      if (!(await this.moveJoints(goal, { label: 'Inserting needle' }))) return;
      this.procStage = 2;
      const nw = this.state.needle;
      $('procInfo').innerHTML = `Inserted to ${fmt(this.q[6])} mm. Tip is ${fmt(dist(nw.tip, p.tipGoal), 2)} mm from the planned pre-fire position.`;
      setStatus(p.preFire > 0 ? 'Needle in pre-fire position. Next: Fire biopsy.' : 'Needle tip at target. Next: Fire biopsy.');
    });
  }

  async fire() {
    const p = this.state.plan;
    await this.runStep('fire', async () => {
      const b = this.robot.cfg.biopsy;
      const dur = 180;
      const t0 = performance.now();
      await new Promise((res) => {
        const tick = () => {
          const t = Math.min(1, (performance.now() - t0) / dur);
          this.state.fire = b.throw * t;
          this.visual.setFire(this.state.fire, t >= 1);
          this.updateNeedleWorld();
          this.requestRender();
          if (t < 1) requestAnimationFrame(tick); else res();
        };
        tick();
      });
      const nw = this.state.needle;
      this.state.core = nw.notch.map((x) => [...x]);
      this.procStage = 3;
      $('procInfo').innerHTML = this.evaluateCore(p.target);
      setStatus('Biopsy fired. Next: Retract.');
    });
  }

  /** Compare the specimen notch with the target (and phantom lesions when known). */
  evaluateCore(target) {
    const [A, B] = this.state.core;
    const u = normalize(sub(B, A)), len = dist(A, B);
    const mid = scale(add(A, B), 0.5);
    const lateral = HexapodNeedleRobot.pointLineDistance(target, A, u);
    const axial = dot(sub(target, mid), u);
    const inside = Math.abs(axial) <= len / 2;
    let html = `Specimen notch centre to target: <b>${fmt(dist(mid, target), 2)} mm</b> (off-axis ${fmt(lateral, 2)} mm, along axis ${axial >= 0 ? '+' : ''}${fmt(axial, 2)} mm).<br>`;
    html += inside && lateral < 1.5 ? '<span class="ok">Target lies within the sampled core.</span>' : '<span class="warn">Target is outside the sampled core.</span>';
    const lesions = this.state.volume?.meta?.lesions;
    if (lesions) {
      let hitLen = 0;
      const N = 60;
      for (let k = 0; k < N; k++) {
        const x = add(A, scale(u, (k + 0.5) / N * len));
        if (lesions.some((L) => dist(x, L.center) < L.radius)) hitLen += len / N;
      }
      html += `<br>Lesion tissue in core: <b>${fmt(hitLen)} / ${fmt(len)} mm</b> ${hitLen > 0 ? '<span class="ok">(diagnostic)</span>' : '<span class="bad">(miss)</span>'}`;
    }
    return html;
  }

  async retract(internal = false) {
    const run = async () => {
      // The fired needle is withdrawn with the carriage; the gun is re-cocked afterwards.
      const goal = [...this.q.slice(0, 6), this.robot.needle.min];
      await this.moveJoints(goal, { label: 'Retracting needle' });
      this.state.fire = 0;
      this.visual.setFire(0, false);
      this.updateNeedleWorld();
      if (!internal) {
        if (this.procStage >= 3) this.procStage = 4;
        setStatus(this.state.core ? 'Needle retracted; specimen acquired (shown in orange).' : 'Needle retracted.');
      }
      this.requestRender();
    };
    if (internal) return run();
    return this.runStep('retract', run);
  }

  async runAll() {
    if (!this.state.plan?.ok && !this.plan()?.ok) return;
    await this.align();
    await this.insert();
    await this.fire();
    await new Promise((r) => setTimeout(r, 400));
    await this.retract();
  }

  // ---------------------------------------------------------------- UI wiring ----------------
  initUI() {
    document.querySelectorAll('.tabs button').forEach((b) => b.addEventListener('click', () => {
      document.querySelectorAll('.tabs button').forEach((x) => x.classList.toggle('active', x === b));
      document.querySelectorAll('.tab-panel').forEach((p) => p.classList.toggle('active', p.dataset.panel === b.dataset.tab));
    }));
    $('robotSelect').innerHTML = ROBOTS.map((r) => `<option value="${r.id}" ${r.create ? '' : 'disabled'}>${r.name}${r.create ? '' : ' (coming)'}</option>`).join('');
    $('robotSelect').addEventListener('change', (e) => this.loadRobot(e.target.value));
    document.querySelectorAll('[data-gizmo]').forEach((b) => b.addEventListener('click', () => this.setGizmoMode(b.dataset.gizmo)));

    // Scan tab.
    $('loadPhantom').onclick = () => this.loadPhantom();
    $('dicomFiles').onchange = (e) => this.loadDicom(e.target.files);
    $('dicomFolder').onchange = (e) => this.loadDicom(e.target.files);
    $('seriesSelect').onchange = (e) => this.loadSeries(Number(e.target.value));
    const dz = $('dropZone');
    const over = (e) => { e.preventDefault(); dz.classList.add('over'); };
    dz.addEventListener('dragover', over);
    dz.addEventListener('dragleave', () => dz.classList.remove('over'));
    dz.addEventListener('drop', async (e) => {
      e.preventDefault();
      dz.classList.remove('over');
      this.loadDicom(await filesFromDataTransfer(e.dataTransfer));
    });
    $('wlCenter').oninput = (e) => { this.state.window.center = Number(e.target.value); this.onWindowChanged(); };
    $('wlWidth').oninput = (e) => { this.state.window.width = Number(e.target.value); this.onWindowChanged(); };
    $('thr3d').oninput = (e) => { this.state.threshold3D = Number(e.target.value); $('thr3dOut').textContent = e.target.value; this.requestRender(); };
    $('showSlices3d').onchange = () => this.requestRender();
    $('showObliqueIn3d').onchange = () => this.requestRender();
    $('roomUp').onchange = (e) => { this.setRoomUp(e.target.value.split(',').map(Number)); this.resetBasePlacement(); this.frameCamera(); };

    // Robot tab.
    this.buildBaseFields();
    $('placeForTarget').onclick = () => this.placeForTarget();
    $('resetBase').onclick = () => { this.resetBasePlacement(); this.clearProcedure(); };
    $('cadFile').onchange = async (e) => {
      const f = e.target.files[0];
      if (!f) return;
      try {
        await this.visual.loadCadPart($('cadBody').value, await f.arrayBuffer(), { fileName: f.name });
        this.applyJoints(this.q);
        setStatus(`Loaded ${f.name} as ${$('cadBody').value}.`);
      } catch (err) { setStatus(`CAD load failed: ${err.message}`, true); }
      e.target.value = '';
    };
    $('cadClear').onclick = () => { this.visual.clearCadPart($('cadBody').value); this.requestRender(); };
    $('showRobot').onchange = (e) => { this.robotRoot.visible = e.target.checked; this.requestRender(); };
    $('robotGhost').onchange = (e) => {
      this.visual.root.traverse((o) => {
        if (!o.material || o.isSprite) return;
        o.material.transparent = e.target.checked || o.material.userData.wasTransparent;
        if (o.material.userData.opacity0 === undefined) { o.material.userData.opacity0 = o.material.opacity; o.material.userData.wasTransparent = o.material.transparent; }
        o.material.opacity = e.target.checked ? 0.35 : o.material.userData.opacity0;
        o.material.depthWrite = !e.target.checked;
      });
      this.requestRender();
    };

    // Joints tab.
    $('homeBtn').onclick = () => { this.state.fire = 0; this.visual.setFire(0, false); this.moveJoints(this.robot.homeJoints(), { label: 'Homing' }); };

    // Biopsy tab.
    $('pickTarget').onclick = () => this.setPickMode(this.pickMode === 'target' ? 'cursor' : 'target');
    $('pickEntry').onclick = () => this.setPickMode(this.pickMode === 'entry' ? 'cursor' : 'entry');
    $('targetFromCursor').onclick = () => this.setTarget(this.state.cursor);
    $('dirMode').onchange = (e) => {
      $('entryRow').classList.toggle('hidden', e.target.value !== 'entry');
      $('anglesRow').classList.toggle('hidden', e.target.value !== 'angles');
      this.clearProcedure();
    };
    for (const id of ['azimuth', 'elevation']) {
      $(id).oninput = (e) => { $(`${id}Out`).textContent = e.target.value; this.clearProcedure(); };
    }
    $('depthMode').onchange = (e) => { $('depthRow').classList.toggle('hidden', e.target.value !== 'fixed'); this.clearProcedure(); };
    $('targetAt').onchange = () => this.clearProcedure();
    $('planBtn').onclick = () => this.plan();
    $('alignBtn').onclick = () => this.align();
    $('insertBtn').onclick = () => this.insert();
    $('fireBtn').onclick = () => this.fire();
    $('retractBtn').onclick = () => this.retract();
    $('runAllBtn').onclick = () => this.runAll();
    $('resetProcBtn').onclick = async () => {
      this.clearProcedure();
      await this.retract();
      this.moveJoints(this.robot.homeJoints(), { label: 'Homing' });
    };
    window.addEventListener('keydown', (e) => { if (e.key === 'Escape') { this.setPickMode('cursor'); this.setGizmoMode('none'); } });
  }

  // ---------------------------------------------------------------- render loop --------------
  updateSlicePlanes() {
    const show = $('showSlices3d').checked && !!this.state.volume;
    for (const [k, m] of Object.entries(this.slicePlanes)) {
      const view = this.views[k];
      m.visible = show && (k !== 'oblique' || $('showObliqueIn3d').checked) && !!view.lastPlane;
      if (!m.visible) continue;
      const P = view.lastPlane, f = view.fov;
      const ex = scale(P.ex, f), ey = scale(P.ey, -f), n = P.n;
      m.matrix.set(ex[0], ey[0], n[0], P.c[0], ex[1], ey[1], n[1], P.c[1], ex[2], ey[2], n[2], P.c[2], 0, 0, 0, 1);
      m.matrixWorldNeedsUpdate = true;
      if (m.userData.version !== view.version) { m.userData.tex.needsUpdate = true; m.userData.version = view.version; }
    }
  }

  update3DMarkers() {
    const s = this.state;
    this.targetMarker.visible = !!s.target;
    if (s.target) this.targetMarker.position.set(...s.target);
    this.entryMarker.visible = !!s.entry && $('dirMode').value === 'entry';
    if (s.entry) this.entryMarker.position.set(...s.entry);
    this.coreMesh.visible = !!s.core;
    if (s.core) {
      const [A, B] = s.core;
      const mid = scale(add(A, B), 0.5);
      this.coreMesh.position.set(...mid);
      this.coreMesh.scale.set(1, dist(A, B), 1);
      this.coreMesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(...normalize(sub(B, A))));
    }
  }

  loop() {
    requestAnimationFrame(() => this.loop());
    if (this.dirty) {
      this.dirty = false;
      for (const v of Object.values(this.views)) v.render();
      this.updateSlicePlanes();
      this.update3DMarkers();
    }
    this.orbit.update();
    this.renderer.render(this.scene, this.camera);
  }
}

window.app = new App();
