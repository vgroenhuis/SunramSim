// SunramSim application: 3D scene, robot control, image views and the biopsy workflow.
//
// Frames: the 3D scene is in a room ("world") frame, mm, whose up direction is state.roomUp.
// The robot base sits in the room at `robotWorld`; an MRI scan (DICOM patient frame, LPS, mm) is
// placed in the room at `volumePose`. Imaging, targets and plans are kept in LPS; `basePose` is
// the robot base expressed in LPS, which is what the kinematics and planning use.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import {
  add, sub, scale, dot, cross, norm, normalize, dist, mulMV, mulMM, transpose, fromRPY, toRPY, rotZ, rotY,
  poseToMatrix4Array, applyPose, invertPose, composePose, I3, DEG,
} from './core/la.js';
import { ROBOTS } from './robots/registry.js';
import { HexapodNeedleRobot } from './robots/hexapodRobot.js';
import { createBreastPhantom } from './imaging/phantom.js';
import { readDicomFiles, filesFromDataTransfer } from './imaging/dicom.js';
import { SliceView } from './ui/sliceViews.js';
import { Gimbal } from './ui/gimbal.js';

const $ = (id) => document.getElementById(id);
const fmt = (v, n = 1) => (Math.abs(v) < 0.5 * 10 ** -n ? 0 : v).toFixed(n);
const fmtV = (v, n = 1) => `(${v.map((x) => fmt(x, n).padStart(7)).join(',')})`;
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

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
function setGroupMatrix(obj, T) {
  obj.matrixAutoUpdate = false;
  obj.matrix.fromArray(poseToMatrix4Array(T));
  obj.matrixWorldNeedsUpdate = true;
}

/** An HTML element pinned next to a point of the 3D scene. */
class Anchor {
  constructor(parent, { className = 'anchor', offset = [12, -12], point }) {
    this.el = document.createElement('div');
    this.el.className = className;
    parent.appendChild(this.el);
    this.offset = offset;
    this.point = point; // () => world point [x, y, z] or null
    this.screen = null;
  }

  update(camera, w, h) {
    const p = this.point();
    if (!p) { this.el.style.display = 'none'; this.screen = null; return; }
    const v = new THREE.Vector3(...p).project(camera);
    if (v.z > 1 || v.z < -1) { this.el.style.display = 'none'; this.screen = null; return; }
    this.el.style.display = '';
    const x = (v.x + 1) / 2 * w + this.offset[0], y = (1 - v.y) / 2 * h + this.offset[1];
    this.screen = [x, y];
    const r = this.el.getBoundingClientRect();
    // Keep the element inside the view.
    const cx = Math.min(Math.max(x, 4), w - r.width - 4), cy = Math.min(Math.max(y, 4), h - r.height - 4);
    this.el.style.transform = `translate(${cx}px, ${cy}px)`;
  }
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
    this.robotWorld = { p: [0, 0, 0], R: I3() };
    this.volumePose = { p: [0, 0, 0], R: I3() };
    this.robotPlaced = false;     // has the user positioned the robot (guide step)?
    this.activeGimbal = null;     // 'base' | 'tip' | 'volume' | null
    this.pickMode = 'cursor';
    this.dirty = true;
    this.motion = null;
    this.busy = false;
    this.initScene();
    this.initViews();
    this.initUI();
    this.initOverlays();
    this.loadRobot('stormram1');
    this.frameCamera();
    this.updateGuide();
    this.loop();
  }

  /** Robot base pose in the scan (LPS) frame. */
  get basePose() { return composePose(invertPose(this.volumePose), this.robotWorld); }

  /** The room's up direction expressed in the scan frame. */
  upInPatient() { return mulMV(transpose(this.volumePose.R), this.state.roomUp); }

  lpsToWorld(x) { return applyPose(this.volumePose, x); }

  // ---------------------------------------------------------------- scene ---------------------
  initScene() {
    const el = $('view3d');
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setClearColor(0x22262c);
    el.prepend(this.renderer.domElement);
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

    // Everything expressed in the scan (LPS) frame lives in patientGroup.
    this.patientGroup = new THREE.Group();
    this.patientGroup.matrixAutoUpdate = false;
    this.scene.add(this.patientGroup);
    this.volumeGroup = new THREE.Group();
    this.patientGroup.add(this.volumeGroup);

    const sphere = (color, r) => new THREE.Mesh(new THREE.SphereGeometry(r, 20, 14), new THREE.MeshBasicMaterial({ color, depthTest: false, transparent: true, opacity: 0.9 }));
    this.targetMarker = sphere(0xff3b6b, 2.5);
    this.entryMarker = sphere(0x3ddc84, 2);
    this.targetMarker.renderOrder = this.entryMarker.renderOrder = 10;
    this.targetMarker.visible = this.entryMarker.visible = false;
    this.patientGroup.add(this.targetMarker, this.entryMarker);
    this.planLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), new THREE.LineDashedMaterial({ color: 0x00e6c8, dashSize: 6, gapSize: 4, depthTest: false, transparent: true }));
    this.planLine.renderOrder = 9;
    this.planLine.visible = false;
    this.patientGroup.add(this.planLine);
    this.coreMesh = new THREE.Mesh(new THREE.CylinderGeometry(1.4, 1.4, 1, 12), new THREE.MeshBasicMaterial({ color: 0xff7a28, depthTest: false, transparent: true, opacity: 0.85 }));
    this.coreMesh.renderOrder = 11;
    this.coreMesh.visible = false;
    this.patientGroup.add(this.coreMesh);

    // Objects the gimbal manipulates (room frame).
    this.handles = { base: new THREE.Object3D(), tip: new THREE.Object3D(), volume: new THREE.Object3D() };
    this.scene.add(...Object.values(this.handles));

    this.resize3D = () => {
      const r = el.getBoundingClientRect();
      this.renderer.setSize(r.width, r.height);
      this.camera.aspect = r.width / Math.max(1, r.height);
      this.camera.updateProjectionMatrix();
      this.viewSize = [r.width, r.height];
    };
    new ResizeObserver(() => { this.resize3D(); this.requestRender(); }).observe(el);
    this.resize3D();
    this.makeControls();
    this.gimbal = new Gimbal(this.camera, this.renderer.domElement, this.scene, {
      onChange: () => this.onGimbalChange(),
      onDragging: (d) => { this.orbit.enabled = !d; },
      onRelease: () => this.syncHandles(),
    });
  }

  makeControls() {
    const target = this.orbit ? this.orbit.target.clone() : new THREE.Vector3();
    this.orbit?.dispose();
    this.camera.up.set(...this.state.roomUp);
    this.orbit = new OrbitControls(this.camera, this.renderer.domElement);
    this.orbit.target.copy(target);
    this.orbit.enableDamping = true;
    this.orbit.dampingFactor = 0.15;
  }

  /** Fit the robot (and the scan, if any) in view, seen from the robot's side, slightly from above. */
  frameCamera() {
    const pts = [];
    const v = this.state.volume;
    if (v) pts.push(...v.corners().map((c) => this.lpsToWorld(c)));
    const base = this.robotWorld;
    const vis = this.robot.cfg.visual;
    for (const z of [-vis.footHeight, 380]) for (const a of [0, 90, 180, 270]) {
      pts.push(applyPose(base, [vis.basePlateRadius * Math.cos(a * DEG), vis.basePlateRadius * Math.sin(a * DEG), z]));
    }
    const c = scale(pts.reduce((a, b) => add(a, b), [0, 0, 0]), 1 / pts.length);
    const r = Math.max(...pts.map((p) => dist(p, c)));
    const up = this.state.roomUp;
    const needle = mulMV(base.R, this.robot.needleInBase(this.robot.homePlatform, 0).dir);
    const fwd = normalize(sub(needle, scale(up, dot(needle, up))));
    const side = normalize(cross(up, fwd));
    // Look across the needle direction so the robot and the scan in front of it are side by side.
    const eye = add(c, scale(add(add(scale(side, -1.0), scale(up, 0.5)), scale(fwd, -0.35)), r * 2.0 * Math.max(1, 1 / this.camera.aspect)));
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

  setSliceDock(open) {
    const dock = $('sliceDock');
    dock.classList.toggle('collapsed', !open);
    this.requestRender();
  }

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
    for (const [c, wd, co, wo] of [['wlCenter', 'wlWidth', 'wlCenterOut', 'wlWidthOut'], ['pvCenter', 'pvWidth', 'pvCenterOut', 'pvWidthOut']]) {
      if (!$(c)) continue;
      $(c).value = w.center; $(wd).value = w.width;
      $(co).textContent = fmt(w.center, 0); $(wo).textContent = fmt(w.width, 0);
    }
    this.requestRender();
  }

  // ---------------------------------------------------------------- images -------------------
  setVolume(vol) {
    const s = this.state;
    s.volume = vol;
    s.window = { ...vol.autoWindow };
    s.cursor = vol.center;
    const range = vol.max - vol.min;
    for (const [c, w] of [['wlCenter', 'wlWidth'], ['pvCenter', 'pvWidth']]) {
      $(c).min = vol.min; $(c).max = vol.max; $(c).step = range / 500;
      $(w).min = 1; $(w).max = range * 2; $(w).step = range / 500;
    }
    this.onWindowChanged();
    const b = vol.bounds();
    const fov = Math.max(...[0, 1, 2].map((a) => b[1][a] - b[0][a])) * 1.05;
    for (const v of Object.values(this.views)) { v.fov = v.kind === 'oblique' ? Math.min(fov, 200) : fov; v.pan = [0, 0]; v.offset = 0; }
    ['x', 'y', 'z'].forEach((k, a) => {
      const el = $(`pvSlice${k}`);
      el.min = b[0][a]; el.max = b[1][a]; el.step = Math.min(...vol.spacing);
    });
    $('volumeInfo').innerHTML = `<b>${esc(vol.name)}</b><br>${vol.dims.join(' × ')} voxels, ${vol.spacing.map((x) => x.toFixed(2)).join(' × ')} mm<br>${esc(vol.meta.modality || '')} ${vol.meta.patientPosition ? `· position ${esc(vol.meta.patientPosition)}` : ''}`;
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
    s.target = null; s.entry = null;
    // Patient position from the DICOM header decides which way is up in the room.
    const pp = (vol.meta.patientPosition || '').toUpperCase();
    const up = pp.endsWith('P') ? [0, 1, 0] : pp.endsWith('S') ? [0, -1, 0] : pp.includes('DL') ? [-1, 0, 0] : pp.includes('DR') ? [1, 0, 0] : s.roomUp;
    $('roomUp').value = up.join(',');
    if (up.join() !== s.roomUp.join()) this.setRoomUp(up);
    this.placeVolumeInFront();
    this.clearProcedure();
    this.updateTargetInfo();
    $('sliceDock').classList.remove('hidden');
    this.frameCamera();
    this.updateGuide();
    this.requestRender();
  }

  /** Change the room's up direction: re-orient camera and robot. */
  setRoomUp(up) {
    this.state.roomUp = up;
    this.makeControls();
    this.sun.position.set(...add(scale(up, 900), [300, 0, 300]));
    this.resetRobotPlacement();
    this.requestRender();
  }

  /** Put the scan (unrotated) ahead of the robot's home needle position. */
  placeVolumeInFront() {
    const vol = this.state.volume;
    if (!vol) return;
    const home = this.robot.needleInBase(this.robot.homePlatform, 0);
    const tip0 = applyPose(this.robotWorld, home.tip0);
    const dir = mulMV(this.robotWorld.R, home.dir);
    const b = vol.bounds();
    const halfAlong = 0.5 * Math.abs(dot(sub(b[1], b[0]), dir.map(Math.abs)));
    // The scan centre sits a bit more than half its depth ahead of the retracted needle tip.
    const centerWorld = add(tip0, scale(dir, halfAlong * 0.55 + 10));
    this.setVolumePose({ p: sub(centerWorld, vol.center), R: I3() });
  }

  setVolumePose(T) {
    this.volumePose = { p: [...T.p], R: T.R.map((r) => [...r]) };
    setGroupMatrix(this.patientGroup, this.volumePose);
    this.updateNeedleWorld();
    this.syncBaseFields();
    if (this.activeGimbal === 'volume' && !this.gimbal.dragging) this.syncHandles();
    this.requestRender();
  }

  loadPhantom() {
    setStatus('Generating synthetic breast phantom…');
    setTimeout(() => {
      const t0 = performance.now();
      this.setVolume(createBreastPhantom());
      setStatus(`Synthetic phantom ready (${(performance.now() - t0).toFixed(0)} ms). Next: choose a target.`);
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
      sel.innerHTML = series.map((s, i) => `<option value="${i}">${esc(s.description)}</option>`).join('');
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
    if (this.motion) this.motion.cancel();
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
    $('robotSelect').value = id;
    $('robotName').textContent = robot.cfg.name;
    $('robotDesc').innerHTML = `${esc(robot.cfg.description)}${robot.cfg.provisional ? `<br><span class="warn">${esc(robot.cfg.provisionalNote || 'Geometry is provisional.')}</span>` : ''}`;
    $('cadBody').innerHTML = this.visual.bodyNames.map((n) => `<option>${n}</option>`).join('');
    this.jointRowSets = [];
    this.buildJointUI($('jointSliders'));
    if (this.jointPop) this.buildJointUI(this.jointPop.querySelector('.pop-joints'));
    this.buildCartesianUI();
    this.q = robot.homeJoints();
    this.platform = robot.homePlatform;
    this.resetRobotPlacement();
    this.placeVolumeInFront();
    this.clearProcedure();
    this.applyJoints(this.q);
    this.updateGuide();
  }

  /** Default placement: robot upright at the room origin, home needle direction along the room's x. */
  resetRobotPlacement() {
    if (!this.robot) return;
    const up = this.state.roomUp;
    let xr = [1, 0, 0];
    if (Math.abs(dot(xr, up)) > 0.9) xr = [0, 0, 1];
    xr = normalize(sub(xr, scale(up, dot(xr, up))));
    const zr = up, yr = cross(zr, xr);
    const home = this.robot.needleInBase(this.robot.homePlatform, 0);
    const az = Math.atan2(home.dir[1], home.dir[0]);
    this.setRobotWorld({ p: [0, 0, 0], R: mulMM(transpose([xr, yr, zr]), rotZ(-az)) });
  }

  setRobotWorld(T) {
    this.robotWorld = { p: [...T.p], R: T.R.map((r) => [...r]) };
    setGroupMatrix(this.robotRoot, this.robotWorld);
    this.syncBaseFields();
    this.updateNeedleWorld();
    if ((this.activeGimbal === 'base' || this.activeGimbal === 'tip') && !this.gimbal.dragging) this.syncHandles();
    this.requestRender();
  }

  /** Set the robot base pose in the scan (LPS) frame. */
  setBasePose(T) { this.setRobotWorld(composePose(this.volumePose, T)); }

  placeForTarget() {
    const s = this.state;
    if (!s.target) { setStatus('Choose a target first.', true); return; }
    // Keep the orientation, move the base so the home needle axis passes through the target with
    // the target reached at mid-stroke.
    const base = this.basePose;
    const dirW = mulMV(base.R, mulMV(this.robot.homePlatform.R, this.robot.needle.dir));
    const tip0 = this.robot.needleInBase(this.robot.homePlatform, 0).tip0;
    const reach = (this.robot.needle.min + this.robot.needle.max) / 2 + this.preFireAdvance();
    const tipLps = sub(s.target, scale(dirW, reach));
    this.setBasePose({ p: sub(tipLps, mulMV(base.R, tip0)), R: base.R });
    this.robotPlaced = true;
    this.clearProcedure();
    this.updateGuide();
    setStatus('Robot moved so that the target lies on its home needle axis. Next: plan the needle path.');
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
    if (this.activeGimbal === 'tip' && !this.gimbal.dragging) this.syncHandles();
    this.requestRender();
    return true;
  }

  /** Needle state in the scan frame (used by the slice views and the biopsy evaluation). */
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
  /** Build a set of joint sliders into `box`; several sets (panel, pop-up) stay in sync. */
  buildJointUI(box) {
    box.innerHTML = '';
    const rows = this.robot.joints.map((j, i) => {
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
    this.jointRowSets.push(rows);
    if (this.q) this.syncJointUI();
  }

  syncJointUI() {
    if (!this.jointRowSets) return;
    const home = this.robot.homeJoints();
    const bad = new Set((this.lastIK?.violations || []).map((v) => v.leg));
    for (const rows of this.jointRowSets) {
      rows.forEach((r, i) => {
        const j = this.robot.joints[i];
        if (document.activeElement !== r.range) r.range.value = this.q[i];
        if (document.activeElement !== r.numIn) r.numIn.value = this.q[i].toFixed(2);
        const steps = Math.round((this.q[i] - home[i]) / j.step);
        r.steps.textContent = `${steps >= 0 ? '+' : ''}${steps} steps from home · range ${j.min}–${j.max} ${j.unit}`;
        r.row.classList.toggle('bad', bad.has(i));
      });
    }
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
    const lines = [
      `Platform position   ${fmtV(p.p)} mm`,
      `Platform roll/pitch/yaw ${fmtV(rpy)} °`,
      `Needle tip (base)   ${fmtV(nb.tip)} mm`,
      `Needle dir (base)   ${fmtV(nb.dir, 3)}`,
    ];
    if (this.state.volume) lines.push(`Needle tip (LPS)    ${fmtV(this.state.needle.tip)} mm`);
    lines.push(`Leg lengths ${this.q.slice(0, 6).map((v) => fmt(v, 1)).join(' ')}`,
      `Joint margin ${(ik.margin * 100).toFixed(0)}%  ${ik.ok ? 'OK' : 'LIMIT VIOLATED'}`);
    $('fkReadout').textContent = lines.join('\n');
    $('cartReadout').textContent = lines.join('\n');
  }

  // ---------------------------------------------------------------- base placement UI --------
  buildBaseFields() {
    const box = $('basePoseFields');
    const keys = ['x', 'y', 'z', 'rx', 'ry', 'rz'];
    box.innerHTML = keys.map((k) => `<label>${k}<input type="number" data-k="${k}" step="1"></label>`).join('');
    box.querySelectorAll('input').forEach((inp) => inp.addEventListener('change', () => {
      const v = Object.fromEntries([...box.querySelectorAll('input')].map((i) => [i.dataset.k, Number(i.value)]));
      this.setRobotWorld({ p: [v.x, v.y, v.z], R: fromRPY(v.rx * DEG, v.ry * DEG, v.rz * DEG) });
      this.robotPlaced = true;
      this.clearProcedure();
      this.updateGuide();
    }));
  }

  syncBaseFields() {
    const box = $('basePoseFields');
    if (!box.children.length) return;
    const T = this.robotWorld;
    const rpy = toRPY(T.R).map((a) => a / DEG);
    const vals = { x: T.p[0], y: T.p[1], z: T.p[2], rx: rpy[0], ry: rpy[1], rz: rpy[2] };
    box.querySelectorAll('input').forEach((i) => { if (document.activeElement !== i) i.value = vals[i.dataset.k].toFixed(1); });
  }

  // ---------------------------------------------------------------- gimbals ------------------
  /** Turn a gimbal on (or off when it is already on). Only one is active at a time. */
  toggleGimbal(name) {
    this.activeGimbal = this.activeGimbal === name ? null : name;
    this.gimbal.detach();
    if (this.activeGimbal) {
      this.syncHandles();
      const spaces = { base: ['world', 'local'], tip: ['local', 'local'], volume: ['world', 'local'] }[name];
      this.gimbal.attach(this.handles[name], { moveSpace: spaces[0], turnSpace: spaces[1] });
      const what = { base: 'the robot', tip: 'the needle tip', volume: 'the scan' }[name];
      setStatus(`Drag an arrow to move ${what}, or a ring to rotate it. Esc turns the gimbal off.`);
    }
    this.updateOverlayButtons();
    this.updateGuide();
    this.requestRender();
  }

  /** Put the gimbal handle on the current pose of whatever it manipulates. */
  syncHandles() {
    const g = this.activeGimbal;
    if (g === 'base') setObjectPose(this.handles.base, this.robotWorld);
    if (g === 'tip') {
      const nb = this.robot.needleInBase(this.platform, this.q[6]);
      setObjectPose(this.handles.tip, composePose(this.robotWorld, { p: nb.tip, R: this.platform.R }));
    }
    if (g === 'volume' && this.state.volume) {
      setObjectPose(this.handles.volume, { p: this.lpsToWorld(this.state.volume.center), R: this.volumePose.R });
    }
  }

  onGimbalChange() {
    const g = this.activeGimbal;
    if (g === 'base') {
      this.setRobotWorld(objectPose(this.handles.base));
      if (!this.robotPlaced) { this.robotPlaced = true; this.updateGuide(); }
      if (this.state.plan || this.state.core) this.clearProcedure();
    } else if (g === 'tip') {
      const T = composePose(invertPose(this.robotWorld), objectPose(this.handles.tip));
      this.commandTipPose(T.p, T.R);
    } else if (g === 'volume') {
      const H = objectPose(this.handles.volume);
      this.setVolumePose({ p: sub(H.p, mulMV(H.R, this.state.volume.center)), R: H.R });
      if (this.state.plan || this.state.core) this.clearProcedure();
    }
  }

  // ---------------------------------------------------------------- 3D overlays --------------
  initOverlays() {
    const root = $('overlays');
    const btn = (label, action, title = '') => `<button data-action="${action}" title="${esc(title)}">${label}</button>`;
    const robotOrigin = () => this.robotWorld.p;
    this.anchors = {
      base: new Anchor(root, { offset: [14, 22], point: robotOrigin }),
      joints: new Anchor(root, {
        offset: [14, -14],
        point: () => scale(add(this.robotWorld.p, applyPose(this.robotWorld, this.platform.p)), 0.5),
      }),
      tip: new Anchor(root, {
        offset: [14, -30],
        point: () => this.state.needle && this.lpsToWorld(this.state.needle.tip),
      }),
      volume: new Anchor(root, { offset: [10, -34], point: () => this.volumeAnchorPoint() }),
    };
    this.anchors.base.el.innerHTML = '<span class="dot"></span>' + btn('Enable robot position gimbal', 'gimbal:base', 'Move and rotate the whole robot');
    this.anchors.joints.el.innerHTML = btn('Enable joint control', 'pop:joints', 'Sliders for each motor');
    this.anchors.tip.el.innerHTML = '<span class="dot"></span>' + btn('Enable end-effector gimbal', 'gimbal:tip', 'Move and rotate the needle tip (inverse kinematics)');
    this.anchors.volume.el.innerHTML = btn('Enable gimbal', 'gimbal:volume', 'Move and rotate the scan') + btn('View settings', 'pop:view', 'Slices and contrast');
    for (const a of Object.values(this.anchors)) {
      // Touch screens show one gimbal mode at a time; this button switches it.
      a.el.insertAdjacentHTML('beforeend', '<button data-action="gimbal-mode" class="hidden" title="Switch between moving and rotating">Rotate instead</button>');
    }

    // Pop-up: joint sliders next to the robot.
    this.jointPop = document.createElement('div');
    this.jointPop.className = 'popover hidden';
    this.jointPop.innerHTML = `<div class="pop-head"><span>Joint control</span><button data-action="pop:joints" title="Close">×</button></div>
      <div class="pop-joints"></div>
      <div class="row">${btn('Home', 'home')}${btn('All joint settings…', 'open:joints')}</div>`;
    root.appendChild(this.jointPop);

    // Pop-up: basic view settings next to the scan.
    this.viewPop = document.createElement('div');
    this.viewPop.className = 'popover hidden';
    this.viewPop.innerHTML = `<div class="pop-head"><span>View settings</span><button data-action="pop:view" title="Close">×</button></div>
      <label class="field">Brightness (window center) <input type="range" id="pvCenter"><output id="pvCenterOut"></output></label>
      <label class="field">Contrast (window width) <input type="range" id="pvWidth"><output id="pvWidthOut"></output></label>
      <label class="field">Hide dark voxels in 3D <input type="range" id="pvThr" min="0" max="255" value="60"><output id="pvThrOut">60</output></label>
      <label class="field">Sagittal slice (x) <input type="range" id="pvSlicex"><output id="pvSlicexOut"></output></label>
      <label class="field">Coronal slice (y) <input type="range" id="pvSlicey"><output id="pvSliceyOut"></output></label>
      <label class="field">Axial slice (z) <input type="range" id="pvSlicez"><output id="pvSlicezOut"></output></label>
      <label class="check"><input type="checkbox" id="pvShow" checked> Show slices in 3D</label>
      <div class="row">${btn('Slice views', 'dock')}${btn('All scan settings…', 'open:scan')}</div>`;
    root.appendChild(this.viewPop);
    $('pvCenter').oninput = (e) => { this.state.window.center = Number(e.target.value); this.onWindowChanged(); };
    $('pvWidth').oninput = (e) => { this.state.window.width = Number(e.target.value); this.onWindowChanged(); };
    $('pvThr').oninput = (e) => this.setThreshold(Number(e.target.value));
    $('pvShow').onchange = (e) => { $('showSlices3d').checked = e.target.checked; this.requestRender(); };
    ['x', 'y', 'z'].forEach((k, a) => {
      $(`pvSlice${k}`).oninput = (e) => {
        const c = [...this.state.cursor];
        c[a] = Number(e.target.value);
        this.state.cursor = c;
        this.requestRender();
      };
    });

    root.addEventListener('click', (e) => {
      const b = e.target.closest('[data-action]');
      if (b) this.runAction(b.dataset.action);
    });
    this.updateOverlayButtons();
  }

  /** The top-most (on screen) corner of the scan's bounding box, in room coordinates. */
  volumeAnchorPoint() {
    const vol = this.state.volume;
    if (!vol) return null;
    let best = null, bestY = Infinity;
    for (const c of vol.corners()) {
      const w = this.lpsToWorld(c);
      const y = new THREE.Vector3(...w).project(this.camera).y * -1;
      if (y < bestY) { bestY = y; best = w; }
    }
    return best;
  }

  updateOverlayButtons() {
    const set = (action, on, onLabel, offLabel) => {
      document.querySelectorAll(`[data-action="${action}"]`).forEach((b) => {
        if (b.closest('.pop-head')) return;
        b.classList.toggle('active', on);
        if (b.closest('.anchor')) b.textContent = on ? onLabel : offLabel;
      });
    };
    set('gimbal:base', this.activeGimbal === 'base', 'Disable robot position gimbal', 'Enable robot position gimbal');
    set('gimbal:tip', this.activeGimbal === 'tip', 'Disable end-effector gimbal', 'Enable end-effector gimbal');
    set('gimbal:volume', this.activeGimbal === 'volume', 'Disable gimbal', 'Enable gimbal');
    set('pop:joints', !this.jointPop.classList.contains('hidden'), 'Hide joint control', 'Enable joint control');
    set('pop:view', !this.viewPop.classList.contains('hidden'), 'Hide view settings', 'View settings');
    for (const [name, a] of Object.entries(this.anchors)) {
      const mode = a.el.querySelector('[data-action="gimbal-mode"]');
      mode.classList.toggle('hidden', !(this.gimbal.touch && this.activeGimbal === name));
      mode.textContent = this.gimbal.mode === 'rotate' ? 'Move instead' : 'Rotate instead';
    }
  }

  togglePopover(pop) {
    pop.classList.toggle('hidden');
    this.updateOverlayButtons();
    this.requestRender();
  }

  setThreshold(v) {
    this.state.threshold3D = v;
    for (const [id, out] of [['thr3d', 'thr3dOut'], ['pvThr', 'pvThrOut']]) { $(id).value = v; $(out).textContent = v; }
    this.requestRender();
  }

  openPanel(name) {
    const p = document.querySelector(`.panel[data-panel="${name}"]`);
    if (!p) return;
    p.open = true;
    p.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
  }

  /** Named UI actions shared by the guide, the 3D buttons and the pop-ups. */
  runAction(action) {
    const [kind, arg] = action.split(':');
    switch (kind) {
      case 'gimbal': return this.toggleGimbal(arg);
      case 'gimbal-mode':
        this.gimbal.setMode(this.gimbal.mode === 'rotate' ? 'translate' : 'rotate');
        return this.updateOverlayButtons();
      case 'pop': return this.togglePopover(arg === 'joints' ? this.jointPop : this.viewPop);
      case 'open': return this.openPanel(arg);
      case 'dock': return this.setSliceDock($('sliceDock').classList.contains('collapsed'));
      case 'home':
        this.state.fire = 0; this.visual.setFire(0, false);
        return this.moveJoints(this.robot.homeJoints(), { label: 'Homing' });
      case 'phantom': return this.loadPhantom();
      case 'dicom-files': return $('dicomFiles').click();
      case 'dicom-folder': return $('dicomFolder').click();
      case 'pick-target':
        this.setSliceDock(true);
        return this.setPickMode('target');
      case 'lesion': return this.setTarget(this.state.volume.meta.lesions[Number(arg)].center);
      case 'place-for-target': return this.placeForTarget();
      case 'plan': return this.plan();
      case 'run-all': return this.runAll();
      default: return undefined;
    }
  }

  // ---------------------------------------------------------------- get-started guide --------
  updateGuide() {
    const box = $('guide');
    if (!box || !this.robot) return;
    const s = this.state;
    const steps = [
      {
        title: 'Add an MRI scan', done: !!s.volume,
        text: s.volume ? `Loaded: ${esc(s.volume.name)}.` : 'Load a scan to plan a biopsy. The synthetic phantom contains three lesions.',
        actions: s.volume ? '<button data-action="open:scan">Scan settings</button>'
          : '<button data-action="phantom" class="primary">Synthetic phantom</button><button data-action="dicom-files">DICOM files…</button><button data-action="dicom-folder">Folder…</button>',
      },
      {
        title: 'Choose a target', done: !!s.target, locked: !s.volume,
        text: s.target ? `Target at ${fmtV(s.target, 0)} (LPS).` : 'Click the lesion in the slice views, or pick one of the phantom lesions.',
        actions: '<button data-action="pick-target">Pick in slice views</button>'
          + (s.volume?.meta?.lesions || []).map((L, i) => `<button data-action="lesion:${i}">${esc(L.name.replace(/ \(.*/, ''))}</button>`).join(''),
      },
      {
        title: 'Position the robot', done: this.robotPlaced, locked: !s.volume,
        text: 'Let the simulator place the robot so the target is on its needle axis, or drag it with the robot position gimbal.',
        actions: `<button data-action="place-for-target" ${s.target ? '' : 'disabled'}>Place robot for target</button>`
          + `<button data-action="gimbal:base" class="${this.activeGimbal === 'base' ? 'active' : ''}">Robot gimbal</button>`,
      },
      {
        title: 'Plan the needle path', done: !!s.plan?.ok, locked: !s.target,
        text: s.plan ? $('planInfo').innerHTML : 'Compute the robot pose that aligns the needle with the target.',
        actions: '<button data-action="plan">Plan</button><button data-action="open:biopsy">Trajectory options…</button>',
      },
      {
        title: 'Perform the biopsy', done: !!s.core, locked: !s.plan?.ok,
        text: s.core ? `${$('procInfo').innerHTML}<br>The sampled core is shown in orange.` : 'Align the needle, insert, fire the biopsy gun and retract.',
        actions: '<button data-action="run-all" class="primary">Run all steps</button><button data-action="open:biopsy">Step by step…</button>',
      },
    ];
    const current = steps.findIndex((st) => !st.done && !st.locked);
    box.innerHTML = steps.map((st, i) => `<div class="guide-step ${st.done ? 'done' : ''} ${st.locked ? 'locked' : ''} ${i === current ? 'current' : ''}">
        <div class="title"><span class="num">${st.done ? '✓' : i + 1}</span>${st.title}</div>
        ${st.locked ? '' : `<p>${st.text}</p><div class="row">${st.actions}</div>`}
      </div>`).join('')
      + `<h3>Explore</h3>
      <div class="guide-step"><div class="title">Switch robot</div>
        <div class="row"><select data-role="robot">${ROBOTS.map((r) => `<option value="${r.id}" ${r.id === this.robot.cfg.id ? 'selected' : ''} ${r.create ? '' : 'disabled'}>${r.name}${r.create ? '' : ' (coming)'}</option>`).join('')}</select></div></div>
      <div class="guide-step"><div class="title">Move the robot</div>
        <p>Use the buttons on the robot in the 3D view, or here:</p>
        <div class="row"><button data-action="pop:joints">Joint control</button><button data-action="gimbal:tip" class="${this.activeGimbal === 'tip' ? 'active' : ''}">End-effector gimbal</button></div></div>`;
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
    this.updateGuide();
    setStatus('Target set. Next: position the robot.');
    this.requestRender();
  }

  updateTargetInfo() {
    const t = this.state.target;
    $('targetInfo').innerHTML = t ? `Target (LPS): ${fmtV(t)} mm` : 'No target set.';
  }

  /** Requested needle direction in the scan frame, or null if incomplete. */
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
    const dir = s.target && this.robot ? this.requestedDirection() : null;
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
    const base = this.basePose;
    const toBase = invertPose(base);
    const preFire = this.preFireAdvance();
    const tipGoalW = sub(s.target, scale(dirW, preFire));
    const dirB = mulMV(toBase.R, dirW);
    const depth = $('depthMode').value === 'fixed' ? Number($('depthValue').value) : null;
    const res = this.robot.planNeedle(applyPose(toBase, tipGoalW), dirB, { depth });
    const q = $('quantize').checked ? this.robot.quantize(res.q) : res.q;
    const fq = this.robot.fk(q, res.platform);
    const nb = this.robot.needleInBase(fq.platform, q[6]);
    const tipW = applyPose(base, nb.tip);
    const dirWa = mulMV(base.R, nb.dir);
    const lineErr = HexapodNeedleRobot.pointLineDistance(tipGoalW, tipW, dirWa);
    s.plan = { ...res, q, target: s.target, tipGoal: tipGoalW, dir: dirW, preFire, predictedTipError: dist(tipW, tipGoalW), predictedLineError: lineErr };
    const info = $('planInfo');
    if (res.ok) {
      info.innerHTML = `<span class="ok">Feasible.</span> Insertion ${fmt(res.depth)} mm, roll ${fmt(res.roll / DEG, 0)}°, joint margin ${(res.margin * 100).toFixed(0)}%.<br>`
        + `Step quantisation: tip error ${fmt(s.plan.predictedTipError, 2)} mm, off-axis ${fmt(lineErr, 2)} mm.`;
    } else {
      info.innerHTML = `<span class="bad">Not reachable</span> from this robot placement: ${describeViolations(res.violations, this.robot)}.<br>Try “Place robot for target”, another direction, or move the robot.`;
    }
    this.updatePlanPreview();
    this.updateProcedureButtons();
    this.updateGuide();
    setStatus(res.ok ? 'Plan ready: Align → Insert → Fire → Retract.' : 'Plan infeasible.', !res.ok);
    return s.plan;
  }

  /** Discard plan and biopsy state (the plan is tied to the current target and robot placement). */
  clearProcedure() {
    const s = this.state;
    const had = !!(s.plan || s.core);
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
    if (had) this.updateGuide();
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
      this.updateGuide();
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
    $('robotSelect').innerHTML = ROBOTS.map((r) => `<option value="${r.id}" ${r.create ? '' : 'disabled'}>${r.name}${r.create ? '' : ' (coming)'}</option>`).join('');
    $('robotSelect').addEventListener('change', (e) => this.loadRobot(e.target.value));
    $('resetView').onclick = () => this.frameCamera();

    // Guide: buttons run named actions; the robot selector switches robots.
    $('guide').addEventListener('click', (e) => {
      const b = e.target.closest('[data-action]');
      if (b && !b.disabled) this.runAction(b.dataset.action);
    });
    $('guide').addEventListener('change', (e) => {
      if (e.target.dataset.role === 'robot') this.loadRobot(e.target.value);
    });
    $('dockTab').onclick = () => this.runAction('dock');
    // Panels behave as an accordion (the guide stays open independently).
    document.querySelectorAll('.panel').forEach((p) => p.addEventListener('toggle', () => {
      if (!p.open || p.dataset.panel === 'guide') return;
      document.querySelectorAll('.panel').forEach((o) => { if (o !== p && o.dataset.panel !== 'guide') o.open = false; });
    }));

    // Scan panel.
    $('loadPhantom').onclick = () => this.loadPhantom();
    $('dicomFiles').onchange = (e) => { this.loadDicom(e.target.files); e.target.value = ''; };
    $('dicomFolder').onchange = (e) => { this.loadDicom(e.target.files); e.target.value = ''; };
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
    $('thr3d').oninput = (e) => this.setThreshold(Number(e.target.value));
    $('showSlices3d').onchange = (e) => { if ($('pvShow')) $('pvShow').checked = e.target.checked; this.requestRender(); };
    $('showObliqueIn3d').onchange = () => this.requestRender();
    $('roomUp').onchange = (e) => {
      this.setRoomUp(e.target.value.split(',').map(Number));
      this.placeVolumeInFront();
      this.clearProcedure();
      this.frameCamera();
    };
    $('resetVolume').onclick = () => { this.placeVolumeInFront(); this.clearProcedure(); this.frameCamera(); };

    // Robot panel.
    this.buildBaseFields();
    $('placeForTarget').onclick = () => this.placeForTarget();
    $('resetBase').onclick = () => { this.resetRobotPlacement(); this.robotPlaced = false; this.clearProcedure(); this.updateGuide(); };
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
        if (o.material.userData.opacity0 === undefined) { o.material.userData.opacity0 = o.material.opacity; o.material.userData.wasTransparent = o.material.transparent; }
        o.material.transparent = e.target.checked || o.material.userData.wasTransparent;
        o.material.opacity = e.target.checked ? 0.35 : o.material.userData.opacity0;
        o.material.depthWrite = !e.target.checked;
      });
      this.requestRender();
    };

    // Joints panel.
    $('homeBtn').onclick = () => this.runAction('home');

    // Biopsy panel.
    $('pickTarget').onclick = () => {
      if (this.pickMode === 'target') this.setPickMode('cursor');
      else this.runAction('pick-target');
    };
    $('pickEntry').onclick = () => {
      this.setSliceDock(true);
      this.setPickMode(this.pickMode === 'entry' ? 'cursor' : 'entry');
    };
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
    window.addEventListener('keydown', (e) => {
      if (e.key !== 'Escape') return;
      this.setPickMode('cursor');
      if (this.activeGimbal) this.toggleGimbal(this.activeGimbal);
    });
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
    if (this.volumeBox) this.volumeBox.visible = !!this.state.volume;
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
    // Slice positions in the view-settings pop-up follow the cursor.
    if (s.volume) {
      ['x', 'y', 'z'].forEach((k, a) => {
        const el = $(`pvSlice${k}`);
        if (document.activeElement !== el) el.value = s.cursor[a];
        $(`pvSlice${k}Out`).textContent = `${fmt(s.cursor[a], 0)} mm`;
      });
    }
  }

  /** Pin the 3D buttons and pop-ups to their scene objects. */
  updateOverlays() {
    const [w, h] = this.viewSize;
    for (const [name, a] of Object.entries(this.anchors)) {
      a.update(this.camera, w, h);
      if (name === 'volume' && !this.state.volume) a.el.style.display = 'none';
    }
    // Pop-ups open beside their button (right, or left when there is no room), vertically centred.
    const place = (pop, anchor) => {
      if (pop.classList.contains('hidden')) return;
      if (!anchor.screen) { pop.style.visibility = 'hidden'; return; }
      pop.style.visibility = '';
      const host = $('view3d').getBoundingClientRect();
      const a = anchor.el.getBoundingClientRect();
      const r = pop.getBoundingClientRect();
      const left = a.left - host.left, right = a.right - host.left, mid = a.top - host.top + a.height / 2;
      let x = right + 10;
      if (x + r.width > w - 8) x = left - r.width - 10;
      x = Math.min(Math.max(x, 8), w - r.width - 8);
      const y = Math.min(Math.max(mid - r.height / 2, 8), h - r.height - 8);
      pop.style.transform = `translate(${x}px, ${y}px)`;
    };
    place(this.jointPop, this.anchors.joints);
    if (!this.state.volume) this.viewPop.classList.add('hidden');
    place(this.viewPop, this.anchors.volume);
  }

  loop() {
    requestAnimationFrame(() => this.loop());
    if (this.dirty) {
      this.dirty = false;
      if (this.state.volume) for (const v of Object.values(this.views)) v.render();
      this.updateSlicePlanes();
      this.update3DMarkers();
    }
    this.orbit.update();
    this.renderer.render(this.scene, this.camera);
    this.updateOverlays();
  }
}

window.app = new App();
