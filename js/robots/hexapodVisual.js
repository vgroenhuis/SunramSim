// three.js rendering of a HexapodNeedleRobot. Procedural geometry approximates the real parts;
// CAD meshes (STL/GLB, exported in the CAD assembly frame at home) replace bodies when supplied.

import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import {
  sub, normalize, mulMM, rotBetween, poseToMatrix4Array, composePose, invertPose, add, scale,
} from '../core/la.js';

const COLORS = {
  plastic: 0xf1ede2,
  plasticDark: 0xd9d3c3,
  joint: 0xd8b867,
  rack: 0xfaf8f2,
  needle: 0xb9c0c8,
  gun: 0x8f969c,
  error: 0xe0453a,
  notch: 0xff9a2e,
};

const DEG = Math.PI / 180;
const mat = (color, opts = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.55, metalness: 0.02, ...opts });

function setMatrixFromPose(obj, T) {
  obj.matrixAutoUpdate = false;
  obj.matrix.fromArray(poseToMatrix4Array(T));
  obj.matrixWorldNeedsUpdate = true;
}

function labelSprite(text) {
  const c = document.createElement('canvas');
  c.width = c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = '#fff';
  g.fillRect(4, 10, 56, 44);
  g.strokeStyle = '#bbb';
  g.strokeRect(4, 10, 56, 44);
  g.fillStyle = '#111';
  g.font = 'bold 36px system-ui, sans-serif';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.fillText(text, 32, 33);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: true }));
  s.scale.set(16, 16, 1);
  return s;
}

/** Box whose local z runs from z0 to z1. */
function zBox(w, h, z0, z1, material) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, z1 - z0), material);
  m.position.z = (z0 + z1) / 2;
  return m;
}

/** Cylinder along local x from x0 to x1. */
function xCyl(r, x0, x1, material, seg = 24) {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, x1 - x0, seg), material);
  m.rotation.z = -Math.PI / 2;
  m.position.x = (x0 + x1) / 2;
  return m;
}

export class HexapodVisual {
  constructor(robot) {
    this.robot = robot;
    const cfg = robot.cfg, v = cfg.visual, kin = robot.kin;
    this.root = new THREE.Group();
    this.root.name = `${cfg.name} base frame`;
    this.materials = { plastic: mat(COLORS.plastic), joint: mat(COLORS.joint, { transparent: true, opacity: 0.85 }) };
    this.homeDirs = kin.ik(robot.homePlatform).dirs;
    this.bodies = {}; // name -> { group, procedural }

    // --- Base -----------------------------------------------------------------------------------
    const base = this.addBody('base', this.root);
    base.procedural.add(this.makeBasePlate(v));
    const ft = v.feet || { azimuths: [0, 120, 240], radius: v.basePlateRadius * 0.62, width: 48, thickness: 8, height: v.footHeight };
    for (const az of ft.azimuths) {
      // Two slotted fins forming a cross, as cut from the plate drawing.
      const a = az * DEG;
      const foot = new THREE.Group();
      foot.position.set(ft.radius * Math.cos(a), ft.radius * Math.sin(a), -v.basePlateThickness);
      foot.rotation.z = a;
      foot.add(zBox(ft.width, ft.thickness, -ft.height, 0, this.materials.plastic));
      foot.add(zBox(ft.thickness, ft.width, -ft.height, 0, this.materials.plastic));
      base.procedural.add(foot);
    }
    const cupR = v.baseCupRadius ?? 26;
    kin.b.forEach((b, i) => {
      // Ball-joint housing: a spherical band around the ball, tilted along the cup axis.
      const g = new THREE.Group();
      setMatrixFromPose(g, { p: b, R: rotBetween([0, 1, 0], kin.baseAxes[i]) });
      g.add(new THREE.Mesh(new THREE.SphereGeometry(cupR, 32, 16, 0, Math.PI * 2, 0.9, 1.0), this.materials.joint));
      const flange = new THREE.Mesh(new THREE.CylinderGeometry(cupR + 10, cupR + 10, 4, 32), this.materials.joint);
      flange.position.y = -2;
      g.add(flange);
      base.procedural.add(g);
    });

    // --- Legs -----------------------------------------------------------------------------------
    this.legs = kin.b.map((b, i) => {
      const motor = this.addBody(`motor${i + 1}`, this.root);
      const motorMat = mat(COLORS.plastic);
      const [w, h, len] = v.motorSize;
      motor.procedural.add(zBox(w, h, v.motorOffset - len / 2, v.motorOffset + len / 2, motorMat));
      // Laminated look: thin dark bands between the laser-cut layers.
      for (let s = -2; s <= 2; s++) {
        const band = zBox(w + 0.6, h + 0.6, v.motorOffset + s * len / 5 - 0.4, v.motorOffset + s * len / 5 + 0.4, mat(COLORS.plasticDark));
        motor.procedural.add(band);
      }
      const ball = new THREE.Mesh(new THREE.SphereGeometry(v.baseBallRadius ?? 16, 24, 16), this.materials.joint);
      motor.procedural.add(ball);
      const label = labelSprite(String(i + 1));
      label.position.set(w / 2 + 2, 0, v.motorOffset + len / 2 + 6);
      motor.procedural.add(label);

      const rack = this.addBody(`rack${i + 1}`, this.root);
      const rackMat = mat(COLORS.rack);
      const top = v.rackTopOffset ?? 6;
      rack.procedural.add(zBox(v.rackSection[0], v.rackSection[1], -top - v.rackLength, -top, rackMat));
      const pball = new THREE.Mesh(new THREE.SphereGeometry(v.platformBallRadius ?? 11, 20, 14), this.materials.joint);
      rack.procedural.add(pball);
      return { motor, rack, motorMat, rackMat };
    });

    // --- Platform -------------------------------------------------------------------------------
    const platform = this.addBody('platform', this.root);
    this.platform = platform;
    const shape = new THREE.Shape();
    if (v.platformRect) {
      const [x0, x1, y0, y1] = v.platformRect;
      shape.moveTo(x0, y0); shape.lineTo(x1, y0); shape.lineTo(x1, y1); shape.lineTo(x0, y1);
    } else {
      const pts = kin.a.map((a) => [a[0], a[1]]).sort((p, q) => Math.atan2(p[1], p[0]) - Math.atan2(q[1], q[0]));
      pts.forEach(([x, y], i) => {
        const r = Math.hypot(x, y), k = (r + 22) / r;
        i === 0 ? shape.moveTo(x * k, y * k) : shape.lineTo(x * k, y * k);
      });
    }
    shape.closePath();
    const plat = new THREE.Mesh(new THREE.ExtrudeGeometry(shape, { depth: v.platformThickness, bevelEnabled: false }), this.materials.plastic);
    platform.procedural.add(plat);
    kin.a.forEach((a) => {
      // Mount from the platform underside down to the ball cup.
      const r = v.platformCupRadius ?? 15;
      const cup = new THREE.Mesh(new THREE.CylinderGeometry(r, r, Math.abs(a[2]), 24), this.materials.joint);
      cup.rotation.x = Math.PI / 2;
      cup.position.set(a[0], a[1], a[2] / 2);
      platform.procedural.add(cup);
    });
    // Insertion stage: a stack of laser-cut plates carrying the rail along the needle axis.
    const n = robot.needle;
    const railLen = n.max + 120;
    const rail = new THREE.Mesh(new THREE.BoxGeometry(railLen, 50, 14), this.materials.plastic);
    const railFrame = new THREE.Group();
    setMatrixFromPose(railFrame, { p: [0, 0, 0], R: rotBetween([1, 0, 0], n.dir) });
    rail.position.set(n.origin[0] - 150 - railLen / 2 + 120, n.origin[1], v.platformThickness + 7);
    railFrame.add(rail);
    const insMotor = new THREE.Mesh(new THREE.BoxGeometry(56, 56, 56), mat(COLORS.plastic));
    insMotor.position.set(n.origin[0] - 150 - railLen + 150, n.origin[1] - 55, v.platformThickness + 28);
    railFrame.add(insMotor);
    platform.procedural.add(railFrame);

    // --- Needle carriage (moves with insertion depth) ---------------------------------------------
    const needle = this.addBody('needle', platform.group);
    this.needleBody = needle;
    // Needle frame: origin at the tip at d = 0, local +x along the insertion axis.
    this.needleFrame0 = { p: n.origin, R: rotBetween([1, 0, 0], n.dir) };
    const bx = cfg.biopsy;
    const freeLen = 115;
    const zDrop = n.origin[2] - v.platformThickness - 14; // support height below the needle axis
    const tri = (x) => {
      const s = new THREE.Shape();
      s.moveTo(-30, -zDrop); s.lineTo(30, -zDrop); s.lineTo(0, 14); s.closePath();
      const m = new THREE.Mesh(new THREE.ExtrudeGeometry(s, { depth: 10, bevelEnabled: false }), this.materials.plastic);
      m.rotation.x = Math.PI / 2; // shape xy -> frame xz
      m.position.set(x, 5, 0);
      return m;
    };
    const gunX1 = -freeLen, gunX0 = gunX1 - v.gunLength;
    needle.procedural.add(tri(gunX0 + 15), tri(gunX1 - 25));
    needle.procedural.add(xCyl(v.gunRadius, gunX0, gunX1 - 40, mat(COLORS.gun, { transparent: true, opacity: 0.85 })));
    needle.procedural.add(xCyl(v.gunRadius * 0.7, gunX1 - 40, gunX1, mat(0xffffff)));
    this.cannula = xCyl(bx.diameter / 2, gunX1, 0, mat(COLORS.needle, { metalness: 0.6, roughness: 0.3 }), 12);
    this.cannulaX0 = gunX1;
    needle.procedural.add(this.cannula);
    // Depth marks every 10 mm on the cannula.
    for (let x = -10; x > gunX1; x -= 10) {
      const mark = xCyl(bx.diameter / 2 + 0.15, x - 0.4, x + 0.4, mat(0x222222), 12);
      needle.procedural.add(mark);
    }
    this.notch = xCyl(bx.diameter / 2 + 0.3, -bx.tipLength - bx.notchLength, -bx.tipLength, mat(COLORS.notch, { emissive: 0x552200 }), 12);
    this.notch.visible = false;
    needle.procedural.add(this.notch);
    this.fireOffset = 0;

    this.cadLoaded = {};
  }

  /** Base plate: the laser-cut outline with its holes when available, else a disc. */
  makeBasePlate(v) {
    const t = v.basePlateThickness;
    let geom;
    if (v.basePlate) {
      const P = v.basePlate;
      const shape = new THREE.Shape(P.outline.map(([x, y]) => new THREE.Vector2(x, y)));
      for (const [x, y, r] of P.circles) {
        const h = new THREE.Path();
        h.absarc(x, y, r, 0, Math.PI * 2, true);
        shape.holes.push(h);
      }
      for (const poly of P.slots) shape.holes.push(new THREE.Path(poly.map(([x, y]) => new THREE.Vector2(x, y))));
      geom = new THREE.ExtrudeGeometry(shape, { depth: t, bevelEnabled: false, curveSegments: 16 });
      geom.translate(0, 0, -t);
    } else {
      geom = new THREE.CylinderGeometry(v.basePlateRadius, v.basePlateRadius, t, 64);
      geom.rotateX(Math.PI / 2);
      geom.translate(0, 0, -t / 2);
    }
    return new THREE.Mesh(geom, this.materials.plastic);
  }

  addBody(name, parent) {
    const group = new THREE.Group();
    group.name = name;
    const procedural = new THREE.Group();
    group.add(procedural);
    parent.add(group);
    this.bodies[name] = { group, procedural, cad: null };
    return this.bodies[name];
  }

  /** Body poses (relative to each body's parent) for the current state. */
  bodyPoses(state) {
    const { platform, q, legDirs } = state;
    const poses = { base: { p: [0, 0, 0], R: [[1, 0, 0], [0, 1, 0], [0, 0, 1]] }, platform };
    const P = this.robot.kin.platformJointsInBase(platform);
    this.robot.kin.b.forEach((b, i) => {
      const d = legDirs ? legDirs[i] : normalize(sub(P[i], b));
      // Minimal rotation from the home leg orientation keeps CAD parts consistent with the home pose.
      const R = mulMM(rotBetween(this.homeDirs[i], d), rotBetween([0, 0, 1], this.homeDirs[i]));
      poses[`motor${i + 1}`] = { p: b, R };
      poses[`rack${i + 1}`] = { p: P[i], R };
    });
    const nf = this.needleFrame0;
    poses.needle = { p: add(nf.p, scale(this.robot.needle.dir, q[6])), R: nf.R };
    return poses;
  }

  update(state) {
    const poses = this.bodyPoses(state);
    for (const [name, T] of Object.entries(poses)) setMatrixFromPose(this.bodies[name].group, T);
    const bad = new Set((state.violations || []).filter((v) => v.leg < 6).map((v) => v.leg));
    this.legs.forEach((leg, i) => {
      const c = bad.has(i) ? COLORS.error : COLORS.plastic;
      leg.motorMat.color.setHex(c);
      leg.rackMat.color.setHex(bad.has(i) ? COLORS.error : COLORS.rack);
    });
  }

  /** Biopsy gun stroke: the stylet and cannula advance by `offset` mm relative to the carriage. */
  setFire(offset, showNotch) {
    this.fireOffset = offset;
    const len = -this.cannulaX0 + offset;
    this.cannula.scale.y = len / -this.cannulaX0;
    this.cannula.position.x = this.cannulaX0 + len / 2;
    this.notch.position.x = (-this.robot.cfg.biopsy.tipLength - this.robot.cfg.biopsy.notchLength / 2) + offset;
    this.notch.visible = showNotch;
  }

  /** Home pose (base frame) of a body, for converting CAD-assembly meshes into body coordinates. */
  homeBodyPoseInBase(name) {
    const home = this.robot.homeJoints();
    const st = { platform: this.robot.homePlatform, q: home };
    const poses = this.bodyPoses(st);
    if (name === 'needle') return composePose(this.robot.homePlatform, poses.needle);
    return poses[name];
  }

  /**
   * Attach a CAD mesh to a body. The file must be in the CAD assembly frame at home configuration;
   * `cadToBase` maps it into the robot base frame.
   */
  async loadCadPart(name, source, { cadToBase, color, fileName } = {}) {
    const body = this.bodies[name];
    if (!body) throw new Error(`Unknown body "${name}"`);
    const ext = (fileName || (typeof source === 'string' ? source : '')).split('.').pop().toLowerCase();
    let object;
    if (ext === 'stl') {
      const buf = typeof source === 'string' ? await (await fetch(source)).arrayBuffer() : source;
      const geom = new STLLoader().parse(buf);
      geom.computeVertexNormals();
      object = new THREE.Mesh(geom, mat(color ?? (name.startsWith('motor') || name === 'base' || name === 'platform' ? COLORS.plastic : COLORS.rack)));
    } else if (ext === 'glb' || ext === 'gltf') {
      const loader = new GLTFLoader();
      const gltf = typeof source === 'string' ? await loader.loadAsync(source) : await loader.parseAsync(source, '');
      object = gltf.scene;
    } else {
      throw new Error(`Unsupported CAD format ".${ext}" (use STL or GLB)`);
    }
    const toBase = cadToBase || this.robot.cfg.cad.cadToBase;
    const local = composePose(invertPose(this.homeBodyPoseInBase(name)), toBase);
    const holder = new THREE.Group();
    setMatrixFromPose(holder, local);
    holder.add(object);
    if (body.cad) body.group.remove(body.cad);
    body.cad = holder;
    body.group.add(holder);
    body.procedural.visible = false;
  }

  clearCadPart(name) {
    const body = this.bodies[name];
    if (body?.cad) { body.group.remove(body.cad); body.cad = null; body.procedural.visible = true; }
  }

  get bodyNames() { return Object.keys(this.bodies); }
}

