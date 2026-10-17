// A combined translate + rotate manipulator: two TransformControls on the same object, the rotation
// rings drawn larger than the translation arrows. With a mouse both are active at once (the handle
// under the pointer wins); on touch screens one mode is shown at a time.

import { TransformControls } from 'three/addons/controls/TransformControls.js';

export class Gimbal {
  /**
   * @param {object} o
   * @param {(dragging: boolean) => void} o.onDragging  called when a drag starts / ends
   */
  constructor(camera, dom, scene, { onChange, onDragging, onRelease }) {
    this.touch = window.matchMedia?.('(pointer: coarse)').matches ?? false;
    this.move = new TransformControls(camera, dom);
    this.move.setMode('translate');
    this.move.setSize(0.75);
    this.turn = new TransformControls(camera, dom);
    this.turn.setMode('rotate');
    this.turn.setSize(1.15);
    for (const c of [this.move, this.turn]) {
      scene.add(c.getHelper());
      c.addEventListener('objectChange', () => onChange?.());
      c.addEventListener('dragging-changed', (e) => onDragging?.(e.value));
      c.addEventListener('mouseUp', () => onRelease?.());
    }
    // When the pointer is over a translation handle, keep the rotation rings from also grabbing it
    // (both controls listen to the same pointerdown).
    dom.addEventListener('pointerdown', () => {
      if (this.move.axis && this.turn.enabled && this.mode === 'both') {
        this.turn.enabled = false;
        window.addEventListener('pointerup', () => { this.turn.enabled = true; }, { once: true });
      }
    }, { capture: true });
    this.mode = this.touch ? 'translate' : 'both';
    this.object = null;
  }

  get dragging() { return this.move.dragging || this.turn.dragging; }

  attach(object, { moveSpace = 'world', turnSpace = 'local' } = {}) {
    this.object = object;
    this.move.attach(object);
    this.turn.attach(object);
    this.move.setSpace(moveSpace);
    this.turn.setSpace(turnSpace);
    this.setMode(this.mode);
  }

  detach() {
    this.object = null;
    this.move.detach();
    this.turn.detach();
  }

  /** 'both' (mouse), or 'translate' / 'rotate' (one at a time, for touch screens). */
  setMode(mode) {
    this.mode = mode;
    const on = !!this.object;
    this.move.getHelper().visible = on && mode !== 'rotate';
    this.turn.getHelper().visible = on && mode !== 'translate';
    this.move.enabled = on && mode !== 'rotate';
    this.turn.enabled = on && mode !== 'translate';
  }
}
