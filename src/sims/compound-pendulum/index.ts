import Matter from 'matter-js';
import { Pane } from 'tweakpane';
import type { MountedSim, SimDefinition } from '../types';

const { Engine, Bodies, Composite, Constraint, Events, Runner, Body } = Matter;

const WIDTH = 640;
const HEIGHT = 640;
const ANCHOR = { x: WIDTH / 2, y: 70 };
const TRAIL_LENGTH = 400;
const DRAG_HISTORY_SIZE = 6;
const THICKNESS = 16; // fixed rod width; mass slider changes density, not shape
const GRAB_PADDING = 16;

type DraggedRod = 'rod1' | 'rod2' | null;

interface Params {
  length1: number;
  length2: number;
  mass1: number;
  mass2: number;
  damping: number;
  showTrail: boolean;
}

const defaults: Params = {
  length1: 150,
  length2: 150,
  mass1: 5,
  mass2: 5,
  damping: 0.01,
  showTrail: true,
};

// A rod's local "hanging" axis is +y (angle 0 = straight down from its
// pivot). This is the world-space offset from center to the tip end at a
// given angle, for a rod of the given half-length.
function endpointOffset(angle: number, halfLength: number): { x: number; y: number } {
  return { x: -Math.sin(angle) * halfLength, y: Math.cos(angle) * halfLength };
}

function rodCenterFromPivot(pivot: { x: number; y: number }, angle: number, length: number) {
  const o = endpointOffset(angle, length / 2);
  return { x: pivot.x + o.x, y: pivot.y + o.y };
}

function rodTipFromCenter(center: { x: number; y: number }, angle: number, length: number) {
  const o = endpointOffset(angle, length / 2);
  return { x: center.x + o.x, y: center.y + o.y };
}

// Angle a rod must have, pivoting at `pivot`, for its tip to point at `target`.
function angleTowardTarget(pivot: { x: number; y: number }, target: { x: number; y: number }) {
  const dx = target.x - pivot.x;
  const dy = target.y - pivot.y;
  return Math.atan2(-dx, dy);
}

function distanceToSegment(p: { x: number; y: number }, a: { x: number; y: number }, b: { x: number; y: number }) {
  const abx = b.x - a.x;
  const aby = b.y - a.y;
  const lenSq = abx * abx + aby * aby || 1;
  let t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / lenSq;
  t = Math.max(0, Math.min(1, t));
  const cx = a.x + abx * t;
  const cy = a.y + aby * t;
  return Math.hypot(p.x - cx, p.y - cy);
}

// Matter.Body.scale operates in world-space X/Y axes, not the body's own
// rotated axes -- scaling a tilted rectangle's height along world Y distorts
// it into a skewed parallelogram instead of lengthening it along its own
// axis. Rotating to angle 0 (where local and world axes coincide), scaling,
// then rotating back preserves both the center and the true rod shape
// regardless of the body's orientation at the time of the resize.
function scaleRodLength(rod: Matter.Body, scaleY: number) {
  const angle = rod.angle;
  Body.setAngle(rod, 0);
  Body.scale(rod, 1, scaleY);
  Body.setAngle(rod, angle);
}

function settleVelocity(rod: Matter.Body) {
  Body.setVelocity(rod, { x: 0, y: 0 });
  Body.setAngularVelocity(rod, 0);
}

function mount(container: HTMLElement): MountedSim {
  const params: Params = { ...defaults };

  const canvas = document.createElement('canvas');
  canvas.width = WIDTH;
  canvas.height = HEIGHT;
  container.appendChild(canvas);

  const ctx = canvas.getContext('2d')!;

  const engine = Engine.create();
  engine.gravity.y = 1;
  engine.constraintIterations = 20;

  // Track the rods' current lengths outside `params` so length-change
  // handlers can compute a resize ratio even though Tweakpane has already
  // written the new value into params by the time the handler runs.
  let currentLength1 = params.length1;
  let currentLength2 = params.length2;

  const rod1 = Bodies.rectangle(ANCHOR.x, ANCHOR.y + params.length1 / 2, THICKNESS, params.length1);
  const rod2 = Bodies.rectangle(
    ANCHOR.x,
    ANCHOR.y + params.length1 + params.length2 / 2,
    THICKNESS,
    params.length2,
  );
  Body.setMass(rod1, params.mass1);
  Body.setMass(rod2, params.mass2);
  rod1.frictionAir = params.damping;
  rod2.frictionAir = params.damping;

  const pivot1 = Constraint.create({
    pointA: { x: ANCHOR.x, y: ANCHOR.y },
    bodyB: rod1,
    pointB: { x: 0, y: -params.length1 / 2 },
    length: 0,
    stiffness: 1,
  });
  const pivot2 = Constraint.create({
    bodyA: rod1,
    pointA: { x: 0, y: params.length1 / 2 },
    bodyB: rod2,
    pointB: { x: 0, y: -params.length2 / 2 },
    length: 0,
    stiffness: 1,
  });

  Composite.add(engine.world, [rod1, rod2, pivot1, pivot2]);

  function layoutRods() {
    const rod1Center = rodCenterFromPivot(ANCHOR, rod1.angle, currentLength1);
    Body.setPosition(rod1, rod1Center);

    const rod1Tip = rodTipFromCenter(rod1Center, rod1.angle, currentLength1);
    const rod2Center = rodCenterFromPivot(rod1Tip, rod2.angle, currentLength2);
    Body.setPosition(rod2, rod2Center);
  }

  let trail: { x: number; y: number }[] = [];

  // Same kinematic-drag approach as the double pendulum: while held, a rod
  // is pinned every physics tick to the angle that points its pivot-to-tip
  // axis at the pointer, so it can never stretch or detach from its joint.
  let draggedRod: DraggedRod = null;
  let lastPointer = { x: 0, y: 0 };
  let dragHistory: { x: number; y: number; angle: number; center: { x: number; y: number }; t: number }[] = [];

  function applyDragPosition() {
    if (draggedRod === 'rod1') {
      const angle = angleTowardTarget(ANCHOR, lastPointer);
      Body.setAngle(rod1, angle);
      Body.setPosition(rod1, rodCenterFromPivot(ANCHOR, angle, currentLength1));
      Body.setVelocity(rod1, { x: 0, y: 0 });
      Body.setAngularVelocity(rod1, 0);
    } else if (draggedRod === 'rod2') {
      const rod1Tip = rodTipFromCenter(rod1.position, rod1.angle, currentLength1);
      const angle = angleTowardTarget(rod1Tip, lastPointer);
      Body.setAngle(rod2, angle);
      Body.setPosition(rod2, rodCenterFromPivot(rod1Tip, angle, currentLength2));
      Body.setVelocity(rod2, { x: 0, y: 0 });
      Body.setAngularVelocity(rod2, 0);
    }
  }

  Events.on(engine, 'beforeUpdate', applyDragPosition);

  function getPointerPos(e: PointerEvent): { x: number; y: number } {
    const rect = canvas.getBoundingClientRect();
    return {
      x: ((e.clientX - rect.left) / rect.width) * WIDTH,
      y: ((e.clientY - rect.top) / rect.height) * HEIGHT,
    };
  }

  function onPointerDown(e: PointerEvent) {
    const p = getPointerPos(e);
    const rod1Tip = rodTipFromCenter(rod1.position, rod1.angle, currentLength1);
    const rod2Tip = rodTipFromCenter(rod2.position, rod2.angle, currentLength2);

    const d1 = distanceToSegment(p, ANCHOR, rod1Tip);
    const d2 = distanceToSegment(p, rod1Tip, rod2Tip);
    const reach = THICKNESS / 2 + GRAB_PADDING;

    if (d1 <= reach && d1 <= d2) {
      draggedRod = 'rod1';
    } else if (d2 <= reach) {
      draggedRod = 'rod2';
    } else {
      return;
    }

    canvas.setPointerCapture(e.pointerId);
    lastPointer = p;
    dragHistory = [];
    applyDragPosition();
    const body = draggedRod === 'rod1' ? rod1 : rod2;
    dragHistory.push({ ...p, angle: body.angle, center: { ...body.position }, t: performance.now() });
  }

  function onPointerMove(e: PointerEvent) {
    if (!draggedRod) return;
    lastPointer = getPointerPos(e);
    applyDragPosition();
    const body = draggedRod === 'rod1' ? rod1 : rod2;
    dragHistory.push({ ...lastPointer, angle: body.angle, center: { ...body.position }, t: performance.now() });
    if (dragHistory.length > DRAG_HISTORY_SIZE) dragHistory.shift();
  }

  function onPointerUp() {
    if (!draggedRod) return;
    const body = draggedRod === 'rod1' ? rod1 : rod2;
    const first = dragHistory[0];
    const last = dragHistory[dragHistory.length - 1];
    if (first && last) {
      const dt = (last.t - first.t) / 1000;
      if (dt > 0.001) {
        Body.setVelocity(body, {
          x: (last.center.x - first.center.x) / dt,
          y: (last.center.y - first.center.y) / dt,
        });
        Body.setAngularVelocity(body, (last.angle - first.angle) / dt);
      }
    }
    draggedRod = null;
    dragHistory = [];
  }

  canvas.addEventListener('pointerdown', onPointerDown);
  canvas.addEventListener('pointermove', onPointerMove);
  canvas.addEventListener('pointerup', onPointerUp);
  canvas.addEventListener('pointercancel', onPointerUp);

  function resetBodies() {
    Body.setAngle(rod1, 0);
    Body.setAngle(rod2, 0);
    Body.setAngularVelocity(rod1, 0);
    Body.setAngularVelocity(rod2, 0);
    Body.setVelocity(rod1, { x: 0, y: 0 });
    Body.setVelocity(rod2, { x: 0, y: 0 });
    layoutRods();
    trail = [];
    draggedRod = null;
    dragHistory = [];
  }

  const controlsHolder = document.createElement('div');
  controlsHolder.className = 'sim-controls';
  container.appendChild(controlsHolder);

  const pane = new Pane({ title: 'Compound Pendulum', container: controlsHolder });

  pane.addBinding(params, 'length1', { min: 50, max: 250, step: 1, label: 'Length 1' })
    .on('change', (ev) => {
      const newLength = ev.value;
      scaleRodLength(rod1, newLength / currentLength1);
      Body.setMass(rod1, params.mass1);

      // Matter mutates constraint.pointA/pointB in place, incrementally
      // rotating them each solve step to track their body's current angle.
      // Assigning a freshly *computed* offset risks a rotation-convention
      // mismatch with that internal tracking (verified the hard way -- an
      // earlier version of this fix did exactly that and still desynced
      // mid-swing). Rescaling the existing offset's magnitude by the length
      // ratio instead preserves whatever direction Matter already has
      // correctly tracked, so there's nothing to desync.
      const ratio = newLength / currentLength1;
      pivot1.pointB = { x: pivot1.pointB!.x * ratio, y: pivot1.pointB!.y * ratio };
      pivot2.pointA = { x: pivot2.pointA!.x * ratio, y: pivot2.pointA!.y * ratio };

      currentLength1 = newLength;
      layoutRods();
      settleVelocity(rod1);
      settleVelocity(rod2);
    });
  pane.addBinding(params, 'length2', { min: 50, max: 250, step: 1, label: 'Length 2' })
    .on('change', (ev) => {
      const newLength = ev.value;
      scaleRodLength(rod2, newLength / currentLength2);
      Body.setMass(rod2, params.mass2);

      const ratio = newLength / currentLength2;
      pivot2.pointB = { x: pivot2.pointB!.x * ratio, y: pivot2.pointB!.y * ratio };

      currentLength2 = newLength;
      layoutRods();
      settleVelocity(rod2);
    });
  pane.addBinding(params, 'mass1', { min: 1, max: 20, step: 0.5, label: 'Mass 1' })
    .on('change', (ev) => {
      // Resizing keeps position continuous, but changing mass/inertia while
      // velocity carries over unchanged injects unphysical kinetic energy --
      // enough, with a large enough change mid-swing, to fling the pendulum
      // wildly (verified: rod2's mass tripled mid-swing sent it flying off
      // the visible canvas, still fully attached, just now way out of frame).
      // Settling velocity on a mass edit avoids that surprise.
      Body.setMass(rod1, ev.value);
      settleVelocity(rod1);
      settleVelocity(rod2);
    });
  pane.addBinding(params, 'mass2', { min: 1, max: 20, step: 0.5, label: 'Mass 2' })
    .on('change', (ev) => {
      Body.setMass(rod2, ev.value);
      settleVelocity(rod2);
    });
  pane.addBinding(params, 'damping', { min: 0, max: 0.05, step: 0.001, label: 'Damping' })
    .on('change', (ev) => {
      rod1.frictionAir = ev.value;
      rod2.frictionAir = ev.value;
    });
  pane.addBinding(params, 'showTrail', { label: 'Show trail' });
  pane.addButton({ title: 'Reset' }).on('click', resetBodies);

  const runner = Runner.create();
  Runner.run(runner, engine);

  let frameId = 0;

  function drawRod(body: Matter.Body, length: number, color: string) {
    ctx.save();
    ctx.translate(body.position.x, body.position.y);
    ctx.rotate(body.angle);
    ctx.fillStyle = color;
    ctx.fillRect(-THICKNESS / 2, -length / 2, THICKNESS, length);
    ctx.restore();
  }

  function draw() {
    ctx.clearRect(0, 0, WIDTH, HEIGHT);

    const rod1Tip = rodTipFromCenter(rod1.position, rod1.angle, currentLength1);
    const rod2Tip = rodTipFromCenter(rod2.position, rod2.angle, currentLength2);

    if (params.showTrail) {
      trail.push({ x: rod2Tip.x, y: rod2Tip.y });
      if (trail.length > TRAIL_LENGTH) trail.shift();

      ctx.beginPath();
      for (let i = 0; i < trail.length; i++) {
        const p = trail[i];
        if (i === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      }
      ctx.strokeStyle = 'rgba(59, 110, 245, 0.5)';
      ctx.lineWidth = 2;
      ctx.stroke();
    } else if (trail.length) {
      trail = [];
    }

    drawRod(rod1, currentLength1, '#3b6ef5');
    drawRod(rod2, currentLength2, '#f5533b');

    ctx.fillStyle = '#cfd3da';
    ctx.beginPath();
    ctx.arc(ANCHOR.x, ANCHOR.y, 5, 0, Math.PI * 2);
    ctx.fill();

    ctx.beginPath();
    ctx.arc(rod1Tip.x, rod1Tip.y, 5, 0, Math.PI * 2);
    ctx.fill();

    frameId = requestAnimationFrame(draw);
  }

  frameId = requestAnimationFrame(draw);

  return {
    destroy() {
      cancelAnimationFrame(frameId);
      Runner.stop(runner);
      Events.off(engine, 'beforeUpdate', applyDragPosition);
      canvas.removeEventListener('pointerdown', onPointerDown);
      canvas.removeEventListener('pointermove', onPointerMove);
      canvas.removeEventListener('pointerup', onPointerUp);
      canvas.removeEventListener('pointercancel', onPointerUp);
      Composite.clear(engine.world, false);
      Engine.clear(engine);
      pane.dispose();
    },
  };
}

export const compoundPendulumSim: SimDefinition = {
  id: 'compound-pendulum',
  title: 'Compound Pendulum',
  mount,
};
