import Matter from 'matter-js';
import { Pane } from 'tweakpane';
import type { MountedSim, SimDefinition } from '../types';

const { Engine, Bodies, Composite, Constraint, Events, Runner, Body } = Matter;

const WIDTH = 640;
const HEIGHT = 640;
const ANCHOR = { x: WIDTH / 2, y: 70 };
const TRAIL_LENGTH = 400;
const DRAG_HISTORY_SIZE = 6;

type DraggedBob = 'bob1' | 'bob2' | null;

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

function radiusForMass(mass: number): number {
  return Math.min(40, 8 + Math.sqrt(mass) * 6);
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
  // Default constraintIterations (2) lets the rods stretch a little under
  // fast, chaotic swings, which quietly bleeds energy and softens the
  // sensitivity that makes a double pendulum chaotic. A stiffer solve keeps
  // the rods closer to truly rigid.
  engine.constraintIterations = 20;

  const bob1 = Bodies.circle(ANCHOR.x, ANCHOR.y + params.length1, radiusForMass(params.mass1), {
    frictionAir: params.damping,
  });
  const bob2 = Bodies.circle(
    ANCHOR.x,
    ANCHOR.y + params.length1 + params.length2,
    radiusForMass(params.mass2),
    { frictionAir: params.damping },
  );
  Body.setMass(bob1, params.mass1);
  Body.setMass(bob2, params.mass2);

  const rod1 = Constraint.create({
    pointA: { x: ANCHOR.x, y: ANCHOR.y },
    bodyB: bob1,
    length: params.length1,
    stiffness: 1,
  });
  const rod2 = Constraint.create({
    bodyA: bob1,
    bodyB: bob2,
    length: params.length2,
    stiffness: 1,
  });

  Composite.add(engine.world, [bob1, bob2, rod1, rod2]);

  let trail: { x: number; y: number }[] = [];

  // Dragging is kinematic rather than a Matter MouseConstraint spring: the
  // grabbed bob is pinned to a point clamped onto its rod's fixed-length
  // circle around its pivot (the anchor for bob1, or bob1 itself for bob2),
  // every physics tick. That makes the rod length exactly correct for the
  // whole drag instead of approximately correct via a spring fighting the
  // rod constraint, which is what caused the visible stretch.
  let draggedBob: DraggedBob = null;
  let lastPointer = { x: 0, y: 0 };
  let dragHistory: { x: number; y: number; t: number }[] = [];

  function clampToRod(target: { x: number; y: number }, center: { x: number; y: number }, length: number) {
    const dx = target.x - center.x;
    const dy = target.y - center.y;
    const dist = Math.hypot(dx, dy) || 1;
    const scale = length / dist;
    return { x: center.x + dx * scale, y: center.y + dy * scale };
  }

  function applyDragPosition() {
    if (draggedBob === 'bob1') {
      Body.setPosition(bob1, clampToRod(lastPointer, ANCHOR, params.length1));
      Body.setVelocity(bob1, { x: 0, y: 0 });
      Body.setAngularVelocity(bob1, 0);
    } else if (draggedBob === 'bob2') {
      Body.setPosition(bob2, clampToRod(lastPointer, bob1.position, params.length2));
      Body.setVelocity(bob2, { x: 0, y: 0 });
      Body.setAngularVelocity(bob2, 0);
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
    const d1 = Math.hypot(p.x - bob1.position.x, p.y - bob1.position.y);
    const d2 = Math.hypot(p.x - bob2.position.x, p.y - bob2.position.y);
    const reach1 = (bob1.circleRadius ?? 10) * 2.5;
    const reach2 = (bob2.circleRadius ?? 10) * 2.5;

    if (d1 <= reach1 && d1 <= d2) {
      draggedBob = 'bob1';
    } else if (d2 <= reach2) {
      draggedBob = 'bob2';
    } else {
      return;
    }

    canvas.setPointerCapture(e.pointerId);
    lastPointer = p;
    dragHistory = [{ ...p, t: performance.now() }];
    applyDragPosition();
  }

  function onPointerMove(e: PointerEvent) {
    if (!draggedBob) return;
    lastPointer = getPointerPos(e);
    dragHistory.push({ ...lastPointer, t: performance.now() });
    if (dragHistory.length > DRAG_HISTORY_SIZE) dragHistory.shift();
    applyDragPosition();
  }

  function onPointerUp() {
    if (!draggedBob) return;
    const body = draggedBob === 'bob1' ? bob1 : bob2;
    const first = dragHistory[0];
    const last = dragHistory[dragHistory.length - 1];
    if (first && last) {
      const dt = (last.t - first.t) / 1000;
      if (dt > 0.001) {
        Body.setVelocity(body, {
          x: (last.x - first.x) / dt,
          y: (last.y - first.y) / dt,
        });
      }
    }
    draggedBob = null;
    dragHistory = [];
  }

  canvas.addEventListener('pointerdown', onPointerDown);
  canvas.addEventListener('pointermove', onPointerMove);
  canvas.addEventListener('pointerup', onPointerUp);
  canvas.addEventListener('pointercancel', onPointerUp);

  function resetBodies() {
    Body.setPosition(bob1, { x: ANCHOR.x, y: ANCHOR.y + params.length1 });
    Body.setPosition(bob2, {
      x: ANCHOR.x,
      y: ANCHOR.y + params.length1 + params.length2,
    });
    Body.setVelocity(bob1, { x: 0, y: 0 });
    Body.setVelocity(bob2, { x: 0, y: 0 });
    Body.setAngularVelocity(bob1, 0);
    Body.setAngularVelocity(bob2, 0);
    trail = [];
    draggedBob = null;
    dragHistory = [];
  }

  function applyMass(body: Matter.Body, mass: number) {
    const newRadius = radiusForMass(mass);
    const oldRadius = body.circleRadius ?? newRadius;
    if (Math.abs(newRadius - oldRadius) > 0.01) {
      const scale = newRadius / oldRadius;
      Body.scale(body, scale, scale);
    }
    Body.setMass(body, mass);
  }

  const controlsHolder = document.createElement('div');
  controlsHolder.className = 'sim-controls';
  container.appendChild(controlsHolder);

  const pane = new Pane({ title: 'Double Pendulum', container: controlsHolder });

  pane.addBinding(params, 'length1', { min: 50, max: 250, step: 1, label: 'Length 1' })
    .on('change', (ev) => {
      rod1.length = ev.value;
    });
  pane.addBinding(params, 'length2', { min: 50, max: 250, step: 1, label: 'Length 2' })
    .on('change', (ev) => {
      rod2.length = ev.value;
    });
  pane.addBinding(params, 'mass1', { min: 1, max: 20, step: 0.5, label: 'Mass 1' })
    .on('change', (ev) => {
      applyMass(bob1, ev.value);
    });
  pane.addBinding(params, 'mass2', { min: 1, max: 20, step: 0.5, label: 'Mass 2' })
    .on('change', (ev) => {
      applyMass(bob2, ev.value);
    });
  pane.addBinding(params, 'damping', { min: 0, max: 0.05, step: 0.001, label: 'Damping' })
    .on('change', (ev) => {
      bob1.frictionAir = ev.value;
      bob2.frictionAir = ev.value;
    });
  pane.addBinding(params, 'showTrail', { label: 'Show trail' });
  pane.addButton({ title: 'Reset' }).on('click', resetBodies);

  const runner = Runner.create();
  Runner.run(runner, engine);

  let frameId = 0;

  function draw() {
    ctx.clearRect(0, 0, WIDTH, HEIGHT);

    if (params.showTrail) {
      trail.push({ x: bob2.position.x, y: bob2.position.y });
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

    ctx.beginPath();
    ctx.moveTo(ANCHOR.x, ANCHOR.y);
    ctx.lineTo(bob1.position.x, bob1.position.y);
    ctx.lineTo(bob2.position.x, bob2.position.y);
    ctx.strokeStyle = '#cfd3da';
    ctx.lineWidth = 3;
    ctx.stroke();

    ctx.fillStyle = '#cfd3da';
    ctx.beginPath();
    ctx.arc(ANCHOR.x, ANCHOR.y, 5, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = '#3b6ef5';
    ctx.beginPath();
    ctx.arc(bob1.position.x, bob1.position.y, bob1.circleRadius ?? 10, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = '#f5533b';
    ctx.beginPath();
    ctx.arc(bob2.position.x, bob2.position.y, bob2.circleRadius ?? 10, 0, Math.PI * 2);
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

export const doublePendulumSim: SimDefinition = {
  id: 'double-pendulum',
  title: 'Double Pendulum',
  mount,
};
