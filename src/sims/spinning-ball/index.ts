import { Pane } from 'tweakpane';
import type { MountedSim, SimDefinition } from '../types';

const WIDTH = 640;
const HEIGHT = 640;
const RADIUS = 18;
const GRAVITY = 1600; // px/s^2
const MAX_DT = 1 / 30; // clamp to avoid tunneling after a stalled frame
const DRAG_HISTORY_SIZE = 6;
const MOMENT_COEFF = 2 / 5; // solid sphere: I = MOMENT_COEFF * m * R^2

interface Params {
  restitutionFloor: number; // normal (vertical) bounce, off floor/ceiling
  restitutionWalls: number; // normal (horizontal) bounce, off left/right walls
  restitutionTangential: number; // e_t: tangential (horizontal) restitution at floor/ceiling, once friction has enough grip
  friction: number; // Coulomb friction coefficient at floor/ceiling contact; also damps tangential speed on wall hits
  spin: number; // angular velocity (rad/s) applied to the ball the moment it's released from a drag
}

const defaults: Params = {
  restitutionFloor: 0.8,
  restitutionWalls: 0.8,
  restitutionTangential: 0.5,
  friction: 0.4,
  spin: 0,
};

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

function mount(container: HTMLElement): MountedSim {
  const params: Params = { ...defaults };

  const canvas = document.createElement('canvas');
  canvas.width = WIDTH;
  canvas.height = HEIGHT;
  container.appendChild(canvas);

  const ctx = canvas.getContext('2d')!;

  let x = WIDTH / 2;
  let y = HEIGHT / 3;
  let vx = 0;
  let vy = 0;
  let angularVelocity = 0; // rad/s, positive = clockwise on screen
  let angle = 0; // current rotation, purely visual

  function resetBall() {
    x = WIDTH / 2;
    y = HEIGHT / 3;
    vx = 0;
    vy = 0;
    angularVelocity = 0;
    angle = 0;
  }

  // Resolves a floor/ceiling bounce with coupled friction + spin, following the
  // standard rigid-body contact-impulse model: friction (bounded by the Coulomb
  // limit set by the normal impulse) tries to drive the contact-point slip
  // toward -e_t * slip (e_t = 0 means the ball grips and ends up rolling; e_t > 0
  // lets grip reverse some of the slip, like a superball's lively spin kick-back).
  // normalSign is +1 for the floor (contact point below center) and -1 for the
  // ceiling (contact point above center).
  function resolveFloorCeilingBounce(vyIncoming: number, normalSign: 1 | -1) {
    vy = -vyIncoming * params.restitutionFloor;

    const slip = vx - normalSign * RADIUS * angularVelocity;
    const jNormal = (1 + params.restitutionFloor) * Math.abs(vyIncoming);
    const jMax = params.friction * jNormal;
    const jStickTarget = (-slip * (1 + params.restitutionTangential) * MOMENT_COEFF) / (MOMENT_COEFF + 1);

    const j = Math.abs(jStickTarget) <= jMax ? jStickTarget : -Math.sign(slip) * jMax;

    vx += j;
    angularVelocity -= (normalSign * j) / (MOMENT_COEFF * RADIUS);
  }

  let dragging = false;
  let dragHistory: { x: number; y: number; t: number }[] = [];

  function getPointerPos(e: PointerEvent): { x: number; y: number } {
    const rect = canvas.getBoundingClientRect();
    return {
      x: ((e.clientX - rect.left) / rect.width) * WIDTH,
      y: ((e.clientY - rect.top) / rect.height) * HEIGHT,
    };
  }

  function onPointerDown(e: PointerEvent) {
    const p = getPointerPos(e);
    const dx = p.x - x;
    const dy = p.y - y;
    if (dx * dx + dy * dy <= (RADIUS * 2.5) ** 2) {
      dragging = true;
      canvas.setPointerCapture(e.pointerId);
      x = clamp(p.x, RADIUS, WIDTH - RADIUS);
      y = clamp(p.y, RADIUS, HEIGHT - RADIUS);
      vx = 0;
      vy = 0;
      angularVelocity = 0;
      dragHistory = [{ x, y, t: performance.now() }];
    }
  }

  function onPointerMove(e: PointerEvent) {
    if (!dragging) return;
    const p = getPointerPos(e);
    x = clamp(p.x, RADIUS, WIDTH - RADIUS);
    y = clamp(p.y, RADIUS, HEIGHT - RADIUS);
    dragHistory.push({ x, y, t: performance.now() });
    if (dragHistory.length > DRAG_HISTORY_SIZE) dragHistory.shift();
  }

  function onPointerUp() {
    if (!dragging) return;
    dragging = false;
    const first = dragHistory[0];
    const last = dragHistory[dragHistory.length - 1];
    if (first && last) {
      const dt = (last.t - first.t) / 1000;
      if (dt > 0.001) {
        vx = (last.x - first.x) / dt;
        vy = (last.y - first.y) / dt;
      }
    }
    angularVelocity = params.spin;
    dragHistory = [];
  }

  canvas.addEventListener('pointerdown', onPointerDown);
  canvas.addEventListener('pointermove', onPointerMove);
  canvas.addEventListener('pointerup', onPointerUp);
  canvas.addEventListener('pointercancel', onPointerUp);

  const controlsHolder = document.createElement('div');
  controlsHolder.className = 'sim-controls';
  container.appendChild(controlsHolder);

  const pane = new Pane({ title: 'Spinning Ball', container: controlsHolder });
  pane.addBinding(params, 'restitutionFloor', {
    min: 0,
    max: 1,
    step: 0.01,
    label: 'Restitution (floor/ceiling)',
  });
  pane.addBinding(params, 'restitutionWalls', {
    min: 0,
    max: 1,
    step: 0.01,
    label: 'Restitution (walls)',
  });
  pane.addBinding(params, 'restitutionTangential', {
    min: 0,
    max: 1,
    step: 0.01,
    label: 'Restitution (horiz., floor/ceiling)',
  });
  pane.addBinding(params, 'friction', {
    min: 0,
    max: 1,
    step: 0.01,
    label: 'Surface friction',
  });
  pane.addBinding(params, 'spin', {
    min: -25,
    max: 25,
    step: 0.5,
    label: 'Spin on release',
  });
  pane.addButton({ title: 'Drop again' }).on('click', resetBall);

  function step(dt: number) {
    if (dragging) return;

    vy += GRAVITY * dt;
    x += vx * dt;
    y += vy * dt;
    angle += angularVelocity * dt;

    if (y + RADIUS > HEIGHT) {
      y = HEIGHT - RADIUS;
      resolveFloorCeilingBounce(vy, 1);
    } else if (y - RADIUS < 0) {
      y = RADIUS;
      resolveFloorCeilingBounce(vy, -1);
    }

    if (x + RADIUS > WIDTH) {
      x = WIDTH - RADIUS;
      vx = -vx * params.restitutionWalls;
      vy *= 1 - params.friction;
    } else if (x - RADIUS < 0) {
      x = RADIUS;
      vx = -vx * params.restitutionWalls;
      vy *= 1 - params.friction;
    }
  }

  const SLICES = 8;
  const SLICE_COLORS = ['#f5a83b', '#c96f1e'];

  function draw() {
    ctx.clearRect(0, 0, WIDTH, HEIGHT);

    for (let i = 0; i < SLICES; i++) {
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.arc(x, y, RADIUS, angle + (i * Math.PI * 2) / SLICES, angle + ((i + 1) * Math.PI * 2) / SLICES);
      ctx.closePath();
      ctx.fillStyle = SLICE_COLORS[i % 2];
      ctx.fill();
    }

    ctx.beginPath();
    ctx.arc(x, y, RADIUS, 0, Math.PI * 2);
    ctx.strokeStyle = '#0b0d12';
    ctx.lineWidth = 1.5;
    ctx.stroke();
  }

  let lastTime = performance.now();
  let frameId = 0;

  function tick(now: number) {
    const dt = Math.min(MAX_DT, (now - lastTime) / 1000);
    lastTime = now;
    step(dt);
    draw();
    frameId = requestAnimationFrame(tick);
  }

  frameId = requestAnimationFrame(tick);

  return {
    destroy() {
      cancelAnimationFrame(frameId);
      canvas.removeEventListener('pointerdown', onPointerDown);
      canvas.removeEventListener('pointermove', onPointerMove);
      canvas.removeEventListener('pointerup', onPointerUp);
      canvas.removeEventListener('pointercancel', onPointerUp);
      pane.dispose();
    },
  };
}

export const spinningBallSim: SimDefinition = {
  id: 'spinning-ball',
  title: 'Spinning Ball',
  mount,
};
