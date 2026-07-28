import { Pane } from 'tweakpane';
import type { MountedSim, SimDefinition } from '../types';

const WIDTH = 640;
const HEIGHT = 640;
const RADIUS = 18;
const GRAVITY = 1600; // px/s^2
const MAX_DT = 1 / 30; // clamp to avoid tunneling after a stalled frame
const DRAG_HISTORY_SIZE = 6;

interface Params {
  restitutionFloor: number; // vertical-plane bounce, off floor/ceiling
  restitutionWalls: number; // horizontal-plane bounce, off left/right walls
  friction: number; // tangential velocity damping applied on contact
}

const defaults: Params = {
  restitutionFloor: 0.8,
  restitutionWalls: 0.8,
  friction: 0.1,
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

  function resetBall() {
    x = WIDTH / 2;
    y = HEIGHT / 3;
    vx = 0;
    vy = 0;
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
    dragHistory = [];
  }

  canvas.addEventListener('pointerdown', onPointerDown);
  canvas.addEventListener('pointermove', onPointerMove);
  canvas.addEventListener('pointerup', onPointerUp);
  canvas.addEventListener('pointercancel', onPointerUp);

  const controlsHolder = document.createElement('div');
  controlsHolder.className = 'sim-controls';
  container.appendChild(controlsHolder);

  const pane = new Pane({ title: 'Bouncing Ball', container: controlsHolder });
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
  pane.addBinding(params, 'friction', {
    min: 0,
    max: 1,
    step: 0.01,
    label: 'Surface friction',
  });
  pane.addButton({ title: 'Drop again' }).on('click', resetBall);

  function step(dt: number) {
    if (dragging) return;

    vy += GRAVITY * dt;
    x += vx * dt;
    y += vy * dt;

    if (y + RADIUS > HEIGHT) {
      y = HEIGHT - RADIUS;
      vy = -vy * params.restitutionFloor;
      vx *= 1 - params.friction;
    } else if (y - RADIUS < 0) {
      y = RADIUS;
      vy = -vy * params.restitutionFloor;
      vx *= 1 - params.friction;
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

  function draw() {
    ctx.clearRect(0, 0, WIDTH, HEIGHT);

    ctx.fillStyle = '#f5a83b';
    ctx.beginPath();
    ctx.arc(x, y, RADIUS, 0, Math.PI * 2);
    ctx.fill();
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

export const bouncingBallSim: SimDefinition = {
  id: 'bouncing-ball',
  title: 'Bouncing Ball',
  mount,
};
