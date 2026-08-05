import { Pane } from 'tweakpane';
import type { MountedSim, SimDefinition } from '../types';

const WIDTH = 640;
const HEIGHT = 640;
const ANCHOR = { x: WIDTH / 2, y: 70 };
const TRAIL_LENGTH = 400;
const GRAVITY = 1200; // px/s^2, tuned so a ~150px pendulum has a natural-feeling period
const FIXED_DT = 1 / 240; // physics substep -- see step() for why this is fixed
const MAX_FRAME_DT = 1 / 20; // clamp a stalled/backgrounded tab's catch-up burst
// Doubles as a rate limit on dragging (see applyDragForFrame) and a
// general safety net after every free RK4 step. Normal swings, even
// vigorous ones, stay well under this -- even at 60fps this allows ~48
// degrees of rotation per rendered frame while dragging, far more than any
// real hand or finger gesture produces.
const MAX_OMEGA = 50; // rad/s
// Time constant for smoothing the drag's finite-difference omega estimate
// before it's stored/fed to the other bob's equation of motion (see
// applyDragForFrame) -- filters out per-frame pointer-sampling noise
// without meaningfully delaying the response to real, sustained cursor
// motion.
const OMEGA_SMOOTHING_TAU = 0.05; // seconds

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
  damping: 0,
  showTrail: true,
};

interface State {
  theta1: number;
  omega1: number;
  theta2: number;
  omega2: number;
}

function radiusForMass(mass: number): number {
  return Math.min(40, 8 + Math.sqrt(mass) * 6);
}

// Same convention used throughout the app: angle 0 hangs straight down
// (screen y increases downward), positive angle rotates as seen on screen.
function endpointOffset(angle: number, length: number): { x: number; y: number } {
  return { x: -Math.sin(angle) * length, y: Math.cos(angle) * length };
}

function angleTowardTarget(pivot: { x: number; y: number }, target: { x: number; y: number }) {
  const dx = target.x - pivot.x;
  const dy = target.y - pivot.y;
  return Math.atan2(-dx, dy);
}

// Shortest signed angular difference b - a, in (-pi, pi], so a drag crossing
// the +-pi wraparound doesn't produce a spurious near-2pi delta.
function angleDiff(b: number, a: number): number {
  let d = (b - a) % (Math.PI * 2);
  if (d > Math.PI) d -= Math.PI * 2;
  if (d < -Math.PI) d += Math.PI * 2;
  return d;
}

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

// Closed-form double pendulum equations of motion (standard Lagrangian
// derivation; see e.g. Wikipedia's "Double pendulum" formal description).
// Point masses m1, m2 on massless rods of length L1, L2. This is the exact
// ODE, not an approximation from an iterative constraint solver -- there's
// no "rod stiffness" to tune because there's no rod body, just the angles.
function derivatives(s: State, m1: number, m2: number, L1: number, L2: number, damping: number): State {
  const delta = s.theta1 - s.theta2;
  const den = 2 * m1 + m2 - m2 * Math.cos(2 * delta);

  const domega1 =
    (-GRAVITY * (2 * m1 + m2) * Math.sin(s.theta1) -
      m2 * GRAVITY * Math.sin(s.theta1 - 2 * s.theta2) -
      2 * Math.sin(delta) * m2 * (s.omega2 * s.omega2 * L2 + s.omega1 * s.omega1 * L1 * Math.cos(delta))) /
      (L1 * den) -
    damping * s.omega1;

  const domega2 =
    (2 *
      Math.sin(delta) *
      (s.omega1 * s.omega1 * L1 * (m1 + m2) +
        GRAVITY * (m1 + m2) * Math.cos(s.theta1) +
        s.omega2 * s.omega2 * L2 * m2 * Math.cos(delta))) /
      (L2 * den) -
    damping * s.omega2;

  return { theta1: s.omega1, omega1: domega1, theta2: s.omega2, omega2: domega2 };
}

function addScaled(s: State, d: State, h: number): State {
  return {
    theta1: s.theta1 + d.theta1 * h,
    omega1: s.omega1 + d.omega1 * h,
    theta2: s.theta2 + d.theta2 * h,
    omega2: s.omega2 + d.omega2 * h,
  };
}

function rk4Step(s: State, dt: number, m1: number, m2: number, L1: number, L2: number, damping: number): State {
  const k1 = derivatives(s, m1, m2, L1, L2, damping);
  const k2 = derivatives(addScaled(s, k1, dt / 2), m1, m2, L1, L2, damping);
  const k3 = derivatives(addScaled(s, k2, dt / 2), m1, m2, L1, L2, damping);
  const k4 = derivatives(addScaled(s, k3, dt), m1, m2, L1, L2, damping);
  return {
    theta1: s.theta1 + (dt / 6) * (k1.theta1 + 2 * k2.theta1 + 2 * k3.theta1 + k4.theta1),
    omega1: s.omega1 + (dt / 6) * (k1.omega1 + 2 * k2.omega1 + 2 * k3.omega1 + k4.omega1),
    theta2: s.theta2 + (dt / 6) * (k1.theta2 + 2 * k2.theta2 + 2 * k3.theta2 + k4.theta2),
    omega2: s.omega2 + (dt / 6) * (k1.omega2 + 2 * k2.omega2 + 2 * k3.omega2 + k4.omega2),
  };
}

function mount(container: HTMLElement): MountedSim {
  const params: Params = { ...defaults };

  const canvas = document.createElement('canvas');
  canvas.width = WIDTH;
  canvas.height = HEIGHT;
  container.appendChild(canvas);

  const ctx = canvas.getContext('2d')!;

  let state: State = { theta1: 0, omega1: 0, theta2: 0, omega2: 0 };
  let trail: { x: number; y: number }[] = [];

  let draggedBob: DraggedBob = null;
  let lastPointer = { x: 0, y: 0 };

  function bob1Position() {
    const o = endpointOffset(state.theta1, params.length1);
    return { x: ANCHOR.x + o.x, y: ANCHOR.y + o.y };
  }

  function bob2Position(bob1: { x: number; y: number }) {
    const o = endpointOffset(state.theta2, params.length2);
    return { x: bob1.x + o.x, y: bob1.y + o.y };
  }

  // "Point the bob at the cursor" (angle from pivot to pointer) gives the
  // most intuitive, responsive tracking almost everywhere -- but it's
  // singular at the pivot itself: as the cursor nears it, an
  // infinitesimal cursor movement implies an enormous angular swing.
  // (Tried a tangent-projection approach instead of this to sidestep the
  // singularity entirely, but it goes the other way: a straight-line drag
  // along the bob's current radius direction -- e.g. dragging bob2 straight
  // up from its resting position -- has *zero* tangential component, so
  // the bob doesn't move no matter how far you drag. Worse, not better.)
  // The fix that keeps the good behavior everywhere else: rate-limit how
  // much theta is allowed to move toward the target in one frame. Far from
  // the pivot, real drag speeds never approach the limit, so it tracks the
  // cursor with no perceptible difference from a direct snap. Near the
  // pivot, where the target angle can swing wildly from one pointer sample
  // to the next, the actual angle only advances by the capped amount --
  // smoothly catching up as the drag continues, rather than snapping
  // through the chaotic region or (as the tangent approach did) freezing.
  //
  // The un-held bob is free to react to the dragged one, same as before --
  // that reaction is wanted (a real double pendulum's other segment does
  // follow along). What actually needs to be smooth is the *signal* that
  // reaction is driven by: state.omega for the dragged bob is a
  // finite-difference estimate from discrete pointer samples, which is
  // inherently noisier than a real continuous velocity, and the coupled
  // equation of motion squares it -- so raw per-frame noise in the drag's
  // omega got amplified into a visibly jerky reaction in the other bob,
  // which read as "elastic cord" even though theta itself (the dragged
  // bob's own position) was already tracking smoothly. Exponentially
  // smoothing omega toward each new raw sample, rather than snapping to
  // it, removes that amplified noise while still responding to real,
  // sustained cursor motion within about OMEGA_SMOOTHING_TAU.
  function applyDragForFrame(frameDt: number) {
    const maxStep = MAX_OMEGA * frameDt;
    const smoothing = 1 - Math.exp(-frameDt / OMEGA_SMOOTHING_TAU);

    if (draggedBob === 'bob1') {
      const target = angleTowardTarget(ANCHOR, lastPointer);
      const dtheta = clamp(angleDiff(target, state.theta1), -maxStep, maxStep);
      state.theta1 += dtheta;
      const rawOmega = frameDt > 0 ? dtheta / frameDt : 0;
      state.omega1 += (rawOmega - state.omega1) * smoothing;
    } else if (draggedBob === 'bob2') {
      const bob1 = bob1Position();
      const target = angleTowardTarget(bob1, lastPointer);
      const dtheta = clamp(angleDiff(target, state.theta2), -maxStep, maxStep);
      state.theta2 += dtheta;
      const rawOmega = frameDt > 0 ? dtheta / frameDt : 0;
      state.omega2 += (rawOmega - state.omega2) * smoothing;
    }
  }

  function stepFree(dt: number) {
    const next = rk4Step(state, dt, params.mass1, params.mass2, params.length1, params.length2, params.damping);
    if (draggedBob === 'bob1') {
      next.theta1 = state.theta1;
      next.omega1 = state.omega1;
    } else if (draggedBob === 'bob2') {
      next.theta2 = state.theta2;
      next.omega2 = state.omega2;
    }
    next.omega1 = clamp(next.omega1, -MAX_OMEGA, MAX_OMEGA);
    next.omega2 = clamp(next.omega2, -MAX_OMEGA, MAX_OMEGA);
    state = next;
  }

  function getPointerPos(e: PointerEvent): { x: number; y: number } {
    const rect = canvas.getBoundingClientRect();
    return {
      x: ((e.clientX - rect.left) / rect.width) * WIDTH,
      y: ((e.clientY - rect.top) / rect.height) * HEIGHT,
    };
  }

  function onPointerDown(e: PointerEvent) {
    const p = getPointerPos(e);
    const b1 = bob1Position();
    const b2 = bob2Position(b1);
    const d1 = Math.hypot(p.x - b1.x, p.y - b1.y);
    const d2 = Math.hypot(p.x - b2.x, p.y - b2.y);
    const reach1 = radiusForMass(params.mass1) * 2.5;
    const reach2 = radiusForMass(params.mass2) * 2.5;

    if (d1 <= reach1 && d1 <= d2) {
      draggedBob = 'bob1';
      // One-time snap so the grab feels precise -- safe because it's a
      // single assignment, not a continuous finite-difference through the
      // pivot singularity.
      state.theta1 = angleTowardTarget(ANCHOR, p);
      state.omega1 = 0;
    } else if (d2 <= reach2) {
      draggedBob = 'bob2';
      state.theta2 = angleTowardTarget(b1, p);
      state.omega2 = 0;
    } else {
      return;
    }

    canvas.setPointerCapture(e.pointerId);
    lastPointer = p;
  }

  function onPointerMove(e: PointerEvent) {
    if (!draggedBob) return;
    lastPointer = getPointerPos(e);
  }

  function onPointerUp() {
    // The drag's last finite-difference omega already carries over as the
    // release "fling" velocity -- nothing extra to compute.
    draggedBob = null;
  }

  canvas.addEventListener('pointerdown', onPointerDown);
  canvas.addEventListener('pointermove', onPointerMove);
  canvas.addEventListener('pointerup', onPointerUp);
  canvas.addEventListener('pointercancel', onPointerUp);

  function resetState() {
    state = { theta1: 0, omega1: 0, theta2: 0, omega2: 0 };
    trail = [];
    draggedBob = null;
  }

  const controlsHolder = document.createElement('div');
  controlsHolder.className = 'sim-controls';
  container.appendChild(controlsHolder);

  const pane = new Pane({ title: 'Double Pendulum', container: controlsHolder });

  // Changing a parameter mid-swing changes the kinetic energy implied by the
  // current angular velocities (e.g. KE scales with L^2*omega^2), which
  // isn't a real physical process -- settling velocity to zero keeps a
  // slider tweak from producing a surprise burst of motion.
  function onParamChange() {
    state.omega1 = 0;
    state.omega2 = 0;
  }

  pane.addBinding(params, 'length1', { min: 50, max: 250, step: 1, label: 'Length 1' }).on('change', onParamChange);
  pane.addBinding(params, 'length2', { min: 50, max: 250, step: 1, label: 'Length 2' }).on('change', onParamChange);
  pane.addBinding(params, 'mass1', { min: 1, max: 20, step: 0.5, label: 'Mass 1' }).on('change', onParamChange);
  pane.addBinding(params, 'mass2', { min: 1, max: 20, step: 0.5, label: 'Mass 2' }).on('change', onParamChange);
  pane.addBinding(params, 'damping', { min: 0, max: 0.5, step: 0.01, label: 'Damping' });
  pane.addBinding(params, 'showTrail', { label: 'Show trail' });
  pane.addButton({ title: 'Reset' }).on('click', resetState);

  let frameId = 0;
  let lastTime = performance.now();
  let accumulator = 0;

  function draw() {
    const b1 = bob1Position();
    const b2 = bob2Position(b1);

    ctx.clearRect(0, 0, WIDTH, HEIGHT);

    if (params.showTrail) {
      trail.push(b2);
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
    ctx.lineTo(b1.x, b1.y);
    ctx.lineTo(b2.x, b2.y);
    ctx.strokeStyle = '#cfd3da';
    ctx.lineWidth = 3;
    ctx.stroke();

    ctx.fillStyle = '#cfd3da';
    ctx.beginPath();
    ctx.arc(ANCHOR.x, ANCHOR.y, 5, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = '#3b6ef5';
    ctx.beginPath();
    ctx.arc(b1.x, b1.y, radiusForMass(params.mass1), 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = '#f5533b';
    ctx.beginPath();
    ctx.arc(b2.x, b2.y, radiusForMass(params.mass2), 0, Math.PI * 2);
    ctx.fill();
  }

  function tick(now: number) {
    const frameDt = Math.min(MAX_FRAME_DT, (now - lastTime) / 1000);
    lastTime = now;

    if (draggedBob) {
      // A held bob is a real-time interactive gesture, not something that
      // benefits from sub-frame precision -- update it once per rendered
      // frame and let the free bob take one correspondingly larger RK4 step.
      applyDragForFrame(frameDt);
      stepFree(frameDt);
      accumulator = 0;
    } else {
      // Free-swinging: step physics at a fixed rate decoupled from the
      // display's refresh rate. A double pendulum is chaotic -- integrating
      // with whatever variable dt the monitor happens to produce makes the
      // exact trajectory refresh-rate dependent, which a fixed-timestep
      // accumulator avoids, on top of just being more numerically accurate
      // per step than one large RK4 step per frame.
      accumulator += frameDt;
      while (accumulator >= FIXED_DT) {
        stepFree(FIXED_DT);
        accumulator -= FIXED_DT;
      }
    }

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

export const doublePendulumSim: SimDefinition = {
  id: 'double-pendulum',
  title: 'Double Pendulum',
  mount,
};
