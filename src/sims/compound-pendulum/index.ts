import { Pane } from 'tweakpane';
import type { MountedSim, SimDefinition } from '../types';

const WIDTH = 640;
const HEIGHT = 640;
const ANCHOR = { x: WIDTH / 2, y: 70 };
const TRAIL_LENGTH = 400;
const GRAVITY = 1200; // px/s^2, matches the point-mass double pendulum's tuning
const FIXED_DT = 1 / 240;
const MAX_FRAME_DT = 1 / 20;
const THICKNESS = 16;
const GRAB_PADDING = 16;
// Doubles as a rate limit on dragging (see applyDragForFrame) and a
// general safety net after every free RK4 step. Normal swings, even
// vigorous ones, stay well under this -- even at 60fps this allows ~48
// degrees of rotation per rendered frame while dragging, far more than any
// real hand or finger gesture produces.
const MAX_OMEGA = 50; // rad/s
// Time constant for smoothing the drag's finite-difference omega estimate
// before it's stored/fed to the other rod's equation of motion (see
// applyDragForFrame) -- filters out per-frame pointer-sampling noise
// without meaningfully delaying the response to real, sustained cursor
// motion.
const OMEGA_SMOOTHING_TAU = 0.05; // seconds
// rod2's pivot is rod1's live tip, which reacts to rod2 being dragged --
// unlike rod1's pivot (the fixed anchor), that reference point can move
// out from under the drag. Two mitigations, both fading in as the cursor
// nears the relevant pivot (a fraction of that rod's own length, so it
// scales with the length sliders):
//   1. The *targeting* pivot is a smoothed version of the live tip (rod2's
//      actual attachment point stays exact -- only the "where should I
//      aim" reference is filtered), so rod2's target angle isn't chasing
//      rod1's own frame-to-frame jitter.
//   2. How much of the dragged rod's omega is allowed to push the other
//      rod's equation of motion is scaled down near the pivot -- grabbing
//      right at a joint gives little leverage to impart torque through it,
//      physically, and this stops the other rod from getting a strong,
//      sustained push exactly when the drag signal is least reliable.
const COUPLING_FADE_FRACTION = 0.3;

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
  damping: 0,
  showTrail: true,
};

interface State {
  theta1: number;
  omega1: number;
  theta2: number;
  omega2: number;
}

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

// Closed-form double *compound* pendulum equations of motion: two uniform
// rigid rods (mass distributed along their length, not point masses),
// derived via Lagrangian mechanics. Unlike the point-mass double pendulum,
// each rod contributes both a center-of-mass term and its own moment of
// inertia about that center (I = m*L^2/12 for a uniform rod), and rod1's
// effective inertia about the anchor also carries rod2's full mass acting
// at its pivot point (distance L1), on top of rod1's own inertia.
//
//   r1, r2   = half-lengths (distance from each rod's pivot to its own CM)
//   I1, I2   = each rod's moment of inertia about its own CM
//   A        = effective inertia of rod1 about the anchor (theta1'' coefficient)
//   B        = effective inertia of rod2 about its own pivot (theta2'' coefficient)
//   C        = inertial coupling term between the two rods
//   D1, D2   = gravitational torque coefficients
//
// Derivation: T = (1/2)A*w1^2 + (1/2)B*w2^2 + C*w1*w2*cos(t1-t2),
// V = -D1*cos(t1) - D2*cos(t2). Applying the Euler-Lagrange equation to
// each of t1, t2 and solving the resulting 2x2 linear system for t1'', t2''
// gives the formulas below.
function derivatives(s: State, m1: number, m2: number, L1: number, L2: number, damping: number): State {
  const r1 = L1 / 2;
  const r2 = L2 / 2;
  const I1 = (m1 * L1 * L1) / 12;
  const I2 = (m2 * L2 * L2) / 12;

  const A = m1 * r1 * r1 + I1 + m2 * L1 * L1;
  const B = m2 * r2 * r2 + I2;
  const C = m2 * L1 * r2;
  const D1 = GRAVITY * (m1 * r1 + m2 * L1);
  const D2 = GRAVITY * m2 * r2;

  const delta = s.theta1 - s.theta2;
  const sinD = Math.sin(delta);
  const cosD = Math.cos(delta);
  const det = A * B - C * C * cosD * cosD;

  const domega1 =
    (-sinD * C * (B * s.omega2 * s.omega2 + C * cosD * s.omega1 * s.omega1) -
      B * D1 * Math.sin(s.theta1) +
      C * D2 * cosD * Math.sin(s.theta2)) /
      det -
    damping * s.omega1;

  const domega2 =
    (sinD * C * (A * s.omega1 * s.omega1 + C * cosD * s.omega2 * s.omega2) -
      A * D2 * Math.sin(s.theta2) +
      C * D1 * cosD * Math.sin(s.theta1)) /
      det -
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

  let draggedRod: DraggedRod = null;
  let lastPointer = { x: 0, y: 0 };
  let smoothedPivot = { x: 0, y: 0 }; // filtered rod1 tip, used only for rod2's targeting
  let couplingTaper = 1; // 0..1, how much of the drag's omega reaches the other rod

  function rod1Tip() {
    const o = endpointOffset(state.theta1, params.length1);
    return { x: ANCHOR.x + o.x, y: ANCHOR.y + o.y };
  }

  function rod2Tip(pivot: { x: number; y: number }) {
    const o = endpointOffset(state.theta2, params.length2);
    return { x: pivot.x + o.x, y: pivot.y + o.y };
  }

  // "Point the rod at the cursor" is the most intuitive tracking almost
  // everywhere, but singular at the pivot -- see the identical note in the
  // point-mass double pendulum for why a tangent-projection approach was
  // tried and rejected (it goes numb on straight-line drags along the
  // rod's current direction). Rate-limiting how far theta can move toward
  // the target per frame keeps the good tracking everywhere except right
  // at the pivot, where it smoothly catches up instead of jittering.
  //
  // The un-held rod is free to react to the dragged one, same as before --
  // that reaction is wanted (a real compound pendulum's other segment does
  // follow along). What actually needs to be smooth is the *signal* that
  // reaction is driven by: state.omega for the dragged rod is a
  // finite-difference estimate from discrete pointer samples, which is
  // inherently noisier than a real continuous velocity, and the coupled
  // equation of motion squares it -- so raw per-frame noise in the drag's
  // omega got amplified into a visibly jerky reaction in the other rod,
  // which read as "elastic cord" even though theta itself (the dragged
  // rod's own position) was already tracking smoothly. Exponentially
  // smoothing omega toward each new raw sample, rather than snapping to
  // it, removes that amplified noise while still responding to real,
  // sustained cursor motion within about OMEGA_SMOOTHING_TAU.
  function applyDragForFrame(frameDt: number) {
    const maxStep = MAX_OMEGA * frameDt;
    const smoothing = 1 - Math.exp(-frameDt / OMEGA_SMOOTHING_TAU);

    if (draggedRod === 'rod1') {
      const target = angleTowardTarget(ANCHOR, lastPointer);
      const dtheta = clamp(angleDiff(target, state.theta1), -maxStep, maxStep);
      state.theta1 += dtheta;
      const rawOmega = frameDt > 0 ? dtheta / frameDt : 0;
      state.omega1 += (rawOmega - state.omega1) * smoothing;

      const gripDist = Math.hypot(lastPointer.x - ANCHOR.x, lastPointer.y - ANCHOR.y);
      couplingTaper = clamp(gripDist / (params.length1 * COUPLING_FADE_FRACTION), 0, 1);
    } else if (draggedRod === 'rod2') {
      const livePivot = rod1Tip();
      smoothedPivot.x += (livePivot.x - smoothedPivot.x) * smoothing;
      smoothedPivot.y += (livePivot.y - smoothedPivot.y) * smoothing;

      const target = angleTowardTarget(smoothedPivot, lastPointer);
      const dtheta = clamp(angleDiff(target, state.theta2), -maxStep, maxStep);
      state.theta2 += dtheta;
      const rawOmega = frameDt > 0 ? dtheta / frameDt : 0;
      state.omega2 += (rawOmega - state.omega2) * smoothing;

      const gripDist = Math.hypot(lastPointer.x - livePivot.x, lastPointer.y - livePivot.y);
      couplingTaper = clamp(gripDist / (params.length2 * COUPLING_FADE_FRACTION), 0, 1);
    }
  }

  function stepFree(dt: number) {
    // The dragged rod's true omega always drives its own release velocity
    // and is what state ends up holding -- only the *coupling* seen by the
    // other rod's equation of motion is tapered, via a scaled-down copy
    // fed into this one RK4 step.
    const couplingState = draggedRod ? { ...state } : state;
    if (draggedRod === 'rod1') {
      couplingState.omega1 = state.omega1 * couplingTaper;
    } else if (draggedRod === 'rod2') {
      couplingState.omega2 = state.omega2 * couplingTaper;
    }

    const next = rk4Step(couplingState, dt, params.mass1, params.mass2, params.length1, params.length2, params.damping);
    if (draggedRod === 'rod1') {
      next.theta1 = state.theta1;
      next.omega1 = state.omega1;
    } else if (draggedRod === 'rod2') {
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
    const tip1 = rod1Tip();
    const tip2 = rod2Tip(tip1);

    const d1 = distanceToSegment(p, ANCHOR, tip1);
    const d2 = distanceToSegment(p, tip1, tip2);
    const reach = THICKNESS / 2 + GRAB_PADDING;

    if (d1 <= reach && d1 <= d2) {
      draggedRod = 'rod1';
      // One-time snap so the grab feels precise -- safe because it's a
      // single assignment, not a continuous finite-difference through the
      // pivot singularity.
      state.theta1 = angleTowardTarget(ANCHOR, p);
      state.omega1 = 0;
    } else if (d2 <= reach) {
      draggedRod = 'rod2';
      state.theta2 = angleTowardTarget(tip1, p);
      state.omega2 = 0;
      smoothedPivot = { ...tip1 };
    } else {
      return;
    }

    couplingTaper = 1;
    canvas.setPointerCapture(e.pointerId);
    lastPointer = p;
  }

  function onPointerMove(e: PointerEvent) {
    if (!draggedRod) return;
    lastPointer = getPointerPos(e);
  }

  function onPointerUp() {
    draggedRod = null;
  }

  canvas.addEventListener('pointerdown', onPointerDown);
  canvas.addEventListener('pointermove', onPointerMove);
  canvas.addEventListener('pointerup', onPointerUp);
  canvas.addEventListener('pointercancel', onPointerUp);

  function resetState() {
    state = { theta1: 0, omega1: 0, theta2: 0, omega2: 0 };
    trail = [];
    draggedRod = null;
  }

  // Both rods collinear with the anchor, extended horizontally to the
  // right (angle -pi/2 in this app's convention: endpointOffset(-pi/2, L)
  // = (+L, 0)). Releasing from rest here is the classic maximal-chaos
  // starting condition -- horizontal release converts the most possible
  // potential energy into motion.
  function resetToHorizontal() {
    state = { theta1: -Math.PI / 2, omega1: 0, theta2: -Math.PI / 2, omega2: 0 };
    trail = [];
    draggedRod = null;
  }

  const controlsHolder = document.createElement('div');
  controlsHolder.className = 'sim-controls';
  container.appendChild(controlsHolder);

  const pane = new Pane({ title: 'Compound Pendulum', container: controlsHolder });

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
  pane.addButton({ title: 'Release Horizontal' }).on('click', resetToHorizontal);

  let frameId = 0;
  let lastTime = performance.now();
  let accumulator = 0;

  function drawRod(center: { x: number; y: number }, angle: number, length: number, color: string) {
    ctx.save();
    ctx.translate(center.x, center.y);
    ctx.rotate(angle);
    ctx.fillStyle = color;
    ctx.fillRect(-THICKNESS / 2, -length / 2, THICKNESS, length);
    ctx.restore();
  }

  function draw() {
    const tip1 = rod1Tip();
    const tip2 = rod2Tip(tip1);
    const rod1Center = { x: ANCHOR.x + (tip1.x - ANCHOR.x) / 2, y: ANCHOR.y + (tip1.y - ANCHOR.y) / 2 };
    const rod2Center = { x: tip1.x + (tip2.x - tip1.x) / 2, y: tip1.y + (tip2.y - tip1.y) / 2 };

    ctx.clearRect(0, 0, WIDTH, HEIGHT);

    if (params.showTrail) {
      trail.push(tip2);
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

    drawRod(rod1Center, state.theta1, params.length1, '#3b6ef5');
    drawRod(rod2Center, state.theta2, params.length2, '#f5533b');

    ctx.fillStyle = '#cfd3da';
    ctx.beginPath();
    ctx.arc(ANCHOR.x, ANCHOR.y, 5, 0, Math.PI * 2);
    ctx.fill();

    ctx.beginPath();
    ctx.arc(tip1.x, tip1.y, 5, 0, Math.PI * 2);
    ctx.fill();
  }

  function tick(now: number) {
    const frameDt = Math.min(MAX_FRAME_DT, (now - lastTime) / 1000);
    lastTime = now;

    if (draggedRod) {
      applyDragForFrame(frameDt);
      stepFree(frameDt);
      accumulator = 0;
    } else {
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

export const compoundPendulumSim: SimDefinition = {
  id: 'compound-pendulum',
  title: 'Compound Pendulum',
  mount,
};
