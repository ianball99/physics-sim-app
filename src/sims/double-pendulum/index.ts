import Matter from 'matter-js';
import { Pane } from 'tweakpane';
import type { MountedSim, SimDefinition } from '../types';

const { Engine, Bodies, Composite, Constraint, Mouse, MouseConstraint, Runner, Body } = Matter;

const WIDTH = 640;
const HEIGHT = 640;
const ANCHOR = { x: WIDTH / 2, y: 70 };
const TRAIL_LENGTH = 400;

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

  const mouse = Mouse.create(canvas);
  const mouseConstraint = MouseConstraint.create(engine, {
    mouse,
    constraint: { stiffness: 0.2, render: { visible: false } },
  });
  Composite.add(engine.world, mouseConstraint);

  let trail: { x: number; y: number }[] = [];

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
