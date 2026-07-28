import './style.css';
import { sims } from './sims/registry';
import type { MountedSim } from './sims/types';

const app = document.getElementById('app')!;

const header = document.createElement('header');
header.className = 'app-header';
header.innerHTML = '<h1>Physics Sim Playground</h1>';

const nav = document.createElement('nav');
nav.className = 'sim-nav';
header.appendChild(nav);

const stage = document.createElement('main');
stage.className = 'sim-stage';

app.appendChild(header);
app.appendChild(stage);

let current: MountedSim | null = null;

function selectSim(id: string) {
  const sim = sims.find((s) => s.id === id) ?? sims[0];

  current?.destroy();
  stage.innerHTML = '';
  current = sim.mount(stage);

  for (const button of nav.querySelectorAll<HTMLButtonElement>('button')) {
    button.classList.toggle('active', button.dataset.simId === sim.id);
  }

  history.replaceState(null, '', `#${sim.id}`);
}

for (const sim of sims) {
  const button = document.createElement('button');
  button.textContent = sim.title;
  button.dataset.simId = sim.id;
  button.addEventListener('click', () => selectSim(sim.id));
  nav.appendChild(button);
}

const initialId = location.hash.replace('#', '') || sims[0].id;
selectSim(initialId);
