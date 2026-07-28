import type { SimDefinition } from './types';
import { doublePendulumSim } from './double-pendulum';

// Add new simulations here as they're built.
export const sims: SimDefinition[] = [doublePendulumSim];
