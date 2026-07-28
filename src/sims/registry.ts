import type { SimDefinition } from './types';
import { doublePendulumSim } from './double-pendulum';
import { bouncingBallSim } from './bouncing-ball';

// Add new simulations here as they're built.
export const sims: SimDefinition[] = [doublePendulumSim, bouncingBallSim];
