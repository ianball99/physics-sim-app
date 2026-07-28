import type { SimDefinition } from './types';
import { doublePendulumSim } from './double-pendulum';
import { bouncingBallSim } from './bouncing-ball';
import { spinningBallSim } from './spinning-ball';
import { compoundPendulumSim } from './compound-pendulum';

// Add new simulations here as they're built.
export const sims: SimDefinition[] = [doublePendulumSim, bouncingBallSim, spinningBallSim, compoundPendulumSim];
