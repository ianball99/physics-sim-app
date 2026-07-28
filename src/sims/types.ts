export interface MountedSim {
  destroy(): void;
}

export interface SimDefinition {
  id: string;
  title: string;
  mount(container: HTMLElement): MountedSim;
}
