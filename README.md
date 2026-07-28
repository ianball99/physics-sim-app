# Physics Sim Playground

Interactive physics simulations with live parameter controls, built with
[Matter.js](https://brm.io/matter-js/) (2D physics) and
[Tweakpane](https://tweakpane.github.io/docs/) (parameter panel).

This app is intentionally separate from the chatbot project — different repo,
different Netlify site, no shared config.

## Simulations

- **Double Pendulum** — two rods connected by constraints, hanging from a
  fixed anchor. Sliders control both rod lengths and both bob masses. Drag
  either bob with a mouse or a finger (touch) to kick it into motion. Starts
  at rest hanging straight down (a stable equilibrium) — drag it to see the
  chaotic swing.

New simulations get added as entries in `src/sims/registry.ts`, each living in
its own folder under `src/sims/<sim-name>/` and implementing the
`SimDefinition` interface in `src/sims/types.ts` (a `mount(container)` that
returns a `destroy()` cleanup function).

## Local development

```
npm install
npm run dev       # start dev server
npm run build     # type-check + production build to dist/
npm run preview   # serve the production build locally
```

## Deploying to Netlify

This repo is not yet connected to a Netlify site. To set one up (mirrors the
chatbot repo's setup — branch deploys enabled, production tracks `main`):

1. Netlify dashboard → **Add new site → Import an existing project**
2. Pick this repo (`physics-sim-app`)
3. Build command: `npm run build`, publish directory: `dist` (already set in
   `netlify.toml`, Netlify should detect these automatically)
4. Site configuration → Build & deploy → Continuous deployment → **Branch
   deploys** → enable "All" so every pushed branch gets its own preview URL
5. Production URL tracks `main`; branch deploys use
   `https://<branch-slug>--<site-name>.netlify.app`

No GitHub Actions workflow is needed — Netlify's own git integration builds
and deploys on every push once connected.
