# Bounce

An interactive rigid-body playground using `wgame` for rendering, `phy` for RK4
integration, and `geom2` for pressure-field contact geometry.

```sh
git submodule update --init --recursive
cargo run --locked --release
```

The binary embeds its image and font, so it can run from any working directory.
The bundled DejaVu Sans font's license is in `assets/DejaVuSans-LICENSE.txt`.

## Web (WebGL2)

After initializing submodules, install the build tools and generate static files:

```sh
rustup target add wasm32-unknown-unknown
cargo install trunk --locked
./scripts/build-web.sh /bounce/
```

The script produces a release build in `dist/`, including JavaScript, WebAssembly,
and `.nojekyll` for GitHub Pages. Use the destination repository's path when
publishing elsewhere; omit the argument for relative URLs.
Publish the contents of `dist/` at the root of the destination's `gh-pages`
branch and select that branch's root in GitHub Pages settings.

For local development, run `NO_COLOR=true trunk serve --no-default-features
--features web` and open the printed URL. Desktop remains the default; do not
combine `desktop` and `web`. The browser needs WebGL2 and keyboard/mouse input;
click the canvas to focus the controls. Refresh after Escape to restart.

## Controls and simulation

- Left mouse: grab an object and pull at the selected point.
- `+` / `-`: add an object / remove a random object.
- `\`: switch between textures and debug outlines with force arrows.
- Space: pause or resume. `S`: toggle one-tenth speed independently of debug mode.
- `R`: reset eight objects using the same seed, preserving pause and speed settings.
- Escape or close the window: quit.

Losing focus pauses simulation; cursor exit, moving into the text header, focus
changes, resizing, and removing the grabbed object release the drag. Reset is
reproducible at the same window size. The stepping and catch-up policy lives in `src/main.rs`.

Contacts use quadratic radial pressure fields for circles and a continuous
four-triangle pressure fan for rectangles. Equal-pressure line/arc interfaces are
integrated analytically for elastic force **and torque**, using `f64` accumulators.
The peak pressure is 200; it is a material parameter, not the previous multiplier
of overlap area. Geometry, body/cell AABBs, and validated domain constraints are
rebuilt once per RK4 stage. Separated body boxes, cell boxes, and wall/cell boxes
are rejected before interface calculation. Disk pairs also retain an exact
center-distance check. Touching boxes are retained so shared cell boundaries
cannot lose contact contributions. Body pairs are still enumerated in their
original order; this does not introduce a spatial index or change the force law.

Damping and viscous friction use three positive quadrature weights per interface
piece. Their bounded, velocity-dependent tractions have nonpositive relative
power; equal/opposite forces and moments are applied to both bodies. The normal
and tangential factors are both 2.0 (inverse speed). Each dissipative component
uses `factor*v / (1 + abs(factor*v))`: it reaches half the local pressure at
speed 0.5 and remains bounded by that pressure. These stronger defaults shorten
bouncing and turn sliding balls toward rolling without softening the bodies
(peak pressure remains 200). This is smooth viscous friction, with no static
friction threshold. The elastic integral remains analytic.
Debug view shows each nonzero contact resultant as one force on its line of
action, preserving the simulation's integrated force and torque. Its application
point is chosen nearest a pressure-weighted contact center, estimated with the
same three-point quadrature used for dissipation. This anchor calculation runs
only in debug view; it does not change the simulated load. Pure torques,
including rotational air drag, are displayed as opposing force pairs.

Walls have an inward-depth linear pressure field. A body completely submerged in
a wall can lose its equal-pressure interface, so an additional center spring acts
when its center is outside the arena. Its elastic energy is `0.5*k*distance²`
(`k=4000`), with dissipative normal damping. This also recovers bodies displaced
outside by a resize; it does not teleport them or silently clamp their velocities.

Coincident cell fields contribute no invented interface normal; surrounding
interfaces and symmetric shared-edge weights still contribute. Exactly coincident
identical bodies can consequently remain coincident until disturbed. RK4 still
uses the fixed 240 Hz step; conservativity of the elastic contact law does not
imply exact finite-step energy conservation.

Run `cargo test --offline --release --lib` for game-integration checks, including
force/torque symmetry, dissipative power, wall recovery, dragging/resizing, the
64-body crowded seed, drop settling, and sliding-to-rolling behavior. Damped
impacts for radii 0.1–0.3 are also compared at 240 Hz and 960 Hz. The `geom2`
tests separately verify pressure integrals against
quadrature and independent energy derivatives. Also run the application to check
the controls, contacts, resizing, and both drawing modes.

## Contact performance

Run the reproducible, opt-in benchmark with:

```sh
cargo test --offline --release --lib benchmark_contact_evaluation -- --ignored --nocapture
```

It uses fixed seeded poses (including rotated rectangles) and zero-duration RK4
steps, preventing trajectory differences from changing the measured workload.
Times include preparing geometry/constraints and are divided by the four force
evaluations in each RK4 step. It reports the median of three runs; these are
physics timings, not rendering times or FPS.

On the development machine, before/after adding body/cell AABBs and cached domain
constraints:

| Scene | Before, µs/evaluation | After, µs/evaluation | Interface calls before → after |
|---|---:|---:|---:|
| 8 bodies, ordinary arena | 18.38 | 6.77 | 28 → 16 |
| 64 bodies, sparse | 94.31 | 50.95 | 134 → 61 |
| 64 bodies, packed | 7889.99 | 3527.85 | 10276 → 6048 |
| 256 bodies, sparse | 689.40 | 413.94 | 816 → 363 |

Pair enumeration remains O(n²). AABBs remove unnecessary narrow-phase work, but
cannot remove real contacts in a dense overlap. In particular, the packed
benchmark still exceeds the CPU budget for real-time 240 Hz stepping on this
machine. A spatial grid or sweep would address pair enumeration for larger sparse
scenes; heavily overlapping scenes remain dominated by contact calculations.
