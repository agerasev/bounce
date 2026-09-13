# Bounce

An interactive rigid-body playground using `wgame` for rendering, `phy` for RK4
integration, and `geom2` for pressure-field contact geometry.

```sh
git submodule update --init --recursive
cargo run --locked --release
```

The binary embeds its image and font, so it can run from any working directory.
The bundled DejaVu Sans font's license is in `assets/DejaVuSans-LICENSE.txt`.

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
of overlap area. Geometry is rebuilt once per RK4 stage, with a body bounding-circle
check before pairwise contact work.

Damping and viscous friction use three positive quadrature weights per interface
piece. Their bounded, velocity-dependent tractions have nonpositive relative
power; equal/opposite forces and moments are applied to both bodies. The normal
and tangential factors are 0.02 and 0.04. The elastic integral remains analytic.
Debug arrows include equivalent force couples for independent contact torques.

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
force/torque symmetry, dissipative power, wall recovery, dragging/resizing, and the
64-body crowded seed. The `geom2` tests separately verify pressure integrals against
quadrature and independent energy derivatives. Also run the application to check
the controls, contacts, resizing, and both drawing modes.
