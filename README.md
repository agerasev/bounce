# Bounce

An interactive rigid-body playground using `wgame` for rendering, `phy` for RK4
integration, and `geom2` for overlap geometry.

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

Losing focus pauses simulation; cursor exit, focus changes, resizing, and removing
the grabbed object release the drag. Reset is reproducible at the same window
size. The stepping and catch-up policy lives in `src/main.rs`.

Validate application changes by running it and checking the controls, contacts,
resizing, and both drawing modes. Automated regression tests belong in the
libraries when their behavior changes.
