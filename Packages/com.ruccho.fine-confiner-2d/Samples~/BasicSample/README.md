# Basic Sample

A top-down scene showing Fine Confiner 2D in action: an L-shaped pair of rooms, a corridor and a tunnel.

- **Bounds** — a `CompositeCollider2D` (geometry type *Outlines*) built from `BoxCollider2D` children. It serves both as the
  walls the player collides with and as the shape the `FineConfiner` bakes.
- **Player** — a `Rigidbody2D` driven by `SamplePlayerController` (WASD / arrow keys / left stick).
- **CinemachineCamera** — uses `FineConfinerBody` instead of Position Composer + Confiner 2D.

Open `BasicSample.unity`, press Play and walk into the corners, the corridor and the tunnel: the camera follows along
the baked region and never shows anything outside the bounds. Select the `FineConfiner` object in the Scene view to see
the region (green) and the path nodes (white).

Every passage is sized to fit the camera window (17.8 × 10 world units at 16:9). Try shrinking the `Corridor` box below
18 units wide: the region splits into two islands, `FineConfiner` logs a warning, and the camera waits at the mouth of the
corridor instead of following the player through it.
