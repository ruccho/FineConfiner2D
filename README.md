# Fine Confiner 2D

English | [日本語](README.ja.md)

A Confiner that restricts the area that the camera can capture in 2D camera implementation using Cinemachine. It resolves several inconveniences of the built-in Confiner 2D in Cinemachine.

![](./docs/hero.png)

Instead of correcting the camera *after* damping (which lets the camera overshoot or jump at concave corners), Fine Confiner 2D

1. bakes the **region of valid camera centers** from the bounding shape (the polygon eroded by the camera window),
2. maps the **target** to a camera position in that region through the offset wavefront between the bounds and the region — a map that is *continuous*, unlike "nearest point", so the goal never jumps when the target rounds a concave corner — and
3. moves the camera toward that goal, sliding along the region boundary, so the camera never leaves the bounds and never jumps.

Baking runs on the .NET thread pool; per-frame queries are allocation-free.

<img width="640" height="360" alt="Image" src="https://github.com/user-attachments/assets/c56ccb90-b009-47e2-9702-970d0c732189" />

## Requirements

- Unity 6000.3 or later
- [Cinemachine](https://docs.unity3d.com/Packages/com.unity.cinemachine@3.1/manual/index.html) 3.1 or later
- [Clipper2](https://www.nuget.org/packages/Clipper2) 2.0 or later from NuGet (see Installation)

## Installation

### 1. Install Clipper2 from NuGet (required)

Fine Confiner 2D uses [Clipper2](https://github.com/AngusJohnson/Clipper2) (Boost Software License) for polygon offsetting. It is **not bundled**; install it from NuGet with [NuGetForUnity](https://github.com/GlitchEnzo/NuGetForUnity):

1. Add NuGetForUnity to your project (Package Manager → *Add package from git URL*):
   `https://github.com/GlitchEnzo/NuGetForUnity.git?path=/src/NuGetForUnity`
2. Open **NuGet → Manage NuGet Packages**, search for `Clipper2` and install it (2.0.0 or later).
   This places `Clipper2Lib.dll` under `Assets/Packages/Clipper2.<version>/lib/netstandard2.0/`.

### 2. Install Fine Confiner 2D

Package Manager → *Add package from git URL*:

```
https://github.com/ruccho/FineConfiner2D.git?path=/Packages/com.ruccho.fine-confiner-2d#release
```

## Quick Start

1. **Build the bounds.** Create a GameObject with a `CompositeCollider2D` (plus the `Rigidbody2D` it requires, set to *Static*) and add `BoxCollider2D` / `PolygonCollider2D` children with *Composite Operation* set to *Merge*. Mark the colliders as triggers if they should not collide with anything.
2. **Add a `FineConfiner`.** Put the `FineConfiner` component on any GameObject and assign the `CompositeCollider2D` to *Initial Shape*. The region is baked on a worker thread when the scene starts.

<img width="800" alt="Image" src="https://github.com/user-attachments/assets/18994d64-7bf0-4a20-b094-8bc1d997abf8" />

3. **Set up the camera.** On your `CinemachineCamera`, remove `CinemachinePositionComposer` and `CinemachineConfiner2D` (if present), add **Fine Confiner Body** (*Cinemachine → Procedural → Position Control*), assign the `FineConfiner` to *Confiner* and set the *Tracking Target*. The window size is derived from the lens automatically.
4. **Tune.** *Damping* controls how fast the camera travels; *Dead Zone*, *Hard Limits*, *Lookahead* and *Screen Position* behave like their Position Composer counterparts.

<img width="400" alt="Image" src="https://github.com/user-attachments/assets/87413ab4-01ab-4c91-ac24-6bebac52e1ea" />

Select the `FineConfiner` in the Scene view to see the baked region (green) and the path nodes (white dots).

### Designing the bounds

The camera window never leaves the bounds — that is the only rule, and it has one consequence: **every passage the camera has to travel through must be at least as large as the window** (width ≥ `2 × OrthographicSize × Aspect`, height ≥ `2 × OrthographicSize`). A passage narrower than the window splits the region into islands; the camera cannot cross between islands and simply waits at the edge of its island while the target goes out of view. `FineConfiner` logs a warning when this happens and draws each island in a different color.

Since the window width depends on the screen aspect ratio, design the bounds for the widest aspect you support (or lock the aspect with letterboxing).

### Changing the bounds at runtime

```csharp
// e.g. when a level section is loaded
confiner.SetShape(compositeCollider);      // starts an asynchronous bake
await confiner.WaitForBakeAsync();         // optional: wait until the new region is in effect
cinemachineCamera.ForceCameraPosition(     // optional: snap the camera after a teleport
    player.position, Quaternion.identity);
```

`SetShape(IReadOnlyList<Vector2[]>)` accepts arbitrary world-space polygons, and `ClearShape()` disables confining.

## Samples

**Basic Sample** — a top-down scene with an L-shaped room pair, a corridor and a tunnel, all sized to fit the camera window. Import it from Package Manager → *Fine Confiner 2D* → *Samples*, open `BasicSample.unity` and walk around with WASD / arrow keys.

## Components

- `FineConfiner` (MonoBehaviour) — holds the bounding shape (`SetShape(CompositeCollider2D)` / `SetShape(loops)`) and the window size, bakes a `ConfineRegion` asynchronously. `WaitForBakeAsync()` lets you delay level loading until the region is ready.
- `FineConfinerBody` (Cinemachine Body stage) — replaces `CinemachinePositionComposer` + `CinemachineConfiner2D`. Supports Lookahead, DeadZone, HardLimits and ScreenPosition with the same semantics as the Position Composer.
- `ConfineRegion` / `ConfinedFollower` / `RegionBaker` — engine-independent core (only depends on `UnityEngine` math types) for custom integrations.

## License

MIT
