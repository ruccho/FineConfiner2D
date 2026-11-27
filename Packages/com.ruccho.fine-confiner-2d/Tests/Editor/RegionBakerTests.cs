using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

namespace FineConfiner2D.Tests
{
    public class RegionBakerTests
    {
        private static readonly Vector2 Window = new(4f, 2f); // 8x4 window

        private static Vector2[] Rect(float x0, float y0, float x1, float y1)
        {
            return new[] { new Vector2(x0, y0), new Vector2(x1, y0), new Vector2(x1, y1), new Vector2(x0, y1) };
        }

        private static void AssertPathInside(ConfineRegion region, Vector2 from, Vector2 to)
        {
            var ctx = new ConfineRegion.PathContext();
            var path = new List<Vector2>();
            var length = region.FindPath(from, to, ctx, path);
            Assert.IsFalse(float.IsPositiveInfinity(length), $"path {from} -> {to} unreachable");
            var prev = from;
            foreach (var wp in path)
            {
                Assert.IsTrue(region.IsSegmentInside(prev, wp, ctx), $"segment {prev} -> {wp} leaves region");
                prev = wp;
            }

            Assert.AreEqual(to, prev);
        }

        [Test]
        public void Rectangle_ErodesByHalfWindow()
        {
            var region = RegionBaker.Bake(new[] { Rect(0, 0, 20, 10) }, Window);
            Assert.AreEqual(1, region.Loops.Count);
            Assert.IsTrue(region.Contains(new Vector2(4, 2)));
            Assert.IsTrue(region.Contains(new Vector2(16, 8)));
            Assert.IsFalse(region.Contains(new Vector2(3.9f, 2)));
            Assert.IsFalse(region.Contains(new Vector2(10, 8.1f)));

            var c = region.Confine(new Vector2(-5, 20));
            Assert.AreEqual(4f, c.x, 1e-3f);
            Assert.AreEqual(8f, c.y, 1e-3f);

            // Reachable in a straight line
            AssertPathInside(region, new Vector2(4, 2), new Vector2(16, 8));
        }

        [Test]
        public void LShape_PathGoesAroundReflexCorner()
        {
            // L shape: horizontal bar (0,0)-(30,10) and vertical bar (0,0)-(10,30)
            var region = RegionBaker.Bake(new[] { Rect(0, 0, 30, 10), Rect(0, 0, 10, 30) }, Window);
            Assert.GreaterOrEqual(region.Nodes.Count, 1, "the reflex vertex becomes a node");

            var a = new Vector2(25, 5); // end of the horizontal bar
            var b = new Vector2(5, 25); // end of the vertical bar
            Assert.IsTrue(region.Contains(a));
            Assert.IsTrue(region.Contains(b));

            var ctx = new ConfineRegion.PathContext();
            Assert.IsFalse(region.IsSegmentInside(a, b, ctx), "the straight line leaves the region");
            AssertPathInside(region, a, b);

            var path = new List<Vector2>();
            region.FindPath(a, b, ctx, path);
            Assert.AreEqual(2, path.Count, "goes through one reflex vertex");
            Assert.AreEqual(new Vector2(6, 8), path[0]);
        }

        [Test]
        public void UShape_PathGoesAroundBothCorners()
        {
            // Bottom (0,0)-(40,10), left arm (0,0)-(10,40), right arm (30,0)-(40,40)
            var region = RegionBaker.Bake(
                new[] { Rect(0, 0, 40, 10), Rect(0, 0, 10, 40), Rect(30, 0, 40, 40) }, Window);
            var a = new Vector2(5, 35);
            var b = new Vector2(35, 35);
            AssertPathInside(region, a, b);
            var ctx = new ConfineRegion.PathContext();
            var path = new List<Vector2>();
            region.FindPath(a, b, ctx, path);
            Assert.AreEqual(3, path.Count, "goes through two reflex vertices");
        }

        [Test]
        public void NarrowShaft_YieldsNoRegion()
        {
            // Room (0,0)-(20,10) with a shaft above it: width 4 (narrower than the 8-wide window), height 20
            var region = RegionBaker.Bake(new[] { Rect(0, 0, 20, 10), Rect(8, 10, 12, 30) }, Window);
            Assert.AreEqual(1, RegionBaker.AssignIslands(region.SourceLoops, new int[region.SourceLoops.Count]),
                "the bounds are one connected polygon");
            Assert.AreEqual(1, region.IslandCount,
                "the shaft is too narrow for the window, so it yields no region at all");

            // Points inside the shaft are confined to the mouth of the room
            var c = region.Confine(new Vector2(9, 20));
            Assert.AreEqual(9f, c.x, 1e-2f);
            Assert.AreEqual(8f, c.y, 1e-2f);
        }

        [Test]
        public void NarrowOpening_SplitsRegionIntoIslands_AndFollowerWaits()
        {
            // Two rooms joined by an opening of height 2 (lower than the 4-high window)
            var region = RegionBaker.Bake(
                new[] { Rect(0, 0, 20, 10), Rect(20, 4, 40, 6), Rect(40, 0, 60, 10) }, Window);
            Assert.AreEqual(2, region.IslandCount);
            Assert.AreEqual(1, RegionBaker.AssignIslands(region.SourceLoops, new int[region.SourceLoops.Count]));

            var ctx = new ConfineRegion.PathContext();
            var path = new List<Vector2>();
            Assert.IsTrue(float.IsPositiveInfinity(region.FindPath(new Vector2(10, 5), new Vector2(50, 5), ctx, path)));
            Assert.AreEqual(0, path.Count);

            // The follower waits instead of cutting through the wall
            var follower = new ConfinedFollower();
            follower.SetRegion(region);
            follower.Teleport(new Vector2(10, 5));
            var remaining = follower.MoveToward(new Vector2(50, 5), 1f);
            Assert.IsFalse(follower.IsGoalReachable);
            Assert.IsTrue(float.IsPositiveInfinity(remaining));
            Assert.AreEqual(new Vector2(10, 5), follower.Position);
        }

        [Test]
        public void Follower_StaysInsideWhileMoving()
        {
            var region = RegionBaker.Bake(new[] { Rect(0, 0, 30, 10), Rect(0, 0, 10, 30) }, Window);
            var follower = new ConfinedFollower();
            follower.SetRegion(region);
            follower.Teleport(new Vector2(25, 5));
            var target = new Vector2(5, 25);
            var guard = 0;
            while (follower.MoveToward(target, 1.5f) > 0f && guard++ < 200)
                Assert.IsTrue(region.Contains(follower.Position), $"left region at {follower.Position}");
            Assert.Less(guard, 200);
            Assert.AreEqual(target, follower.Position);
        }

        [Test]
        public void LargeCoordinates_DiagonalEdge_SegmentsAlongEdgeAreInside()
        {
            // Reproduces a region that oscillated in the game (coordinates ~870, with a diagonal edge).
            // The window is kept small so the diagonal edge survives erosion.
            var loop = new[]
            {
                new Vector2(1016.38f, -159.74f), new Vector2(1016.38f, -118.17f), new Vector2(882.44f, -118.17f),
                new Vector2(882.44f, -115.31f), new Vector2(874.49f, -115.31f), new Vector2(874.49f, -125.26f),
                new Vector2(785.21f, -125.26f), new Vector2(785.21f, -139.66f), new Vector2(865.83f, -139.66f),
                new Vector2(869.34f, -142.84f), new Vector2(881.05f, -142.84f), new Vector2(881.05f, -159.74f)
            };
            var region = RegionBaker.Bake(new[] { loop }, new Vector2(0.5f, 0.5f));
            var ctx = new ConfineRegion.PathContext();

            // Find the diagonal edge
            Vector2 a = default, b = default;
            var found = false;
            foreach (var l in region.Loops)
                for (var i = 0; i < l.Length; i++)
                {
                    var d = l[(i + 1) % l.Length] - l[i];
                    if (Mathf.Abs(d.x) > 1f && Mathf.Abs(d.y) > 1f)
                    {
                        a = l[i];
                        b = l[(i + 1) % l.Length];
                        found = true;
                    }
                }

            Assert.IsTrue(found, "diagonal edge expected");

            var dir = (b - a).normalized;
            var len = (b - a).magnitude;
            var failures = 0;
            var total = 0;
            for (var t0 = 0.05f; t0 < len; t0 += 0.137f)
            for (var t1 = t0 + 0.2f; t1 < len; t1 += 0.211f)
            {
                var p = region.Confine(a + dir * t0);
                var q = region.Confine(a + dir * t1);
                total++;
                if (!region.IsSegmentInside(p, q, ctx)) failures++;
            }

            Assert.AreEqual(0, failures, $"{failures}/{total} edge segments judged outside");

            // Moving toward a goal on the edge never goes backwards (the path is always a straight line)
            var follower = new ConfinedFollower();
            follower.SetRegion(region);
            follower.Teleport(a + dir * 0.3f);
            var goal = a + dir * (len - 0.3f);
            var prevRemaining = float.MaxValue;
            for (var i = 0; i < 200; i++)
            {
                var remaining = follower.MoveToward(goal, 0.05f);
                Assert.LessOrEqual(remaining, prevRemaining + 1e-3f, $"moved backwards at step {i}");
                Assert.AreEqual(1, follower.LastPath.Count, $"detour at step {i}");
                prevRemaining = remaining;
                if (remaining <= 0f) break;
            }
        }

        [Test]
        public void Confine_PrefersPointsThatKeepTargetOnScreen()
        {
            // The sample layout: corridor (x -10..10, y 10..34) into Room C (x -17..17, y 33..47), 16:9 window.
            var hw = new Vector2(5f * 16f / 9f, 5f);
            var region = RegionBaker.Bake(new[] { Rect(-10, 10, 10, 34), Rect(-17, 33, 17, 47) }, hw);

            // Player hugging the corridor's right wall. The Euclidean nearest region point is Room C's corner
            // (8.1, 38) — 6.9 above, off screen. The corridor point (1.1, 31.1) keeps the player on screen.
            var target = new Vector2(8.5f, 31.1f);
            var c = region.Confine(target);
            Assert.LessOrEqual(Mathf.Abs(c.x - target.x), hw.x + 1e-3f,
                $"confined to {c}: target off screen horizontally");
            Assert.LessOrEqual(Mathf.Abs(c.y - target.y), hw.y + 1e-3f,
                $"confined to {c}: target off screen vertically");
            Assert.IsTrue(region.Contains(c));
        }

        [Test]
        public void Slide_FollowsWallsAroundReflexCorner()
        {
            // L: arm A is x 4..26, y 2..8 and arm B is x 4..6, y 2..28 after erosion; reflex corner at (6, 8)
            var region = RegionBaker.Bake(new[] { Rect(0, 0, 30, 10), Rect(0, 0, 10, 30) }, Window);

            // Diagonal move into the top wall of arm A: stops at the wall and slides along it
            var p1 = region.Slide(new Vector2(20, 7), new Vector2(-10, 5));
            Assert.AreEqual(10f, p1.x, 1e-3f);
            Assert.AreEqual(8f, p1.y, 1e-3f);
            Assert.IsTrue(region.Contains(p1));

            // Continuing along the wall passes the reflex corner into arm B and stops at its far wall
            var p2 = region.Slide(p1, new Vector2(-8, 0));
            Assert.AreEqual(4f, p2.x, 1e-3f);
            Assert.AreEqual(8f, p2.y, 1e-3f);
            Assert.IsTrue(region.Contains(p2));

            // A move never travels farther than requested and never leaves the region
            var pos = new Vector2(20, 7);
            for (var i = 0; i < 200; i++)
            {
                var delta = new Vector2(-0.2f, 0.15f);
                var next = region.Slide(pos, delta);
                Assert.LessOrEqual((next - pos).magnitude, delta.magnitude + 1e-3f, $"step {i} moved too far");
                Assert.IsTrue(region.Contains(next), $"step {i} left the region at {next}");
                pos = next;
            }
        }

        private static float ScaledChebyshev(Vector2 a, Vector2 b, Vector2 hw)
        {
            return Mathf.Max(Mathf.Abs(a.x - b.x) / hw.x, Mathf.Abs(a.y - b.y) / hw.y);
        }

        [Test]
        public void MapTarget_Rectangle_ProjectsAlongNormalsAndConvergesAtCorners()
        {
            var region = RegionBaker.Bake(new[] { Rect(0, 0, 20, 10) }, Window);
            var m = region.MapTarget(new Vector2(10, 1)); // near the bottom wall
            Assert.AreEqual(10f, m.x, 1e-3f);
            Assert.AreEqual(2f, m.y, 1e-3f);
            m = region.MapTarget(new Vector2(2, 1)); // on the corner bisector (window-scaled)
            Assert.AreEqual(4f, m.x, 1e-3f);
            Assert.AreEqual(2f, m.y, 1e-3f);
            m = region.MapTarget(new Vector2(10, 5)); // inside the region: identity
            Assert.AreEqual(new Vector2(10, 5), m);
        }

        [Test]
        public void MapTarget_IsContinuousAroundReflexCorner_AndKeepsTargetOnScreen()
        {
            // L: horizontal bar (0,0)-(30,10), vertical bar (0,0)-(10,30). Inner (reflex) corner of the bounds at (10,10).
            var region = RegionBaker.Bake(new[] { Rect(0, 0, 30, 10), Rect(0, 0, 10, 30) }, Window);

            // Walk around the inner corner at several distances from the walls, crossing the medial axis
            foreach (var gap in new[] { 0.5f, 1.5f, 3f })
            {
                var prev = region.MapTarget(new Vector2(25, 10 - gap));
                var maxRatio = 0f;
                for (var i = 1; i <= 400; i++)
                {
                    // path: along the bottom arm toward the corner, then up the vertical arm
                    var u = i / 400f * 2f; // 0..2
                    var t = u < 1f
                        ? new Vector2(Mathf.Lerp(25f, 10 - gap, u), 10 - gap)
                        : new Vector2(10 - gap, Mathf.Lerp(10 - gap, 25f, u - 1f));
                    var m = region.MapTarget(t);
                    Assert.IsTrue(region.Contains(m), $"gap {gap} step {i}: {m} outside region");
                    Assert.LessOrEqual(ScaledChebyshev(t, m, Window), 1f + 1e-3f,
                        $"gap {gap} step {i}: target off screen");
                    var dt = u < 1f ? (25f - (10 - gap)) / 400f * 2f : (25f - (10 - gap)) / 400f * 2f;
                    maxRatio = Mathf.Max(maxRatio, (m - prev).magnitude / dt);
                    prev = m;
                }

                Assert.Less(maxRatio, 4f,
                    $"gap {gap}: goal moved {maxRatio:F1}x faster than the target (discontinuity)");
            }
        }

        [Test]
        public void MapTarget_Rectangle_ClampsAxisWiseNearWallsAndCorners()
        {
            // Region of camera centers: [4,16] x [2,8]. Outside it, the camera follows the target along the walls
            // and reaches the corners, like an axis-wise clamp.
            var region = RegionBaker.Bake(new[] { Rect(0, 0, 20, 10) }, Window);
            for (var x = 0.25f; x < 20f; x += 0.5f)
            for (var y = 0.25f; y < 10f; y += 0.5f)
            {
                var t = new Vector2(x, y);
                var m = region.MapTarget(t);
                var expected = new Vector2(Mathf.Clamp(x, 4f, 16f), Mathf.Clamp(y, 2f, 8f));
                Assert.AreEqual(expected.x, m.x, 2e-3f, $"target {t}: mapped to {m}");
                Assert.AreEqual(expected.y, m.y, 2e-3f, $"target {t}: mapped to {m}");
            }

            var c = region.MapTarget(new Vector2(6, 1));
            Assert.AreEqual(6f, c.x, 1e-3f);
            Assert.AreEqual(2f, c.y, 1e-3f);
        }

        [Test]
        public void MapTarget_LShape_FollowsWallsAwayFromReflexCorner()
        {
            // Inner (reflex) corner of the bounds at (10,10); the region's reflex corner is at (6,8).
            var region = RegionBaker.Bake(new[] { Rect(0, 0, 30, 10), Rect(0, 0, 10, 30) }, Window);

            // Far from the reflex corner, the target near the ceiling of the horizontal arm is followed along it
            var m = region.MapTarget(new Vector2(20, 9.5f));
            Assert.AreEqual(20f, m.x, 1e-3f);
            Assert.AreEqual(8f, m.y, 1e-3f);

            // Near the floor at the far end: the convex corner is reached
            m = region.MapTarget(new Vector2(29, 0.5f));
            Assert.AreEqual(26f, m.x, 1e-3f);
            Assert.AreEqual(2f, m.y, 1e-3f);
        }

        [TestCaseSource(nameof(WellBehavedShapes))]
        public void MapTarget_IsContinuousAndKeepsTargetOnScreen(Vector2[][] loops)
        {
            var region = RegionBaker.Bake(loops, Window);
            Assert.AreEqual(1, region.IslandCount);

            var min = new Vector2(float.PositiveInfinity, float.PositiveInfinity);
            var max = new Vector2(float.NegativeInfinity, float.NegativeInfinity);
            foreach (var loop in loops)
            foreach (var p in loop)
            {
                min = Vector2.Min(min, p);
                max = Vector2.Max(max, p);
            }

            const float step = 0.05f;
            // Scan lines in both directions; consecutive samples inside the bounds must map close to each other
            for (var pass = 0; pass < 2; pass++)
            {
                var lines = pass == 0 ? max.y - min.y : max.x - min.x;
                var along = pass == 0 ? max.x - min.x : max.y - min.y;
                for (var l = 0.37f; l < lines; l += 0.9f)
                {
                    var hasPrev = false;
                    var prev = Vector2.zero;
                    for (var k = 0f; k <= along; k += step)
                    {
                        var t = pass == 0 ? new Vector2(min.x + k, min.y + l) : new Vector2(min.x + l, min.y + k);
                        if (!InsideAny(loops, t))
                        {
                            hasPrev = false;
                            continue;
                        }

                        var m = region.MapTarget(t);
                        Assert.IsTrue(region.Contains(m), $"target {t}: {m} outside region");
                        // Wherever some window inside the bounds shows the target, the mapped camera must show it
                        if (ScaledChebyshev(t, region.Confine(t), Window) <= 1f)
                            Assert.LessOrEqual(ScaledChebyshev(t, m, Window), 1f + 1e-3f,
                                $"target {t}: off screen from {m}");
                        if (hasPrev)
                            Assert.Less((m - prev).magnitude, 4f * step + 1e-3f,
                                $"target {t}: jump from {prev} to {m}");
                        prev = m;
                        hasPrev = true;
                    }
                }
            }
        }

        private static IEnumerable<TestCaseData> WellBehavedShapes()
        {
            yield return new TestCaseData(new object[] { new[] { Rect(0, 0, 20, 10) } }).SetName("{m}(Rectangle)");
            yield return new TestCaseData(new object[] { new[] { Rect(0, 0, 30, 10), Rect(0, 0, 10, 30) } })
                .SetName("{m}(LShape)");
            yield return new TestCaseData(new object[] { ConvexWithDiagonals }).SetName("{m}(ConvexWithDiagonals)");
            yield return new TestCaseData(new object[] { DiagonalReflexCorner }).SetName("{m}(DiagonalReflexCorner)");
        }

        // Convex, with diagonal edges of several slopes and sharp vertices
        private static readonly Vector2[][] ConvexWithDiagonals =
        {
            new[]
            {
                new Vector2(0, 0), new Vector2(40, 0), new Vector2(48, 16), new Vector2(20, 30), new Vector2(-6, 12)
            }
        };

        // Reflex corner between a horizontal and a diagonal edge
        private static readonly Vector2[][] DiagonalReflexCorner =
        {
            new[]
            {
                new Vector2(0, 0), new Vector2(40, 0), new Vector2(40, 10), new Vector2(25, 10),
                new Vector2(15, 25), new Vector2(0, 25)
            }
        };

        [TestCaseSource(nameof(WellBehavedShapes))]
        public void Region_WindowStaysInsideBoundsAndTouchesThem(Vector2[][] loops)
        {
            var region = RegionBaker.Bake(loops, Window);
            Assert.IsFalse(region.IsEmpty);
            var inset = new Vector2(Window.x - 0.01f, Window.y - 0.01f);
            var outset = new Vector2(Window.x + 0.05f, Window.y + 0.05f);
            foreach (var loop in region.Loops)
                for (int i = 0, j = loop.Length - 1; i < loop.Length; j = i++)
                    for (var k = 0; k <= 10; k++)
                    {
                        var c = Vector2.Lerp(loop[j], loop[i], k / 10f);
                        // The window (slightly shrunk) fits inside the bounds at every point of the region...
                        foreach (var p in WindowPerimeter(c, inset))
                            Assert.IsTrue(InsideAny(loops, p), $"window at {c} leaves the bounds at {p}");
                        // ...and, on the region's boundary, a slightly larger window does not: the region is not
                        // smaller than the exact erosion
                        var touches = false;
                        foreach (var p in WindowPerimeter(c, outset))
                            if (!InsideAny(loops, p))
                                touches = true;
                        Assert.IsTrue(touches, $"window at {c} on the region boundary does not touch the bounds");
                    }
        }

        /// <summary>Points along the border of the window of the given half size (corners included).</summary>
        private static IEnumerable<Vector2> WindowPerimeter(Vector2 center, Vector2 half)
        {
            const int perSide = 100;
            for (var i = 0; i < perSide; i++)
            {
                var u = Mathf.Lerp(-1f, 1f, i / (float)perSide);
                yield return center + Vector2.Scale(half, new Vector2(u, -1f));
                yield return center + Vector2.Scale(half, new Vector2(1f, u));
                yield return center + Vector2.Scale(half, new Vector2(-u, 1f));
                yield return center + Vector2.Scale(half, new Vector2(-1f, -u));
            }
        }

        /// <summary>Whether p is strictly inside any of the (possibly overlapping) loops.</summary>
        private static bool InsideAny(Vector2[][] loops, Vector2 p)
        {
            foreach (var loop in loops)
            {
                var inside = false;
                for (int i = 0, j = loop.Length - 1; i < loop.Length; j = i++)
                    if ((loop[i].y > p.y) != (loop[j].y > p.y) &&
                        p.x < (loop[j].x - loop[i].x) * (p.y - loop[i].y) / (loop[j].y - loop[i].y) + loop[i].x)
                        inside = !inside;
                if (inside) return true;
            }

            return false;
        }

        [Test]
        public void MapTarget_HandlesVanishingNotch()
        {
            // Rectangle with a small notch in the bottom wall (2 wide, 1 deep): the notch edges vanish under erosion
            var region = RegionBaker.Bake(new[] { Rect(0, 0, 20, 10), Rect(9, -1, 11, 0.5f) }, Window);
            Assert.AreEqual(1, region.IslandCount);
            var prev = region.MapTarget(new Vector2(5, 0.5f));
            for (var i = 1; i <= 200; i++)
            {
                var t = new Vector2(Mathf.Lerp(5f, 15f, i / 200f), i % 2 == 0 ? 0.5f : -0.5f);
                if (t.y < 0f && (t.x < 9f || t.x > 11f)) continue; // outside the bounds
                var m = region.MapTarget(t);
                Assert.IsTrue(region.Contains(m), $"step {i}: {m} outside region");
                // Inside the notch (narrower than the window) no camera position can show the target; elsewhere it must
                if (t.y >= 0f)
                    Assert.LessOrEqual(ScaledChebyshev(t, m, Window), 1f + 1e-3f, $"step {i}: target off screen");
                Assert.Less((m - prev).magnitude, 1.5f, $"step {i}: jump from {prev} to {m}");
                prev = m;
            }
        }
    }
}