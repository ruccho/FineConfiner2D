using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using Clipper2Lib;
using UnityEngine;

namespace FineConfiner2D
{
    /// <summary>
    ///     Bakes a <see cref="ConfineRegion" /> from bounding polygons and a window size.
    ///     Pure computation with no main-thread-only Unity API, so it can run on a worker thread.
    /// </summary>
    public static class RegionBaker
    {
        /// <summary>Integer scale passed to Clipper (1.0 in window space = 1000).</summary>
        private const double IntScale = 1000.0;

        private const float Tiny = 1e-3f;

        /// <summary>
        ///     Runs the bake on the thread pool. The input is copied at the time of this call.
        /// </summary>
        /// <param name="worldLoops">Bounding polygons in world space. Multiple closed loops; overlapping loops are unioned.</param>
        /// <param name="halfWindow">Half size of the camera window (world units).</param>
        public static Task<ConfineRegion> BakeAsync(
            IReadOnlyList<Vector2[]> worldLoops, Vector2 halfWindow, CancellationToken cancellationToken = default)
        {
            var copy = new Vector2[worldLoops.Count][];
            for (var i = 0; i < copy.Length; i++)
                copy[i] = (Vector2[])worldLoops[i].Clone();
            return Task.Run(() => Bake(copy, halfWindow, cancellationToken), cancellationToken);
        }

        /// <summary>Bakes synchronously (for tests / editor use).</summary>
        public static ConfineRegion Bake(
            IReadOnlyList<Vector2[]> worldLoops, Vector2 halfWindow, CancellationToken cancellationToken = default)
        {
            if (halfWindow.x <= 0f || halfWindow.y <= 0f)
                throw new ArgumentOutOfRangeException(nameof(halfWindow), "window size must be positive.");

            // --- 1. Scale into window space (window = [-1,1]^2) ---
            var scaled = new List<Vector2[]>();
            foreach (var loop in worldLoops)
            {
                if (loop == null || loop.Length < 3) continue;
                var s = new Vector2[loop.Length];
                for (var i = 0; i < loop.Length; i++)
                    s[i] = new Vector2(loop[i].x / halfWindow.x, loop[i].y / halfWindow.y);
                scaled.Add(s);
            }

            if (scaled.Count == 0) return ConfineRegion.Empty;

            // --- 2. Union ---
            var p64 = Clipper.Union(ToPaths64(scaled), FillRule.NonZero);
            var source = ToLoops(p64);
            cancellationToken.ThrowIfCancellationRequested();

            // --- 3. Erode by the window: centers where the window fits entirely.
            // In window space the window is the square [-1,1]^2. The exact erosion is the bounds minus the area
            // swept by the window along their boundary. (A Euclidean offset of -1 would only be exact for
            // axis-aligned edges: a diagonal edge has to move by |n.x| + |n.y|, as the wavefront map assumes.)
            var eroded64 = Clipper.Difference(p64, SweepWindowAlongBoundary(p64), FillRule.NonZero);
            var full = ToLoops(eroded64);
            RemoveDegenerateLoops(full);
            cancellationToken.ThrowIfCancellationRequested();

            // --- 4. Continuous target map (offset wavefront), in window space.
            // Built on the morphological opening of the bounds (erode, then dilate back by the same square):
            // features narrower than the window (notches, slits) are removed, which guarantees that the wavefront
            // from the opening to the eroded region only has edge events. The eroded region itself is unchanged by
            // the opening, so the wavefront ends exactly on it.
            var opened = ToLoops(Clipper.Union(eroded64, SweepWindowAlongBoundary(eroded64), FillRule.NonZero));
            RemoveDegenerateLoops(opened);
            var map = WavefrontMap.Build(opened);
            cancellationToken.ThrowIfCancellationRequested();

            // --- 5. Back to world space ---
            var worldSource = Unscale(source, halfWindow);
            var worldFull = Unscale(full, halfWindow);

            // --- 6. Islands + visibility graph ---
            return BuildGraph(worldSource, worldFull, halfWindow, map, cancellationToken);
        }

        #region Window sweep

        /// <summary>
        ///     The area covered by the window (the square [-1,1]^2 in window space) while its center moves along every
        ///     edge of the paths: the union of the convex hulls of the window at both ends of each edge.
        /// </summary>
        private static Paths64 SweepWindowAlongBoundary(Paths64 paths)
        {
            var hulls = new Paths64();
            var corners = new Point64[8];
            var s = (long)IntScale;
            foreach (var path in paths)
            {
                var n = path.Count;
                for (var i = 0; i < n; i++)
                {
                    var a = path[i];
                    var b = path[(i + 1) % n];
                    corners[0] = new Point64(a.X - s, a.Y - s);
                    corners[1] = new Point64(a.X + s, a.Y - s);
                    corners[2] = new Point64(a.X + s, a.Y + s);
                    corners[3] = new Point64(a.X - s, a.Y + s);
                    corners[4] = new Point64(b.X - s, b.Y - s);
                    corners[5] = new Point64(b.X + s, b.Y - s);
                    corners[6] = new Point64(b.X + s, b.Y + s);
                    corners[7] = new Point64(b.X - s, b.Y + s);
                    hulls.Add(ConvexHull(corners));
                }
            }

            return Clipper.Union(hulls, FillRule.NonZero);
        }

        /// <summary>Convex hull (counter-clockwise) by the monotone chain algorithm.</summary>
        private static Path64 ConvexHull(Point64[] points)
        {
            var sorted = (Point64[])points.Clone();
            Array.Sort(sorted, (p, q) => p.X != q.X ? p.X.CompareTo(q.X) : p.Y.CompareTo(q.Y));
            var hull = new Point64[sorted.Length * 2];
            var k = 0;
            for (var i = 0; i < sorted.Length; i++)
            {
                while (k >= 2 && Cross(hull[k - 2], hull[k - 1], sorted[i]) <= 0) k--;
                hull[k++] = sorted[i];
            }

            for (int i = sorted.Length - 2, lower = k + 1; i >= 0; i--)
            {
                while (k >= lower && Cross(hull[k - 2], hull[k - 1], sorted[i]) <= 0) k--;
                hull[k++] = sorted[i];
            }

            var result = new Path64();
            for (var i = 0; i < k - 1; i++) result.Add(hull[i]);
            return result;
        }

        private static double Cross(Point64 o, Point64 a, Point64 b)
        {
            return (double)(a.X - o.X) * (b.Y - o.Y) - (double)(a.Y - o.Y) * (b.X - o.X);
        }

        #endregion

        #region Clipper conversion

        private static Paths64 ToPaths64(IReadOnlyList<Vector2[]> loops)
        {
            var result = new Paths64();
            foreach (var loop in loops)
            {
                var path = new Path64();
                foreach (var p in loop)
                    path.Add(new Point64(p.x * IntScale, p.y * IntScale));
                result.Add(path);
            }

            return result;
        }

        private static List<Vector2[]> ToLoops(Paths64 paths)
        {
            var result = new List<Vector2[]>(paths.Count);
            foreach (var path in paths)
            {
                if (path.Count < 3) continue;
                var loop = new Vector2[path.Count];
                for (var i = 0; i < path.Count; i++)
                    loop[i] = new Vector2((float)(path[i].X / IntScale), (float)(path[i].Y / IntScale));
                result.Add(loop);
            }

            return result;
        }

        private static void RemoveDegenerateLoops(List<Vector2[]> loops)
        {
            for (var i = loops.Count - 1; i >= 0; i--)
                if (Mathf.Abs(Geometry2D.SignedArea(loops[i])) < Tiny * Tiny)
                    loops.RemoveAt(i);
        }

        private static List<Vector2[]> Unscale(IReadOnlyList<Vector2[]> loops, Vector2 halfWindow)
        {
            var result = new List<Vector2[]>(loops.Count);
            foreach (var loop in loops)
            {
                var s = new Vector2[loop.Length];
                for (var i = 0; i < loop.Length; i++) s[i] = Vector2.Scale(loop[i], halfWindow);
                result.Add(s);
            }

            return result;
        }

        #endregion

        #region Islands + visibility graph

        /// <summary>
        ///     Assigns every loop to an island. Clipper outputs outers CCW (positive area) and holes CW;
        ///     each outer is an island and a hole belongs to the outer that contains it.
        /// </summary>
        public static int AssignIslands(IReadOnlyList<Vector2[]> loops, int[] islandOfLoop)
        {
            var outers = new List<int>();
            for (var i = 0; i < loops.Count; i++)
                if (Geometry2D.SignedArea(loops[i]) > 0f)
                {
                    islandOfLoop[i] = outers.Count;
                    outers.Add(i);
                }

            var single = new Vector2[1][];
            for (var i = 0; i < loops.Count; i++)
            {
                if (Geometry2D.SignedArea(loops[i]) > 0f) continue;
                islandOfLoop[i] = 0;
                // The innermost outer containing the hole is the smallest one that contains it
                var bestArea = float.PositiveInfinity;
                foreach (var o in outers)
                {
                    single[0] = loops[o];
                    if (!Geometry2D.ContainsPoint(single, loops[i][0])) continue;
                    var area = Geometry2D.SignedArea(loops[o]);
                    if (area < bestArea)
                    {
                        bestArea = area;
                        islandOfLoop[i] = islandOfLoop[o];
                    }
                }
            }

            return outers.Count;
        }

        private static ConfineRegion BuildGraph(
            List<Vector2[]> source, List<Vector2[]> full, Vector2 halfWindow, WavefrontMap map, CancellationToken ct)
        {
            var loopIslands = new int[full.Count];
            var islandCount = AssignIslands(full, loopIslands);

            // Reflex vertices of the polygons (concave as seen from inside the region). Clipper outputs
            // outers CCW and holes CW, so in both cases a "right turn" vertex is a corner that blocks the way.
            var nodes = new List<Vector2>();
            foreach (var loop in full)
            {
                var n = loop.Length;
                for (var i = 0; i < n; i++)
                {
                    var prev = loop[(i + n - 1) % n];
                    var cur = loop[i];
                    var next = loop[(i + 1) % n];
                    if (Geometry2D.Cross(cur - prev, next - cur) < -Tiny * Tiny) AddNode(nodes, cur);
                }
            }

            var nodeArray = nodes.ToArray();
            var region = new ConfineRegion(source, full, loopIslands, islandCount, nodeArray,
                new int[nodeArray.Length + 1], Array.Empty<int>(), Array.Empty<float>(), halfWindow, map);

            // Edges: node pairs that can see each other inside the region
            var context = new ConfineRegion.PathContext();
            var edges = new List<(int a, int b, float cost)>();
            for (var i = 0; i < nodeArray.Length; i++)
            {
                ct.ThrowIfCancellationRequested();
                for (var j = i + 1; j < nodeArray.Length; j++)
                    if (region.IsSegmentInside(nodeArray[i], nodeArray[j], context))
                        edges.Add((i, j, (nodeArray[j] - nodeArray[i]).magnitude));
            }

            var start = new int[nodeArray.Length + 1];
            foreach (var e in edges)
            {
                start[e.a + 1]++;
                start[e.b + 1]++;
            }

            for (var i = 0; i < nodeArray.Length; i++) start[i + 1] += start[i];
            var target = new int[edges.Count * 2];
            var cost = new float[edges.Count * 2];
            var fill = new int[nodeArray.Length];
            foreach (var e in edges)
            {
                var ia = start[e.a] + fill[e.a]++;
                var ib = start[e.b] + fill[e.b]++;
                target[ia] = e.b;
                cost[ia] = e.cost;
                target[ib] = e.a;
                cost[ib] = e.cost;
            }

            return new ConfineRegion(source, full, loopIslands, islandCount, nodeArray, start, target, cost,
                halfWindow, map);
        }

        private static void AddNode(List<Vector2> nodes, Vector2 p)
        {
            foreach (var n in nodes)
                if ((n - p).sqrMagnitude <= Tiny * Tiny)
                    return;
            nodes.Add(p);
        }

        #endregion
    }
}