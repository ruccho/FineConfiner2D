using System;
using System.Collections.Generic;
using UnityEngine;

namespace FineConfiner2D
{
    /// <summary>
    ///     The baked region of valid camera centers: the bounding polygons eroded by the camera window,
    ///     so that the window always lies entirely inside the bounds. Immutable and readable from any thread.
    ///     <para>
    ///         <see cref="FindPath" /> returns the shortest path between two points that stays inside the region.
    ///         If the bounds contain passages narrower than the window, the region splits into several
    ///         <see cref="IslandCount">islands</see> with no path between them; keeping the bounds wide enough is
    ///         the responsibility of the level design.
    ///     </para>
    /// </summary>
    public sealed class ConfineRegion
    {
        /// <summary>The empty region. Nothing is confined (points are returned unchanged).</summary>
        public static readonly ConfineRegion Empty = new(
            Array.Empty<Vector2[]>(), Array.Empty<Vector2[]>(), Array.Empty<int>(), 0,
            Array.Empty<Vector2>(), new[] { 0 }, Array.Empty<int>(), Array.Empty<float>(), Vector2.zero, null);

        private readonly float[] _adjacencyCost;

        // Adjacency list in CSR form
        private readonly int[] _adjacencyStart;
        private readonly int[] _adjacencyTarget;

        private readonly int[] _loopIslands;
        private readonly WavefrontMap _map;
        private readonly Vector2[] _nodes;

        internal ConfineRegion(
            IReadOnlyList<Vector2[]> sourceLoops,
            IReadOnlyList<Vector2[]> loops,
            int[] loopIslands,
            int islandCount,
            Vector2[] nodes,
            int[] adjacencyStart,
            int[] adjacencyTarget,
            float[] adjacencyCost,
            Vector2 halfWindow,
            WavefrontMap map)
        {
            _map = map;
            SourceLoops = sourceLoops;
            Loops = loops;
            _loopIslands = loopIslands;
            IslandCount = islandCount;
            _nodes = nodes;
            _adjacencyStart = adjacencyStart;
            _adjacencyTarget = adjacencyTarget;
            _adjacencyCost = adjacencyCost;
            HalfWindow = halfWindow;
        }

        /// <summary>The input bounding loops (world space). For debug drawing.</summary>
        public IReadOnlyList<Vector2[]> SourceLoops { get; }

        /// <summary>Polygons of camera centers where the window fits entirely (outers CCW, holes CW).</summary>
        public IReadOnlyList<Vector2[]> Loops { get; }

        /// <summary>
        ///     Number of disconnected islands the region consists of. More than one island means the bounds contain a
        ///     passage narrower than the window: the camera cannot travel between islands.
        /// </summary>
        public int IslandCount { get; }

        /// <summary>Visibility graph nodes (reflex vertices of the polygons).</summary>
        public IReadOnlyList<Vector2> Nodes => _nodes;

        /// <summary>Half window size used for baking (world units).</summary>
        public Vector2 HalfWindow { get; }

        /// <summary>Whether the region is empty.</summary>
        public bool IsEmpty => Loops.Count == 0;

        public int EdgeCount => _adjacencyTarget.Length / 2;

        /// <summary>Index of the island a loop belongs to (holes share the island of their outer loop).</summary>
        public int GetIslandOfLoop(int loopIndex)
        {
            return _loopIslands[loopIndex];
        }

        public void GetEdges(int node, List<(int target, float cost)> result)
        {
            result.Clear();
            for (var i = _adjacencyStart[node]; i < _adjacencyStart[node + 1]; i++)
                result.Add((_adjacencyTarget[i], _adjacencyCost[i]));
        }

        /// <summary>Whether the point is inside the region (boundary included).</summary>
        public bool Contains(Vector2 p)
        {
            return IsEmpty || Geometry2D.ContainsPoint(Loops, p);
        }

        /// <summary>
        ///     Returns the point inside the region from which p is "most visible": the point minimizing the
        ///     window-scaled Chebyshev distance max(|dx| / HalfWindow.x, |dy| / HalfWindow.y). A result with distance
        ///     ≤ 1 keeps p inside the camera window. Ties are broken by Euclidean distance.
        ///     <para>
        ///         The Euclidean nearest point would be wrong here: a point 7 units below the target (off screen) is
        ///         closer than one 8 units beside it (on screen).
        ///     </para>
        /// </summary>
        public Vector2 Confine(Vector2 p)
        {
            if (IsEmpty) return p;
            if (Geometry2D.ContainsPoint(Loops, p)) return p;

            var hw = HalfWindow.x > 0f && HalfWindow.y > 0f ? HalfWindow : Vector2.one;
            var best = p;
            var bestCheb = float.PositiveInfinity;
            var bestEuclid = float.PositiveInfinity;

            for (var li = 0; li < Loops.Count; li++)
            {
                var loop = Loops[li];
                var n = loop.Length;
                for (int i = 0, j = n - 1; i < n; j = i++)
                {
                    var c = Geometry2D.ClosestPointOnSegmentChebyshev(loop[j], loop[i], p, hw, out var cheb);
                    var euclid = (c - p).sqrMagnitude;
                    if (cheb < bestCheb - 1e-5f || (cheb <= bestCheb + 1e-5f && euclid < bestEuclid))
                    {
                        bestCheb = Mathf.Min(bestCheb, cheb);
                        bestEuclid = euclid;
                        best = c;
                    }
                }
            }

            return best;
        }

        /// <summary>
        ///     Moves <paramref name="from" /> (inside the region) by <paramref name="delta" />, sliding along the
        ///     boundary where the move would leave the region ("collide and slide"). The result is continuous in both
        ///     <paramref name="from" /> and <paramref name="delta" />, so a camera following a target around a concave
        ///     corner keeps a smooth velocity. It can get stuck in a local minimum (e.g. the target is straight behind
        ///     a wall); callers detect that through the returned progress and fall back to <see cref="FindPath" />.
        /// </summary>
        /// <param name="keepSpeed">Keep the full move length along the wall instead of only the tangential component.</param>
        public Vector2 Slide(Vector2 from, Vector2 delta, bool keepSpeed = false)
        {
            if (IsEmpty) return from + delta;

            var pos = from;
            var rem = delta;
            for (var iter = 0; iter < 8 && rem.sqrMagnitude > 1e-12f; iter++)
            {
                // Earliest boundary edge the remaining move would cross from inside to outside.
                // Every edge has the region on its left (outers CCW, holes CW).
                var bestT = float.PositiveInfinity;
                var bestNormal = Vector2.zero;
                for (var li = 0; li < Loops.Count; li++)
                {
                    var loop = Loops[li];
                    var n = loop.Length;
                    for (int i = 0, j = n - 1; i < n; j = i++)
                    {
                        var a = loop[j];
                        var b = loop[i];
                        var e = b - a;
                        var normal = new Vector2(-e.y, e.x); // inward (left) normal, unnormalized
                        var along = Vector2.Dot(rem, normal);
                        if (along >= 0f) continue; // moving inward or parallel: this edge cannot block

                        var tol = Geometry2D.Tolerance(pos);
                        // Signed distance of pos from the edge line (positive = inside), scaled
                        var normalLen = normal.magnitude;
                        if (normalLen <= 1e-12f) continue;
                        var dist = Vector2.Dot(pos - a, normal) / normalLen;
                        if (dist < -tol) continue; // already outside this edge's half-plane: not the blocking edge
                        // Parameter along rem where the edge line is reached
                        var t = dist * normalLen / -along;
                        if (t > 1f || t >= bestT) continue;
                        // Must actually hit the segment (with tolerance), not the extension of its line
                        var hit = pos + rem * Mathf.Max(0f, t);
                        var u = Vector2.Dot(hit - a, e) / e.sqrMagnitude;
                        if (u < -tol || u > 1f + tol) continue;
                        bestT = t;
                        bestNormal = normal / normalLen;
                    }
                }

                if (float.IsPositiveInfinity(bestT))
                {
                    pos += rem;
                    break;
                }

                var tt = Mathf.Clamp01(bestT);
                pos += rem * tt;
                rem *= 1f - tt;
                // Remove the component going through the wall; keep the tangential part (the slide).
                // With keepSpeed the tangential move is stretched back to the blocked length, so a camera
                // pressed against a wall keeps pace along it instead of slowing down by the cosine.
                var blockedLength = rem.magnitude;
                rem -= bestNormal * Vector2.Dot(rem, bestNormal);
                if (keepSpeed && rem.sqrMagnitude > 1e-12f) rem = rem.normalized * blockedLength;
            }

            // Numerical safety: never end up outside
            return Geometry2D.ContainsPoint(Loops, pos) ? pos : Confine(pos);
        }

        /// <summary>
        ///     Maps a target position to the camera position that should show it: the target itself when it is
        ///     inside the region, otherwise the point of the region boundary given by the offset wavefront
        ///     (see <see cref="WavefrontMap" />). Unlike <see cref="Confine" />, this map is continuous in the target,
        ///     so a camera following it moves smoothly around concave corners. Falls back to <see cref="Confine" />
        ///     for targets outside the bounds.
        /// </summary>
        public Vector2 MapTarget(Vector2 target)
        {
            if (IsEmpty) return target;
            if (Geometry2D.ContainsPoint(Loops, target)) return target;
            if (_map != null && HalfWindow.x > 0f && HalfWindow.y > 0f)
            {
                var scaled = new Vector2(target.x / HalfWindow.x, target.y / HalfWindow.y);
                if (_map.TryMap(scaled, out var mapped))
                {
                    var world = new Vector2(mapped.x * HalfWindow.x, mapped.y * HalfWindow.y);
                    // Snap the tiny numerical drift of the wavefront construction onto the exact region
                    return Geometry2D.ContainsPoint(Loops, world) ? world : Confine(world);
                }
            }

            return Confine(target);
        }

        /// <summary>Whether segment ab lies entirely inside the region.</summary>
        public bool IsSegmentInside(Vector2 a, Vector2 b, PathContext context)
        {
            return IsEmpty || Geometry2D.ContainsSegment(Loops, a, b, context.ScratchFloats);
        }

        /// <summary>
        ///     Finds the shortest in-region path from <paramref name="from" /> to <paramref name="to" />.
        ///     Both points must already be inside the region (see <see cref="Confine" />).
        ///     <paramref name="waypoints" /> receives the path excluding <paramref name="from" /> and including
        ///     <paramref name="to" />; it is left empty when the destination is unreachable (different island).
        /// </summary>
        /// <returns>Path length, or positive infinity when unreachable.</returns>
        public float FindPath(Vector2 from, Vector2 to, PathContext context, List<Vector2> waypoints)
        {
            waypoints.Clear();
            if (IsSegmentInside(from, to, context))
            {
                waypoints.Add(to);
                return (to - from).magnitude;
            }

            var n = _nodes.Length;
            if (n == 0) return float.PositiveInfinity;

            context.EnsureCapacity(n);
            var dist = context.Dist;
            var prev = context.Prev;
            var done = context.Done;
            var toVisible = context.ToVisible;

            for (var i = 0; i < n; i++)
            {
                prev[i] = -1;
                done[i] = false;
                toVisible[i] = IsSegmentInside(_nodes[i], to, context);
                dist[i] = IsSegmentInside(from, _nodes[i], context)
                    ? (_nodes[i] - from).magnitude
                    : float.PositiveInfinity;
            }

            var bestTotal = float.PositiveInfinity;
            var bestEnd = -1;

            // Dijkstra (at most a few hundred nodes are expected, so O(n^2) is fine)
            while (true)
            {
                var u = -1;
                var ud = float.PositiveInfinity;
                for (var i = 0; i < n; i++)
                    if (!done[i] && dist[i] < ud)
                    {
                        ud = dist[i];
                        u = i;
                    }

                if (u < 0 || ud >= bestTotal) break;
                done[u] = true;

                if (toVisible[u])
                {
                    var total = ud + (to - _nodes[u]).magnitude;
                    if (total < bestTotal)
                    {
                        bestTotal = total;
                        bestEnd = u;
                    }
                }

                for (var e = _adjacencyStart[u]; e < _adjacencyStart[u + 1]; e++)
                {
                    var v = _adjacencyTarget[e];
                    var nd = ud + _adjacencyCost[e];
                    if (nd < dist[v])
                    {
                        dist[v] = nd;
                        prev[v] = u;
                    }
                }
            }

            if (bestEnd < 0) return float.PositiveInfinity;

            // Reconstruct the path (collected backwards, then reversed)
            for (var u = bestEnd; u >= 0; u = prev[u]) waypoints.Add(_nodes[u]);
            waypoints.Reverse();
            waypoints.Add(to);
            return bestTotal;
        }

        /// <summary>
        ///     Scratch storage used by <see cref="FindPath" /> and friends. Keep one per thread / per user.
        /// </summary>
        public sealed class PathContext
        {
            internal readonly List<float> ScratchFloats = new(64);
            internal float[] Dist = Array.Empty<float>();
            internal bool[] Done = Array.Empty<bool>();
            internal int[] Prev = Array.Empty<int>();
            internal bool[] ToVisible = Array.Empty<bool>();

            internal void EnsureCapacity(int n)
            {
                if (Dist.Length >= n) return;
                Dist = new float[n];
                Prev = new int[n];
                Done = new bool[n];
                ToVisible = new bool[n];
            }
        }
    }
}