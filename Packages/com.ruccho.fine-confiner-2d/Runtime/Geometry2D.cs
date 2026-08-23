using System.Collections.Generic;
using UnityEngine;

namespace FineConfiner2D
{
    /// <summary>
    ///     2D geometry helpers. Pure functions, no allocations (thread-safe).
    /// </summary>
    internal static class Geometry2D
    {
        public const float Epsilon = 1e-4f;

        /// <summary>
        ///     Tolerance scaled by coordinate magnitude. float has only ~7 significant digits,
        ///     so an absolute 1e-4 becomes unreliable for on-boundary tests once coordinates exceed a few hundred.
        /// </summary>
        public static float Tolerance(in Vector2 p)
        {
            return Mathf.Max(Epsilon, 4e-6f * Mathf.Max(Mathf.Abs(p.x), Mathf.Abs(p.y)));
        }

        public static float Cross(in Vector2 a, in Vector2 b)
        {
            return a.x * b.y - a.y * b.x;
        }

        public static float SignedArea(IReadOnlyList<Vector2> loop)
        {
            var n = loop.Count;
            if (n < 3) return 0f;
            var area = 0f;
            for (int i = 0, j = n - 1; i < n; j = i++)
                area += Cross(loop[j], loop[i]);
            return area * 0.5f;
        }

        /// <summary>Returns the point on segment ab closest to p.</summary>
        public static Vector2 ClosestPointOnSegment(in Vector2 a, in Vector2 b, in Vector2 p)
        {
            var ab = b - a;
            var len2 = ab.sqrMagnitude;
            if (len2 <= 1e-12f) return a;
            var t = Mathf.Clamp01(Vector2.Dot(p - a, ab) / len2);
            return a + ab * t;
        }

        /// <summary>
        ///     Point on segment ab minimizing the scaled Chebyshev distance max(|x-p.x|/scale.x, |y-p.y|/scale.y).
        ///     The distance is convex and piecewise linear along the segment, so the minimum is at an endpoint,
        ///     where one component vanishes, or where both scaled components are equal.
        /// </summary>
        public static Vector2 ClosestPointOnSegmentChebyshev(
            in Vector2 a, in Vector2 b, in Vector2 p, in Vector2 scale, out float distance)
        {
            // Work in scaled space relative to p
            var ax = (a.x - p.x) / scale.x;
            var ay = (a.y - p.y) / scale.y;
            var dx = (b.x - a.x) / scale.x;
            var dy = (b.y - a.y) / scale.y;

            var bestT = 0f;
            var bestD = Mathf.Max(Mathf.Abs(ax), Mathf.Abs(ay));
            var bestE = ax * ax + ay * ay;

            // Candidates are compared by Chebyshev distance first, then (on ties, e.g. along a flat minimum)
            // by Euclidean distance so that the result is the natural projection within the flat interval.
            void Consider(float t)
            {
                if (float.IsNaN(t) || float.IsInfinity(t)) return;
                t = Mathf.Clamp01(t);
                var x = ax + t * dx;
                var y = ay + t * dy;
                var d = Mathf.Max(Mathf.Abs(x), Mathf.Abs(y));
                var e = x * x + y * y;
                if (d < bestD - 1e-6f || (d <= bestD + 1e-6f && e < bestE))
                {
                    bestD = Mathf.Min(bestD, d);
                    bestE = e;
                    bestT = t;
                }
            }

            Consider(1f);
            if (dx != 0f) Consider(-ax / dx);
            if (dy != 0f) Consider(-ay / dy);
            // |X(t)| = |Y(t)|  ->  X = Y or X = -Y
            if (dx - dy != 0f) Consider((ay - ax) / (dx - dy));
            if (dx + dy != 0f) Consider(-(ax + ay) / (dx + dy));
            // Euclidean projection (best tie-breaker inside a flat minimum)
            var len2 = dx * dx + dy * dy;
            if (len2 > 0f) Consider(-(ax * dx + ay * dy) / len2);

            distance = bestD;
            return a + (b - a) * bestT;
        }

        public static float DistanceToSegment(in Vector2 a, in Vector2 b, in Vector2 p)
        {
            return (ClosestPointOnSegment(a, b, p) - p).magnitude;
        }

        /// <summary>Whether p lies on segment ab (within tolerance).</summary>
        public static bool IsOnSegment(in Vector2 a, in Vector2 b, in Vector2 p)
        {
            return DistanceToSegment(a, b, p) <= Tolerance(p);
        }

        public static bool IsOnSegment(in Vector2 a, in Vector2 b, in Vector2 p, float eps)
        {
            return DistanceToSegment(a, b, p) <= eps;
        }

        /// <summary>
        ///     Intersects segment p0p1 with segment q0q1 and returns parameters along p0p1.
        ///     For collinear segments the overlap interval is returned as t0/t1 (count = 2).
        ///     Touching at an endpoint is also reported as an intersection.
        /// </summary>
        public static int IntersectSegments(
            in Vector2 p0, in Vector2 p1, in Vector2 q0, in Vector2 q1,
            out float t0, out float t1, out bool proper)
        {
            t0 = t1 = 0f;
            proper = false;
            var r = p1 - p0;
            var s = q1 - q0;
            var rLen2 = r.sqrMagnitude;
            var sLen2 = s.sqrMagnitude;
            if (rLen2 <= 1e-12f || sLen2 <= 1e-12f) return 0;
            var rLen = Mathf.Sqrt(rLen2);
            var sLen = Mathf.Sqrt(sLen2);
            var tol = Tolerance(p0);

            // Signed distances of q's endpoints from line p
            var dq0 = Cross(r, q0 - p0) / rLen;
            var dq1 = Cross(r, q1 - p0) / rLen;
            // Signed distances of p's endpoints from line q
            var dp0 = Cross(s, p0 - q0) / sLen;
            var dp1 = Cross(s, p1 - q0) / sLen;

            var qOnLineP = Mathf.Abs(dq0) <= tol && Mathf.Abs(dq1) <= tol;
            var pOnLineQ = Mathf.Abs(dp0) <= tol && Mathf.Abs(dp1) <= tol;
            if (qOnLineP || pOnLineQ)
            {
                // Collinear (within tolerance): overlap interval
                var ta = Vector2.Dot(q0 - p0, r) / rLen2;
                var tb = Vector2.Dot(q1 - p0, r) / rLen2;
                if (ta > tb) (ta, tb) = (tb, ta);
                var lo = Mathf.Max(0f, ta);
                var hi = Mathf.Min(1f, tb);
                if (lo > hi + tol / rLen) return 0;
                t0 = Mathf.Clamp01(lo);
                t1 = Mathf.Clamp01(hi);
                return 2;
            }

            // No intersection if both endpoints are on the same side (with tolerance)
            if ((dq0 > tol && dq1 > tol) || (dq0 < -tol && dq1 < -tol)) return 0;
            if ((dp0 > tol && dp1 > tol) || (dp0 < -tol && dp1 < -tol)) return 0;

            var rxs = Cross(r, s);
            if (Mathf.Abs(rxs) <= 1e-12f) return 0; // parallel, not collinear
            var t = Cross(q0 - p0, s) / rxs;
            t0 = Mathf.Clamp01(t);

            // Proper crossing: both segments have endpoints clearly on opposite sides
            proper = ((dq0 > tol && dq1 < -tol) || (dq0 < -tol && dq1 > tol))
                     && ((dp0 > tol && dp1 < -tol) || (dp0 < -tol && dp1 > tol));
            return 1;
        }

        /// <summary>
        ///     Even-odd point-in-polygon test against a set of loops (outers + holes). Points on the boundary count as inside.
        /// </summary>
        public static bool ContainsPoint(IReadOnlyList<Vector2[]> loops, in Vector2 p)
        {
            return ContainsPoint(loops, p, Tolerance(p));
        }

        public static bool ContainsPoint(IReadOnlyList<Vector2[]> loops, in Vector2 p, float eps)
        {
            var inside = false;
            foreach (var loop in loops)
            {
                var n = loop.Length;
                for (int i = 0, j = n - 1; i < n; j = i++)
                {
                    var a = loop[j];
                    var b = loop[i];
                    if (IsOnSegment(a, b, p, eps)) return true;
                    if (a.y > p.y != b.y > p.y)
                    {
                        var x = a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x);
                        if (p.x < x) inside = !inside;
                    }
                }
            }

            return inside;
        }

        /// <summary>
        ///     Whether the whole segment ab lies inside the loops (boundary included).
        ///     The segment is split at every boundary intersection and the midpoint of each piece is tested.
        /// </summary>
        public static bool ContainsSegment(IReadOnlyList<Vector2[]> loops, in Vector2 a, in Vector2 b,
            List<float> scratch)
        {
            if ((b - a).sqrMagnitude <= 1e-12f) return ContainsPoint(loops, a);

            scratch.Clear();
            scratch.Add(0f);
            scratch.Add(1f);
            foreach (var loop in loops)
            {
                var n = loop.Length;
                for (int i = 0, j = n - 1; i < n; j = i++)
                {
                    var c = IntersectSegments(a, b, loop[j], loop[i], out var t0, out var t1, out var proper);
                    if (c == 0) continue;
                    if (proper) return false;
                    scratch.Add(t0);
                    if (c == 2) scratch.Add(t1);
                }
            }

            scratch.Sort();
            for (var i = 1; i < scratch.Count; i++)
            {
                var lo = scratch[i - 1];
                var hi = scratch[i];
                if (hi - lo <= Epsilon) continue;
                var mid = Vector2.Lerp(a, b, (lo + hi) * 0.5f);
                if (!ContainsPoint(loops, mid)) return false;
            }

            return true;
        }
    }
}