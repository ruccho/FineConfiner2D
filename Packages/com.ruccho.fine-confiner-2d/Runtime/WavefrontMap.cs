using System;
using System.Collections.Generic;
using UnityEngine;

namespace FineConfiner2D
{
    /// <summary>
    ///     Continuous map from the bounds polygon P to the eroded region R, built from the offset wavefront
    ///     (a weighted straight skeleton with edge events). Works in window space, where the camera window is the
    ///     unit square [-1,1]^2 and every edge moves inward at speed |n.x| + |n.y| (the support function of the
    ///     square), so that the wavefront at time 1 is exactly the region where the window fits.
    ///     <para>
    ///         A point t of P \ R lies on the wavefront of exactly one original edge e at some time λ &lt; 1.
    ///         Its image is on the final wavefront edge e(1): t is pushed straight onto the line of e(1) (keeping its
    ///         position along the edge, as far as the window allows) and clamped to the edge. Adjacent faces share
    ///         their vertex trajectories, which must map to the shared vertex of e(1) from both sides. At a convex
    ///         corner the clamp already does this, so near walls and convex corners the camera follows the target
    ///         along the wall and reaches the corner, as with an axis-wise clamp. At a reflex corner the push of the
    ///         trajectory lands inside e(1); within a short distance of the corner that offset is blended out, so the
    ///         map stays continuous there too — unlike the nearest-point map, which jumps across the medial axis at
    ///         concave corners.
    ///     </para>
    /// </summary>
    internal sealed class WavefrontMap
    {
        private const float Eps = 1e-6f;

        /// <summary>
        ///     Distance along the wavefront (window space) over which the push of a target is bent toward a reflex
        ///     corner's trajectory. Beyond it the target is pushed straight onto the final edge.
        /// </summary>
        private const float CornerBlendLength = 1f;

        private readonly List<Face> _faces = new();
        private IReadOnlyList<Vector2[]> _loops = Array.Empty<Vector2[]>();

        private WavefrontMap()
        {
        }

        public int FaceCount => _faces.Count;

        /// <summary>Builds the map for a set of loops in window space (outers CCW, holes CW; region on the left).</summary>
        public static WavefrontMap Build(IReadOnlyList<Vector2[]> loops)
        {
            var map = new WavefrontMap { _loops = loops };
            foreach (var loop in loops) map.BuildLoop(loop);
            return map;
        }

        /// <summary>
        ///     Maps a point of the bounds (window space) to the final wavefront. Returns false when the point is
        ///     not inside any face (outside the bounds, or already inside the eroded region).
        /// </summary>
        public bool TryMap(Vector2 t, out Vector2 result)
        {
            if (TryMapInFaces(t, out result)) return true;

            // Not inside any face: either outside the (opened) bounds, e.g. inside a notch narrower than the window
            // that the opening removed, or a numerical gap. Project onto the bounds and map from there.
            if (_loops.Count == 0) return false;
            var nearest = t;
            var best = float.PositiveInfinity;
            foreach (var loop in _loops)
            {
                var n = loop.Length;
                for (int i = 0, j = n - 1; i < n; j = i++)
                {
                    var c = Geometry2D.ClosestPointOnSegment(loop[j], loop[i], t);
                    var d = (c - t).sqrMagnitude;
                    if (d < best)
                    {
                        best = d;
                        nearest = c;
                    }
                }
            }

            return TryMapInFaces(nearest, out result);
        }

        private bool TryMapInFaces(Vector2 t, out Vector2 result)
        {
            for (var i = 0; i < _faces.Count; i++)
            {
                var f = _faces[i];
                if (!Geometry2D.ContainsPoint(f.PolygonAsList, t)) continue;

                var lambda = Mathf.Clamp01(Vector2.Dot(t - f.EdgeA, f.Normal) / f.Speed);
                var a1 = f.ChainA.PositionAt(1f);
                var b1 = f.ChainB.PositionAt(1f);
                var e1 = b1 - a1;
                var len1 = e1.magnitude;
                if (len1 <= Eps)
                {
                    // The edge has collapsed to a point by time 1
                    result = a1;
                    return true;
                }

                var dir = e1 / len1;
                var a = f.ChainA.PositionAt(lambda);
                var b = f.ChainB.PositionAt(lambda);
                var ab = b - a;
                var abLen = ab.magnitude;
                // Relative position along the wavefront edge at time λ: 0 on trajectory A, 1 on trajectory B
                var sProp = abLen > Eps ? Mathf.Clamp01(Vector2.Dot(t - a, ab) / (abLen * abLen)) : 0f;

                // Relative position of the push of t onto the final edge, and the same for the two trajectory
                // points. A trajectory point pushes outside the final edge at a convex corner (the edge shrinks),
                // where clamping alone joins the neighbouring face; at a reflex corner it pushes inside the edge,
                // and that offset is removed near the corner so that the trajectory maps to the corner itself.
                var sOrth = PushedPosition(t, a1, dir, len1);
                var offsetA = Mathf.Max(0f, PushedPosition(a, a1, dir, len1));
                var offsetB = Mathf.Min(0f, PushedPosition(b, a1, dir, len1) - 1f);
                var weightA = Mathf.Min(1f - sProp, Mathf.Clamp01(1f - sProp * abLen / CornerBlendLength));
                var weightB = Mathf.Min(sProp, Mathf.Clamp01(1f - (1f - sProp) * abLen / CornerBlendLength));
                var s = Mathf.Clamp01(sOrth - weightA * offsetA - weightB * offsetB);

                result = a1 + e1 * s;
                return true;
            }

            result = t;
            return false;
        }

        /// <summary>
        ///     Moves a point on the wavefront at time λ onto the line of the final edge (starting at a1, direction dir,
        ///     length len1) and returns its relative position along the final edge (not clamped).
        ///     <para>
        ///         The point is moved to the final line point nearest to its perpendicular foot among those that keep
        ///         it inside the window (Chebyshev distance ≤ 1). For axis-aligned edges that is the foot itself; for
        ///         diagonal edges close to the bounds it slides toward the window corner that touches the bounds.
        ///     </para>
        /// </summary>
        private static float PushedPosition(Vector2 p, Vector2 a1, Vector2 dir, float len1)
        {
            var foot = Vector2.Dot(p - a1, dir);
            var min = float.NegativeInfinity;
            var max = float.PositiveInfinity;
            // Final line points a1 + dir * u with |x - p.x| ≤ 1 and |y - p.y| ≤ 1
            Restrict(p.x - a1.x, dir.x, ref min, ref max);
            Restrict(p.y - a1.y, dir.y, ref min, ref max);
            var u = min <= max ? Mathf.Clamp(foot, min, max) : (min + max) * 0.5f;
            return u / len1;
        }

        private static void Restrict(float offset, float direction, ref float min, ref float max)
        {
            if (Mathf.Abs(direction) <= Eps) return;
            var u0 = (offset - 1f) / direction;
            var u1 = (offset + 1f) / direction;
            if (u0 > u1) (u0, u1) = (u1, u0);
            min = Mathf.Max(min, u0);
            max = Mathf.Min(max, u1);
        }

        #region Construction

        /// <summary>A wavefront vertex state: position p at time t, moving with velocity d until replaced by next.</summary>
        private sealed class Node
        {
            public Vector2 D;
            public Node Next;
            public Vector2 P;
            public float T;

            public Vector2 PositionAt(float time)
            {
                var n = this;
                while (n.Next != null && n.Next.T <= time) n = n.Next;
                return n.P + n.D * (time - n.T);
            }
        }

        /// <summary>The face swept by one original edge.</summary>
        private sealed class Face
        {
            public Node ChainA, ChainB; // trajectories of the edge's start and end vertices
            public Vector2 EdgeA; // a point on the original edge
            public Vector2 Normal; // inward unit normal
            public Vector2[][] PolygonAsList; // face polygon (single loop), for point location
            public float Speed;
        }

        /// <summary>An active vertex of the shrinking loop.</summary>
        private sealed class Active
        {
            public int EdgeIn, EdgeOut; // original edge indices
            public Active Next, Prev;
            public Node Node;
        }

        private void BuildLoop(Vector2[] loop)
        {
            var n = loop.Length;
            if (n < 3) return;

            // Edge normals (inward = left of the edge direction) and speeds
            var normals = new Vector2[n];
            var speeds = new float[n];
            for (var i = 0; i < n; i++)
            {
                var e = loop[(i + 1) % n] - loop[i];
                var len = e.magnitude;
                if (len < Eps) e = Vector2.right;
                else e /= len;
                normals[i] = new Vector2(-e.y, e.x);
                speeds[i] = Mathf.Abs(normals[i].x) + Mathf.Abs(normals[i].y);
            }

            // Initial active ring and vertex nodes
            var actives = new Active[n];
            var startNodes = new Node[n];
            for (var i = 0; i < n; i++)
            {
                var edgeIn = (i + n - 1) % n;
                var node = new Node { P = loop[i], T = 0f };
                node.D = Velocity(normals[edgeIn], speeds[edgeIn], normals[i], speeds[i]);
                startNodes[i] = node;
                actives[i] = new Active { EdgeIn = edgeIn, EdgeOut = i, Node = node };
            }

            for (var i = 0; i < n; i++)
            {
                actives[i].Next = actives[(i + 1) % n];
                actives[i].Prev = actives[(i + n - 1) % n];
            }

            // Process edge events (edges shrinking to zero length) in time order until t = 1.
            // Split events (a reflex vertex hitting another edge) are not handled: they only occur when the
            // region splits into islands, which the level design is required to avoid.
            var head = actives[0];
            var count = n;
            var time = 0f;
            for (var guard = 0; guard < 4 * n && count >= 3; guard++)
            {
                Active best = null;
                var bestTime = 1f;
                var a = head;
                for (var k = 0; k < count; k++, a = a.Next)
                {
                    var b = a.Next;
                    var pa = a.Node.PositionAt(time);
                    var pb = b.Node.PositionAt(time);
                    var e = pb - pa;
                    var len = e.magnitude;
                    if (len < Eps) continue;
                    var rate = Vector2.Dot(b.Node.D - a.Node.D, e / len);
                    if (rate >= -Eps) continue; // not shrinking
                    var tc = time + len / -rate;
                    if (tc < bestTime)
                    {
                        bestTime = tc;
                        best = a;
                    }
                }

                if (best == null) break;

                // Collapse edge (best, best.Next) at bestTime: merge the two vertices
                var u = best;
                var v = best.Next;
                time = bestTime;
                var merged = new Node { P = u.Node.PositionAt(time), T = time };
                merged.D = Velocity(normals[u.EdgeIn], speeds[u.EdgeIn], normals[v.EdgeOut], speeds[v.EdgeOut]);
                u.Node.Next = merged;
                v.Node.Next = merged;

                var m = new Active
                    { EdgeIn = u.EdgeIn, EdgeOut = v.EdgeOut, Node = merged, Prev = u.Prev, Next = v.Next };
                u.Prev.Next = m;
                v.Next.Prev = m;
                if (head == u || head == v) head = m;
                count--;
            }

            // Faces: one per original edge, bounded by the edge, the two vertex trajectories and the final edge
            for (var i = 0; i < n; i++)
            {
                var chainA = startNodes[i];
                var chainB = startNodes[(i + 1) % n];
                var poly = new List<Vector2>();
                AppendTrajectory(chainA, poly, false);
                AppendTrajectory(chainB, poly, true);
                _faces.Add(new Face
                {
                    ChainA = chainA,
                    ChainB = chainB,
                    EdgeA = loop[i],
                    Normal = normals[i],
                    Speed = speeds[i],
                    PolygonAsList = new[] { poly.ToArray() }
                });
            }
        }

        /// <summary>Appends the positions of a vertex trajectory from t=0 to t=1 (reversed if requested).</summary>
        private static void AppendTrajectory(Node start, List<Vector2> poly, bool reverse)
        {
            var pts = new List<Vector2>();
            var node = start;
            while (true)
            {
                if (node.T >= 1f) break;
                pts.Add(node.P);
                if (node.Next == null || node.Next.T >= 1f)
                {
                    pts.Add(node.P + node.D * (1f - node.T));
                    break;
                }

                node = node.Next;
            }

            if (reverse) pts.Reverse();
            poly.AddRange(pts);
        }

        /// <summary>Velocity of the vertex between two offset lines moving at the given speeds along their normals.</summary>
        private static Vector2 Velocity(Vector2 na, float sa, Vector2 nb, float sb)
        {
            var det = na.x * nb.y - na.y * nb.x;
            if (Mathf.Abs(det) < 1e-4f)
                // (Nearly) collinear edges: move along the common normal. Antiparallel edges (a zero-width slit)
                // cannot occur after the opening; keep the vertex still in that case.
                return Vector2.Dot(na, nb) >= 0f ? na * sa : Vector2.zero;

            // Solve na·d = sa, nb·d = sb
            var dx = (sa * nb.y - sb * na.y) / det;
            var dy = (na.x * sb - nb.x * sa) / det;
            return new Vector2(dx, dy);
        }

        #endregion
    }
}