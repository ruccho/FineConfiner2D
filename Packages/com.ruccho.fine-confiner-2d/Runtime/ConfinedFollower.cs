using System.Collections.Generic;
using UnityEngine;

namespace FineConfiner2D
{
    /// <summary>
    ///     Moves toward a goal along the shortest path inside a <see cref="ConfineRegion" />.
    ///     The position is always inside the region and travels along the path, so it never leaves the bounds or jumps.
    ///     Plain C# class with no dependency on Unity components.
    /// </summary>
    public sealed class ConfinedFollower
    {
        private readonly ConfineRegion.PathContext _context = new();
        private readonly List<Vector2> _waypoints = new();
        public ConfineRegion Region { get; private set; } = ConfineRegion.Empty;

        /// <summary>Current position (always inside the region).</summary>
        public Vector2 Position { get; private set; }

        /// <summary>
        ///     Path computed by the last <see cref="MoveToward" /> / <see cref="PathLengthTo" /> (excludes the current
        ///     position, includes the goal).
        /// </summary>
        public IReadOnlyList<Vector2> LastPath => _waypoints;

        /// <summary>The last goal (already confined).</summary>
        public Vector2 LastGoal { get; private set; }

        /// <summary>
        ///     Whether the last goal was reachable. False when the goal lies on another island of the region
        ///     (a passage narrower than the window); the follower then waits where it is.
        /// </summary>
        public bool IsGoalReachable { get; private set; } = true;

        /// <summary>Replaces the region. If the current position is outside the new region it is moved to the closest point.</summary>
        public void SetRegion(ConfineRegion region)
        {
            Region = region ?? ConfineRegion.Empty;
            Position = Region.Confine(Position);
            _waypoints.Clear();
        }

        /// <summary>Sets the position immediately (confined to the region).</summary>
        public void Teleport(Vector2 position)
        {
            Position = Region.Confine(position);
            _waypoints.Clear();
        }

        /// <summary>
        ///     Length of the in-region path to target (any point).
        ///     Returns positive infinity when the goal is unreachable (see <see cref="IsGoalReachable" />).
        /// </summary>
        public float PathLengthTo(Vector2 target)
        {
            LastGoal = Region.Confine(target);
            var length = Region.FindPath(Position, LastGoal, _context, _waypoints);
            IsGoalReachable = !float.IsPositiveInfinity(length);
            return length;
        }

        /// <summary>
        ///     Advances along the path toward target by at most maxDistance.
        ///     Does nothing (waits) when the goal is unreachable.
        /// </summary>
        /// <returns>Remaining path length after the move.</returns>
        public float MoveToward(Vector2 target, float maxDistance)
        {
            var remaining = PathLengthTo(target);
            if (!IsGoalReachable) return remaining; // wait: there is no in-region way to the goal
            if (_waypoints.Count == 0) return 0f;

            var budget = Mathf.Max(0f, maxDistance);
            var index = 0;
            while (budget > 0f && index < _waypoints.Count)
            {
                var wp = _waypoints[index];
                var delta = wp - Position;
                var dist = delta.magnitude;
                if (dist <= budget)
                {
                    Position = wp;
                    budget -= dist;
                    remaining -= dist;
                    index++;
                }
                else
                {
                    Position += delta * (budget / dist);
                    remaining -= budget;
                    budget = 0f;
                }
            }

            return Mathf.Max(0f, remaining);
        }
    }
}