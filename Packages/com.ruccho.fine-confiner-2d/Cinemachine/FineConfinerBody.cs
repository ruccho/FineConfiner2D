using System;
using Unity.Cinemachine;
using UnityEngine;

namespace FineConfiner2D.Cinemachine
{
    /// <summary>
    ///     Cinemachine Body stage. Confines the Follow target to the <see cref="FineConfiner" /> region first,
    ///     then follows it with damping along the shortest path inside the region.
    ///     Replaces PositionComposer + Confiner2D.
    ///     <para>The camera is always inside the region, so it never pokes outside the bounds and never jumps.</para>
    ///     <para>
    ///         Lookahead / DeadZone / HardLimits have the same meaning and units as in the Position Composer,
    ///         and are all applied before confining (when deciding the goal point).
    ///     </para>
    /// </summary>
    [AddComponentMenu("Cinemachine/Procedural/Position Control/Fine Confiner Body")]
    [CameraPipeline(CinemachineCore.Stage.Body)]
    [DisallowMultipleComponent]
    public class FineConfinerBody : CinemachineComponentBase
    {
        [Tooltip("The FineConfiner that provides the region")]
        public FineConfiner confiner;

        [Tooltip("Distance from the target to the camera along the camera's -forward axis")]
        public float cameraDistance = 10f;

        [Tooltip("Damping time in seconds for following along the path. 0 follows instantly.")] [Range(0f, 20f)]
        public float damping = 0.5f;

        [Tooltip("Derive the window size from the lens (OrthographicSize / Aspect)")]
        public bool autoWindowFromLens = true;

        [Tooltip("Screen position where the target should be placed (screen ratio, 0 = center)")]
        public Vector2 screenPosition = Vector2.zero;

        [Tooltip("Predict the target's future position from its velocity and follow that instead")]
        public LookaheadSettings lookahead;

        public DeadZoneSettings deadZone = new() { enabled = false, size = new Vector2(0.1f, 0.2f) };

        public HardLimitsSettings hardLimits = new() { enabled = false, size = new Vector2(0.8f, 0.8f) };

        private PositionPredictor _predictor;
        private float _previousExcess;

        /// <summary>The internal follower (for debugging).</summary>
        public ConfinedFollower Follower { get; } = new();

        /// <summary>Half window size used in the last update (world units).</summary>
        public Vector2 HalfWindow { get; private set; } = new(16f, 9f);

        public override bool IsValid => enabled && FollowTarget != null;

        public override CinemachineCore.Stage Stage => CinemachineCore.Stage.Body;

        private void OnValidate()
        {
            cameraDistance = Mathf.Max(0.01f, cameraDistance);
            damping = Mathf.Max(0f, damping);
            lookahead.Time = Mathf.Max(0f, lookahead.Time);
            lookahead.Smoothing = Mathf.Max(0f, lookahead.Smoothing);
            deadZone.size = Vector2.Max(Vector2.zero, deadZone.size);
            hardLimits.size = Vector2.Max(deadZone.enabled ? deadZone.size : Vector2.zero, hardLimits.size);
        }

        public override float GetMaxDampTime()
        {
            return damping;
        }

        public override void MutateCameraState(ref CameraState curState, float deltaTime)
        {
            if (!IsValid) return;

            var lens = curState.Lens;
            var halfHeight = lens.Orthographic
                ? lens.OrthographicSize
                : cameraDistance * Mathf.Tan(lens.FieldOfView * 0.5f * Mathf.Deg2Rad);
            HalfWindow = new Vector2(halfHeight * lens.Aspect, halfHeight);
            if (autoWindowFromLens && confiner) confiner.SetHalfWindow(HalfWindow);

            var region = confiner ? confiner.Region : ConfineRegion.Empty;
            if (!ReferenceEquals(region, Follower.Region)) Follower.SetRegion(region);

            var targetPos = FollowTargetPosition;
            var previousStateIsValid = deltaTime >= 0f && VirtualCamera.PreviousStateIsValid;
            if (!previousStateIsValid || VirtualCamera.FollowTargetChanged) _predictor.Reset();

            // --- Lookahead (same predictor as the Position Composer) ---
            var target = (Vector2)targetPos;
            if (lookahead.Enabled && lookahead.Time > 1e-4f)
            {
                _predictor.Smoothing = lookahead.Smoothing;
                _predictor.AddPosition(targetPos, deltaTime);
                var delta = (Vector2)_predictor.PredictPositionDelta(lookahead.Time);
                if (lookahead.IgnoreY) delta.y = 0f;
                target += delta;
            }

            // Desired camera center that places the target at ScreenPosition
            var desired = target - screenPosition * (HalfWindow * 2f);

            // Continuous camera goal for the target: the target itself inside the region, otherwise the point of
            // the region boundary given by the offset wavefront. Continuity here is what makes the motion smooth
            // around concave corners: the nearest boundary point would jump across the corner's medial axis.
            var mapped = Follower.Region.MapTarget(desired);

            if (!previousStateIsValid || damping <= 0f)
            {
                Follower.Teleport(mapped);
                _previousExcess = 0f;
            }
            else
            {
                var current = Follower.Position;

                // --- DeadZone: stay put inside the rectangle, otherwise move just enough to reach its edge ---
                var goal = mapped;
                if (deadZone.enabled)
                    goal = current + ClampOutside(mapped - current, deadZone.size * HalfWindow);

                // --- HardLimits: keep the target inside this rectangle. The minimum travel is the amount by
                // which the overshoot *grew* since the last frame (keep pace with the target), never the
                // accumulated overshoot, which would be applied all at once when the camera gets unblocked.
                var excess = hardLimits.enabled
                    ? ClampOutside(desired - current, hardLimits.size * HalfWindow).magnitude
                    : 0f;
                var required = Mathf.Max(0f, excess - _previousExcess);

                var toGoal = goal - current;
                var distance = toGoal.magnitude;
                var step = Mathf.Min(distance, Mathf.Max(Damper.Damp(distance, damping, deltaTime), required));

                if (step > 1e-5f)
                {
                    // Move straight toward the goal, sliding along the bounds where the straight line cuts a
                    // corner. The goal is continuous, so this is continuous too.
                    var next = Follower.Region.Slide(current, toGoal * (step / distance));
                    var progress = (next - current).magnitude;
                    if (progress >= 0.1f * step)
                    {
                        Follower.Teleport(next);
                    }
                    else
                    {
                        // Safety net: blocked (e.g. the goal is on the far side of a wall after a teleport of
                        // the target). Take the shortest in-region path; wait if the goal is on another island.
                        Follower.PathLengthTo(goal);
                        Follower.MoveToward(goal, Follower.IsGoalReachable ? step : 0f);
                    }
                }

                _previousExcess = hardLimits.enabled
                    ? ClampOutside(desired - Follower.Position, hardLimits.size * HalfWindow).magnitude
                    : 0f;
            }

            var pos = Follower.Position;
            var depth = curState.RawOrientation * Vector3.back * cameraDistance;
            curState.RawPosition = new Vector3(pos.x, pos.y, targetPos.z) + depth;
        }

        /// <summary>
        ///     Returns zero if offset lies within the rectangle of half size halfSize,
        ///     otherwise the per-axis amount by which it sticks out.
        /// </summary>
        private static Vector2 ClampOutside(Vector2 offset, Vector2 halfSize)
        {
            return new Vector2(
                offset.x > halfSize.x ? offset.x - halfSize.x : offset.x < -halfSize.x ? offset.x + halfSize.x : 0f,
                offset.y > halfSize.y ? offset.y - halfSize.y : offset.y < -halfSize.y ? offset.y + halfSize.y : 0f);
        }

        public override void ForceCameraPosition(Vector3 pos, Quaternion rot)
        {
            Follower.Teleport(pos);
            _predictor.Reset();
        }

        public override void OnTargetObjectWarped(Transform target, Vector3 positionDelta)
        {
            if (target != FollowTarget) return;
            Follower.Teleport(Follower.Position + (Vector2)positionDelta);
            _predictor.ApplyTransformDelta(positionDelta);
        }

        [Serializable]
        public struct DeadZoneSettings
        {
            [Tooltip("When enabled, the camera does not move while the target stays inside this rectangle")]
            public bool enabled;

            [Tooltip("Size in screen ratio (1 = full screen)")]
            public Vector2 size;
        }

        [Serializable]
        public struct HardLimitsSettings
        {
            [Tooltip(
                "When enabled, the camera moves immediately so that the target never leaves this rectangle (unless stopped by the bounds)")]
            public bool enabled;

            [Tooltip("Size in screen ratio (1 = full screen)")]
            public Vector2 size;
        }
    }
}