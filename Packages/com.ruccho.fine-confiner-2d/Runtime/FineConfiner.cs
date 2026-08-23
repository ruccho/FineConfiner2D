using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

namespace FineConfiner2D
{
    /// <summary>
    ///     Takes a bounding shape and a window size, bakes a <see cref="ConfineRegion" /> on a worker thread and holds it.
    ///     While baking, the previous region stays in use; the new one is swapped in during Update on the frame it completes.
    ///     When the shape / window changes repeatedly, only the latest request wins (an in-flight bake is cancelled).
    /// </summary>
    [AddComponentMenu("FineConfiner2D/Fine Confiner")]
    public class FineConfiner : MonoBehaviour
    {
        private static readonly List<Vector2[]> PathScratch = new();

        [SerializeField]
        [Tooltip(
            "Half size of the camera window (world units). Set automatically from the lens when used with Cinemachine.")]
        private Vector2 halfWindow = new(16f, 9f);

        [SerializeField] [Tooltip("If set, the shape of this CompositeCollider2D is used as the bounds on Start.")]
        private CompositeCollider2D initialShape;

        private TaskCompletionSource<ConfineRegion> _applied;
        private bool _dirty;
        private Task<ConfineRegion> _pending;
        private CancellationTokenSource _pendingCancellation;

        private Vector2[][] _shape = Array.Empty<Vector2[]>();

        /// <summary>The region currently in effect. <see cref="ConfineRegion.Empty" /> until the first bake completes.</summary>
        public ConfineRegion Region { get; private set; } = ConfineRegion.Empty;

        public Vector2 HalfWindow => halfWindow;

        public bool IsBaking => _pending != null;

        private void Start()
        {
            if (initialShape) SetShape(initialShape);
        }

        private void Update()
        {
            if (_dirty) StartBake();
            if (_pending == null || !_pending.IsCompleted) return;

            var task = _pending;
            _pending = null;
            _pendingCancellation?.Dispose();
            _pendingCancellation = null;

            if (task.IsCanceled) return;
            if (task.IsFaulted)
            {
                Debug.LogException(task.Exception?.GetBaseException() ?? task.Exception, this);
                var failed = _applied;
                _applied = null;
                failed?.TrySetException(task.Exception ?? new Exception("bake failed"));
                return;
            }

            Region = task.Result;
            WarnIfSplit(Region);
            var tcs = _applied;
            _applied = null;
            RegionChanged?.Invoke(Region);
            tcs?.TrySetResult(Region);
        }

        private void OnDestroy()
        {
            CancelPending();
            _applied?.TrySetCanceled();
            _applied = null;
        }

        /// <summary>Raised (on the main thread) whenever a new region is applied.</summary>
        public event Action<ConfineRegion> RegionChanged;

        /// <summary>Sets the bounding loops and starts baking.</summary>
        public void SetShape(IReadOnlyList<Vector2[]> worldLoops)
        {
            var copy = new Vector2[worldLoops.Count][];
            for (var i = 0; i < copy.Length; i++) copy[i] = (Vector2[])worldLoops[i].Clone();
            _shape = copy;
            MarkDirty();
        }

        /// <summary>Uses the outlines of a CompositeCollider2D as the bounds.</summary>
        public void SetShape(CompositeCollider2D composite)
        {
            GetWorldPaths(composite, PathScratch);
            SetShape(PathScratch);
        }

        /// <summary>Clears the bounds (nothing is confined).</summary>
        public void ClearShape()
        {
            _shape = Array.Empty<Vector2[]>();
            MarkDirty();
        }

        /// <summary>Sets the half window size. Rebakes if it changed.</summary>
        public void SetHalfWindow(Vector2 value)
        {
            if ((value - halfWindow).sqrMagnitude < 1e-8f) return;
            halfWindow = value;
            MarkDirty();
        }

        /// <summary>
        ///     Waits until the region for the latest shape / window has been applied.
        ///     Completes immediately when nothing is baking. Intended for delaying level-load completion.
        /// </summary>
        public Task<ConfineRegion> WaitForBakeAsync()
        {
            if (_dirty) StartBake();
            return _pending == null ? Task.FromResult(Region) : _applied.Task;
        }

        private void MarkDirty()
        {
            _dirty = true;
            // The bake is started in Update (or WaitForBakeAsync) so that multiple changes within a frame are coalesced
        }

        private void StartBake()
        {
            _dirty = false;
            CancelPending();

            _applied ??= new TaskCompletionSource<ConfineRegion>();
            _pendingCancellation = new CancellationTokenSource();
            var token = _pendingCancellation.Token;
            var hw = halfWindow;
            var input = _shape;

            if (input.Length == 0 || hw.x <= 0f || hw.y <= 0f)
            {
                _pending = Task.FromResult(ConfineRegion.Empty);
                return;
            }

            _pending = Task.Run(() => RegionBaker.Bake(input, hw, token), token);
        }

        private void CancelPending()
        {
            if (_pendingCancellation != null)
            {
                _pendingCancellation.Cancel();
                _pendingCancellation.Dispose();
                _pendingCancellation = null;
            }

            _pending = null;
        }

        /// <summary>
        ///     Logs a warning when the eroded region has more islands than the bounds themselves, i.e. the bounds
        ///     contain a passage narrower than the camera window. The camera cannot travel between islands and
        ///     waits at the edge of its island instead; widen the bounds to fix it.
        /// </summary>
        private void WarnIfSplit(ConfineRegion region)
        {
            if (region.IsEmpty) return;
            var sourceIslands = RegionBaker.AssignIslands(region.SourceLoops, new int[region.SourceLoops.Count]);
            if (region.IslandCount <= sourceIslands) return;
            Debug.LogWarning(
                $"[FineConfiner] The bounds contain passages narrower than the camera window ({region.HalfWindow * 2f}): " +
                $"the region splits into {region.IslandCount} islands ({sourceIslands} expected). " +
                "The camera cannot move between islands. Widen the bounds so every passage fits the window " +
                "(select this object to see the islands in the Scene view).", this);
        }

        /// <summary>Extracts every path of a CompositeCollider2D in world space.</summary>
        public static void GetWorldPaths(CompositeCollider2D composite, List<Vector2[]> result)
        {
            result.Clear();
            if (!composite) return;
            var toWorld = composite.transform.localToWorldMatrix;
            var offset = composite.offset;
            var buffer = new List<Vector2>();
            for (var i = 0; i < composite.pathCount; i++)
            {
                composite.GetPath(i, buffer);
                var loop = new Vector2[buffer.Count];
                for (var j = 0; j < loop.Length; j++)
                    loop[j] = toWorld.MultiplyPoint3x4(buffer[j] + offset);
                result.Add(loop);
            }
        }

#if UNITY_EDITOR
        private static readonly Color[] IslandColors =
        {
            Color.green, Color.cyan, Color.magenta, Color.yellow, new(1f, 0.5f, 0f), new(0.5f, 0.5f, 1f)
        };

        private void OnDrawGizmosSelected()
        {
            var region = Region;
            if (region == null || region.IsEmpty) return;

            Gizmos.color = new Color(1f, 1f, 0f, 0.5f);
            foreach (var loop in region.SourceLoops) DrawLoop(loop);

            // One color per island: several colors mean the bounds have a passage narrower than the window
            for (var i = 0; i < region.Loops.Count; i++)
            {
                Gizmos.color = IslandColors[region.GetIslandOfLoop(i) % IslandColors.Length];
                DrawLoop(region.Loops[i]);
            }

            Gizmos.color = Color.white;
            foreach (var n in region.Nodes) Gizmos.DrawSphere(n, 0.1f);
        }

        private static void DrawLoop(Vector2[] loop)
        {
            for (int i = 0, j = loop.Length - 1; i < loop.Length; j = i++)
                Gizmos.DrawLine(loop[j], loop[i]);
        }
#endif
    }
}