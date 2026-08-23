using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace FineConfiner2D.Samples
{
    /// <summary>
    ///     Minimal top-down controller for the sample scene. Moves the Rigidbody2D with WASD / arrow keys / left stick.
    /// </summary>
    [RequireComponent(typeof(Rigidbody2D))]
    public class SamplePlayerController : MonoBehaviour
    {
        [SerializeField] private float speed = 8f;

        private Rigidbody2D body;

        private void Awake()
        {
            body = GetComponent<Rigidbody2D>();
        }

        private void FixedUpdate()
        {
            var input = ReadInput();
            if (input.sqrMagnitude > 1f) input.Normalize();
            body.linearVelocity = input * speed;
        }

        private static Vector2 ReadInput()
        {
            var input = Vector2.zero;
#if ENABLE_INPUT_SYSTEM
            var keyboard = Keyboard.current;
            if (keyboard != null)
            {
                if (keyboard.aKey.isPressed || keyboard.leftArrowKey.isPressed) input.x -= 1f;
                if (keyboard.dKey.isPressed || keyboard.rightArrowKey.isPressed) input.x += 1f;
                if (keyboard.sKey.isPressed || keyboard.downArrowKey.isPressed) input.y -= 1f;
                if (keyboard.wKey.isPressed || keyboard.upArrowKey.isPressed) input.y += 1f;
            }

            var gamepad = Gamepad.current;
            if (gamepad != null) input += gamepad.leftStick.ReadValue();
#elif ENABLE_LEGACY_INPUT_MANAGER
            input = new Vector2(Input.GetAxisRaw("Horizontal"), Input.GetAxisRaw("Vertical"));
#endif
            return input;
        }
    }
}