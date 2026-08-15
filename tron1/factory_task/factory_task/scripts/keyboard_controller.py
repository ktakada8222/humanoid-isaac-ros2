import numpy as np
import carb, omni


class KeyboardController():
    base_command = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    up_command = np.array([0.75, 0.0, 0.0], dtype=np.float32)
    left_command = np.array([0.0, 0.0, 0.75], dtype=np.float32)
    right_command = np.array([0.0, 0.0, -0.75], dtype=np.float32)
    keymap = {
        "NUMPAD_8": up_command,  # 前進
        "UP":       up_command,  # 前進
        "NUMPAD_4": left_command,  # 左旋回
        "LEFT":     left_command,  # 左旋回
        "NUMPAD_6": right_command,  # 右旋回
        "RIGHT":    right_command,  # 右旋回
    }

    def __init__(self, keymap=None):
        if keymap:
            self.keymap = keymap
        appwindow = omni.appwindow.get_default_app_window()
        input_iface = carb.input.acquire_input_interface()
        keyboard = appwindow.get_keyboard()
        input_iface.subscribe_to_keyboard_events(keyboard, self.on_key)

    def on_key(self, event, *args, **kwargs):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self.keymap:
                self.base_command += self.keymap[event.input.name]
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            if event.input.name in self.keymap:
                self.base_command -= self.keymap[event.input.name]
        return True

    def get_command(self):
        return self.base_command
