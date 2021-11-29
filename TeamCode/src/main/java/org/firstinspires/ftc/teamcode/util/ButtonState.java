package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ButtonState {
    public enum Button {
        left_stick_button,
        right_stick_button,
        dpad_up,
        dpad_down,
        dpad_left,
        dpad_right,
        a,
        b,
        x,
        y,
        guide,
        start,
        back,
        left_bumper,
        right_bumper,
        right_trigger,
        left_trigger;
    }

    public boolean buttonHeld = false;

    Gamepad gamepad;
    Button button;

    public ButtonState(Gamepad gamepad, Button button) {
        this.gamepad = gamepad;
        this.button = button;
    }

    public boolean getCurrentPress(){
        boolean buttonPress = false;

        switch (button){
            case a:
                buttonPress = gamepad.a;
                break;
            case b:
                buttonPress = gamepad.b;
                break;
            case x:
                buttonPress = gamepad.x;
                break;
            case y:
                buttonPress = gamepad.y;
                break;
            case back:
                buttonPress = gamepad.back;
                break;
            case guide:
                buttonPress = gamepad.guide;
                break;
            case start:
                buttonPress = gamepad.start;
                break;
            case dpad_up:
                buttonPress = gamepad.dpad_up;
                break;
            case dpad_down:
                buttonPress = gamepad.dpad_down;
                break;
            case dpad_left:
                buttonPress = gamepad.dpad_left;
                break;
            case dpad_right:
                buttonPress = gamepad.dpad_right;
                break;
            case left_bumper:
                buttonPress = gamepad.left_bumper;
                break;
            case right_bumper:
                buttonPress = gamepad.right_bumper;
                break;
            case left_stick_button:
                buttonPress = gamepad.left_stick_button;
                break;
            case right_stick_button:
                buttonPress = gamepad.right_stick_button;
                break;
            case right_trigger:
                buttonPress = (gamepad.right_trigger > 0);
                break;
            case left_trigger:
                buttonPress = (gamepad.left_trigger > 0);
                break;
        }

        if (!buttonPress) {
            buttonHeld = false;
        }
        return buttonPress;
    }

    public boolean newPress(){
        boolean state = getCurrentPress();

        if(state && buttonHeld){
            return false;
        } else if (state){
            buttonHeld = true;
            return true;
        } else {
            buttonHeld = false;
            return false;
        }
    }


}
