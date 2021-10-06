package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Drive drive;

    public Robot(HardwareMap hardwareMap) {
        drive = new Drive(hardwareMap);
    }
}
