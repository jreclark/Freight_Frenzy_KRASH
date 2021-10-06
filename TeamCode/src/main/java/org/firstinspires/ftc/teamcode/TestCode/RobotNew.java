package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotNew {
    public Drive drive;

    public RobotNew(HardwareMap hardwareMap) {
        drive = new Drive(hardwareMap);
    }
}
