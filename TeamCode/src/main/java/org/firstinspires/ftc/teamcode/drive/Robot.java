package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public KRASHMecanumDrive drive;
    public Arm arm;

    public Robot(HardwareMap hardwareMap) {
        drive = new KRASHMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
    }

}
