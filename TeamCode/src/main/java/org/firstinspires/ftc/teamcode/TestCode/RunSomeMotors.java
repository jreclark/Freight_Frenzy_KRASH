package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Robot;

@Autonomous
@Disabled
public class RunSomeMotors extends LinearOpMode {
    public Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        waitForStart();

        //Drive forward for 1s
        robot.drive.setMotorPowers(0.5, 0.5, 0.5, 0.5);
        sleep(1000);

        //Spin for 2s
        robot.drive.setMotorPowers(0.5, 0.5, -0.5, -0.5);
        sleep(2000);

        //Stop moving
        robot.drive.stopRobot();

    }

}
