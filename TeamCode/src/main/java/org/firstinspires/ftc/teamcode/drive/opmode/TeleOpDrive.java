package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robot;

@TeleOp
public class TeleOpDrive extends LinearOpMode {
    public Robot robot;
    public double drivePower;
    public double strafePower;
    public double turnPower;
    public double leftFrontPower, leftRearPower, rightFrontPower, rightRearPower;
    public double scaleFactor;
    public boolean lowPowerMode = false;
    public final double LOWPOWERSCALE = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.arm.gripperServo.gotoInit();

        waitForStart();

        while(!isStopRequested()){
            driveControl();
        }


    }

    private void driveControl() {
        if(gamepad1.right_bumper){
            lowPowerMode = true;
        }

        if(gamepad1.left_bumper){
            lowPowerMode = false;
        }

        robot.arm.gripperServo.setPosition(gamepad2.left_trigger);


        drivePower = -gamepad1.left_stick_y;
        strafePower = gamepad1.left_stick_x;
        turnPower = gamepad1.right_stick_x;

        leftFrontPower = drivePower + turnPower + strafePower;
        leftRearPower = drivePower + turnPower - strafePower;
        rightFrontPower = drivePower - turnPower - strafePower;
        rightRearPower = drivePower - turnPower + strafePower;

        scaleFactor = robot.drive.scalePower(leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);


        if(lowPowerMode){
            scaleFactor = scaleFactor * LOWPOWERSCALE;
        }

        leftFrontPower = scaleFactor * leftFrontPower;
        leftRearPower = scaleFactor * leftRearPower;
        rightFrontPower = scaleFactor * rightFrontPower;
        rightRearPower = scaleFactor * rightRearPower;

        robot.drive.setMotorPowers(leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);
        sleep(10);

    }

}
