package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.util.ButtonState;

@TeleOp
//@Disabled
public class MotorTest extends LinearOpMode {
    public Robot robot;
    public double drivePower;
    public double strafePower;
    public double turnPower;
    public double leftFrontPower, leftRearPower, rightFrontPower, rightRearPower;
    public double scaleFactor;
    public boolean lowPowerMode = false;
    public boolean armIsMoving = false;
    public int armStartedLocation = 0;
    public final double LOWPOWERSCALE = 0.5;
    public final double CAROUSEL_POWER = 1.0;
    public final int ARM_TOP = 4000;


    public ButtonState gripperButton = new ButtonState(gamepad2, ButtonState.Button.x);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {

            if (gamepad1.dpad_up) {
                robot.drive.leftFront.setPower(0.5);
                telemetry.addLine("Left Front");
                telemetry.update();
            } else {
                robot.drive.leftFront.setPower(0);

            }

            if(gamepad1.dpad_down){
                robot.drive.leftRear.setPower(0.5);
                telemetry.addLine("Left Rear");
                telemetry.update();
            } else {
                robot.drive.leftRear.setPower(0);
            }

            if(gamepad1.dpad_right){
                robot.drive.rightFront.setPower(0.5);
                telemetry.addLine("Right Front");
                telemetry.update();
            } else {
                robot.drive.rightFront.setPower(0);
            }

            if(gamepad1.dpad_left){
                robot.drive.rightRear.setPower(0.5);
                telemetry.addLine("Right Rear");
                telemetry.update();
            } else {
                robot.drive.rightRear.setPower(0);
            }

        }
    }
}




