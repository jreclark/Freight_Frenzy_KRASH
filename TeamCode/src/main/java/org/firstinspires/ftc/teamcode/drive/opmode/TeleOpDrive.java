package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
    public boolean armIsMoving = false;
    public int armStartedLocation = 0;
    public final double LOWPOWERSCALE = 0.5;
    public final double CAROUSEL_POWER = 1.0;
    public final int ARM_TOP = 4000;

    public enum armState {
        UP,
        DOWN,
        STOPPED
    }



    public ButtonState gripperButton = new ButtonState(gamepad2, ButtonState.Button.x);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        waitForStart();

        while(!isStopRequested()){

            driveControl();

            //robot.arm.pivotArm(-gamepad2.left_stick_y);

            if(gamepad1.dpad_up && gamepad2.dpad_up){
                robot.arm.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.arm.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad1.dpad_down) {
                robot.arm.pivotArm(-gamepad2.left_stick_y);
            } else {
                if (armIsMoving) {
                    if (gamepad2.left_stick_y == 0) {
                        robot.arm.pivotArm(0);
                        armIsMoving = false;
                    } else if (armStartedLocation <= ARM_TOP) {
                        robot.arm.pivotArm(-gamepad2.left_stick_y);
                    } else {
                        robot.arm.pivotArm(gamepad2.left_stick_y);
                    }
                } else {
                    if (gamepad2.left_stick_y == 0) {
                        robot.arm.pivotArm(0);
                        armIsMoving = false;
                    } else if (robot.arm.armMotor.getCurrentPosition() <= ARM_TOP) {
                        armStartedLocation = robot.arm.armMotor.getCurrentPosition();
                        armIsMoving = true;
                        robot.arm.pivotArm(-gamepad2.left_stick_y);
                    } else {
                        armStartedLocation = robot.arm.armMotor.getCurrentPosition();
                        armIsMoving = true;
                        robot.arm.pivotArm(gamepad2.left_stick_y);
                    }
                }
            }

            robot.arm.spinArm(-gamepad2.right_stick_x);

            robot.arm.extendArm(gamepad2.right_stick_y);

            if(gamepad2.right_trigger>0){
                robot.arm.useIntake(-gamepad2.right_trigger);
            } else if (gamepad2.left_trigger >0){
                robot.arm.useIntake(gamepad2.left_trigger);
            } else {
                robot.arm.useIntake(0);
            }


            if(gamepad2.right_bumper){
                robot.drive.runCarousel(CAROUSEL_POWER);
            }else if (gamepad2.left_bumper){
                robot.drive.runCarousel(-CAROUSEL_POWER);
            } else {
                robot.drive.runCarousel(0);
            }

            telemetry.addData("Motor Current:", robot.arm.intakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();
        }


    }

    private void driveControl() {
        if(gamepad1.right_bumper){
            lowPowerMode = true;
        }

        if(gamepad1.left_bumper){
            lowPowerMode = false;
        }

        drivePower = -gamepad1.left_stick_y;
        strafePower = gamepad1.left_stick_x;
        turnPower = gamepad1.right_stick_x;

        leftFrontPower = drivePower + turnPower + strafePower;
        leftRearPower = drivePower + turnPower - strafePower;
        rightFrontPower = drivePower - turnPower + strafePower;
        rightRearPower = drivePower - turnPower - strafePower;

        scaleFactor = robot.drive.scalePower(leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);


        if(lowPowerMode){
            scaleFactor = scaleFactor * LOWPOWERSCALE;
        }

        leftFrontPower = scaleFactor * leftFrontPower;
        leftRearPower = scaleFactor * leftRearPower;
        rightFrontPower = scaleFactor * rightFrontPower;
        rightRearPower = scaleFactor * rightRearPower;

        robot.drive.setMotorPowers(leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);

    }

}
