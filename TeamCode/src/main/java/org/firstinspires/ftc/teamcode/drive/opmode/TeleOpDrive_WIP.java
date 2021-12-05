package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Arm;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.util.ButtonState;

@TeleOp
public class TeleOpDrive_WIP extends LinearOpMode {
    public Robot robot;
    public double drivePower;
    public double strafePower;
    public double turnPower;
    public double leftFrontPower, leftRearPower, rightFrontPower, rightRearPower;
    public double scaleFactor;
    public boolean lowPowerMode = false;
    public boolean armIsMoving = false;
    public boolean turretIsMoving = false;
    public int armStartedLocation = 0;
    public final double LOWPOWERSCALE = 0.5;
    public final double CAROUSEL_POWER = 1.0;
    public final int ARM_TOP = 4000;

    public ButtonState driverX;
    public ButtonState manipB, manipA, manipX, manipY;
    public boolean manipButtonPressed = false;

    public final double expDrivePwrSetpoint = 3;
    public double expDrivePwr = expDrivePwrSetpoint;
    public double expArmPwr = 3;

    private NanoClock clock = NanoClock.system();

    private double moveStart;
    private boolean midMove = false;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        driverX = new ButtonState(gamepad1, ButtonState.Button.x);
        manipB = new ButtonState(gamepad2, ButtonState.Button.b);
        manipA = new ButtonState(gamepad2, ButtonState.Button.a);
        manipX = new ButtonState(gamepad2, ButtonState.Button.x);
        manipY = new ButtonState(gamepad2, ButtonState.Button.y);

        waitForStart();

        while (!isStopRequested()) {

            manipButtonPressed = manipA.getCurrentPress()
                    || manipB.getCurrentPress()
                    || manipX.getCurrentPress()
                    || manipY.getCurrentPress();

            if (!driverX.getCurrentPress()) {
                driveControl();
            }

            if (driverX.getCurrentPress()) {
                if (driverX.newPress()) {
                    robot.drive.turnAsync(Math.toRadians(180.0001));
                } else {
                    robot.drive.update();
                }

            }


            //Reset arm encoder
            if (gamepad1.dpad_left && gamepad2.dpad_up) {
                robot.arm.resetArmEncoder();
            }

            if (gamepad2.dpad_up) {
                //Override encoder-based arm pivot
                robot.arm.pivotArm(Math.pow(-gamepad2.right_stick_y, expArmPwr));
            } else if (!manipButtonPressed) {
                armControl(Math.pow(-gamepad2.right_stick_y, expArmPwr));
            }

            if (gamepad2.dpad_up) {
                robot.arm.spinArm(Math.pow(-gamepad2.right_stick_x, expArmPwr));
            } else if (!manipButtonPressed) {
                turretControl(Math.pow(-gamepad2.right_stick_x, expArmPwr));
            }

            if (!manipButtonPressed) {
                robot.arm.extendArm(gamepad2.left_stick_y);
            }

            //Check for automated routines
            armToSide();
            armToBackIntake();
            armToFrontIntake();
            armToHighHub();

            if (gamepad2.right_trigger > 0) {
                robot.arm.intakeSenseAsync(-gamepad2.right_trigger);
            } else if (gamepad2.left_trigger > 0) {
                robot.arm.intakeSenseAsync(gamepad2.left_trigger * .75);
            } else {
                robot.arm.intakeSenseAsync(0);
            }


            if (gamepad2.right_bumper) {
                robot.drive.runCarousel(CAROUSEL_POWER);
            } else if (gamepad2.left_bumper) {
                robot.drive.runCarousel(-CAROUSEL_POWER);
            } else {
                robot.drive.runCarousel(0);
            }

            telemetry.addData("Arm Position:", robot.arm.armMotor.getCurrentPosition());
            telemetry.addData("Arm Extension:", robot.arm.extensionMotor.getCurrentPosition());
            telemetry.addData("Turret Position:", robot.arm.spinnerMotor.getCurrentPosition());
            telemetry.update();
        }


    }

    private void armToFrontIntake() {
        double armDelay = 2.0; //Wait for turret before pivoting arm
        if (manipA.getCurrentPress()) {
            if (manipA.newPress()) {
                moveStart = clock.seconds();
                midMove = false;
                robot.arm.moveTurretToTarget(Arm.MovingMode.START, robot.arm.FORWARD_TURRET_LIMIT, 1.0, 5);
                robot.arm.moveExtensionToTarget(Arm.MovingMode.START, robot.arm.INTAKE_EXTENSION_COUNTS, 0.8, 5);
            } else if (getElapsedTime(moveStart) >= armDelay && !midMove) {
                midMove = true;
                robot.arm.moveArmToTarget(Arm.MovingMode.START, robot.arm.INTAKE_ARM_COUNTS, 0.6, 5 - armDelay);
                robot.arm.armIsBusy();
                robot.arm.turretIsBusy();
                robot.arm.extensionIsBusy();
            } else {
                robot.arm.armIsBusy();
                robot.arm.turretIsBusy();
                robot.arm.extensionIsBusy();
            }
        }
    }

    private void armToSide() {
        if (manipB.getCurrentPress()) {
            if (manipB.newPress()) {
                robot.arm.moveArmToTarget(Arm.MovingMode.START, robot.arm.getArmTarget(Arm.HubLevel.BOTTOM), 0.8, 5);
                robot.arm.moveTurretToTarget(Arm.MovingMode.START, robot.arm.SIDE_TURRET_LIMIT, 1.0, 5);
                robot.arm.moveExtensionToTarget(Arm.MovingMode.START, robot.arm.MIDDLE_EXTENSION_COUNTS, 0.8, 5);
            } else {
                robot.arm.armIsBusy();
                robot.arm.turretIsBusy();
                robot.arm.extensionIsBusy();
            }
        }
    }

    private void armToBackIntake() {
        double armDelay = 2.0; //Wait for turret before pivoting arm
        if (manipY.getCurrentPress()) {
            if (manipY.newPress()) {
                moveStart = clock.seconds();
                midMove = false;

                robot.arm.moveTurretToTarget(Arm.MovingMode.START, robot.arm.BACK_TURRET_LIMIT, 1.0, 5);
                robot.arm.moveExtensionToTarget(Arm.MovingMode.START, robot.arm.INTAKE_EXTENSION_COUNTS, 0.8, 5);
            } else if (getElapsedTime(moveStart) >= armDelay && !midMove) {
                midMove = true;
                robot.arm.moveArmToTarget(Arm.MovingMode.START, robot.arm.INTAKE_ARM_COUNTS, 0.4, 5 - armDelay);
                robot.arm.armIsBusy();
                robot.arm.turretIsBusy();
                robot.arm.extensionIsBusy();
            } else {
                robot.arm.armIsBusy();
                robot.arm.turretIsBusy();
                robot.arm.extensionIsBusy();
            }
        }
    }

    private void armToHighHub() {
        if (manipX.getCurrentPress()) {
            if (manipX.newPress()) {
                robot.arm.moveArmToTarget(Arm.MovingMode.START, robot.arm.TOP_HUB_COUNTS, 0.8, 5);
                robot.arm.moveTurretToTarget(Arm.MovingMode.START, robot.arm.FORWARD_TURRET_LIMIT, 1.0, 5);
                robot.arm.moveExtensionToTarget(Arm.MovingMode.START, robot.arm.TOP_EXTENSION_COUNTS, 0.8, 5);
            } else {
                robot.arm.armIsBusy();
                robot.arm.turretIsBusy();
                robot.arm.extensionIsBusy();
            }
        }
    }

    private void driveControl() {

        if (gamepad1.right_bumper) {
            lowPowerMode = true;
            expDrivePwr = 1;
        }

        if (gamepad1.left_bumper) {
            lowPowerMode = false;
            expDrivePwr = expDrivePwrSetpoint;
        }

        if (gamepad1.dpad_up) {
            drivePower = 0.8;
            strafePower = -0.7;
            turnPower = 0;
        } else if (gamepad1.dpad_down) {
            drivePower = -0.8;
            strafePower = -0.7;
            turnPower = 0;
        } else {
            drivePower = Math.pow(-gamepad1.left_stick_y, expDrivePwr);
            strafePower = Math.pow(gamepad1.left_stick_x, expDrivePwr);
            turnPower = Math.pow(gamepad1.right_stick_x, expDrivePwr);
        }

        leftFrontPower = drivePower + turnPower + strafePower;
        leftRearPower = drivePower + turnPower - strafePower;
        rightFrontPower = drivePower - turnPower + strafePower;
        rightRearPower = drivePower - turnPower - strafePower;

        scaleFactor = robot.drive.scalePower(leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);


        if (lowPowerMode) {
            scaleFactor = scaleFactor * LOWPOWERSCALE;
        }

        leftFrontPower = scaleFactor * leftFrontPower;
        leftRearPower = scaleFactor * leftRearPower;
        rightFrontPower = scaleFactor * rightFrontPower;
        rightRearPower = scaleFactor * rightRearPower;

        robot.drive.setMotorPowers(leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);
    }

    private void armControl(double power) {
        if (armIsMoving) {
            if (power == 0) {
                robot.arm.pivotArm(0);
                armIsMoving = false;
            } else if (armStartedLocation <= ARM_TOP) {
                robot.arm.pivotArm(power);
            } else {
                robot.arm.pivotArm(power);
            }
        } else {
            if (power == 0) {
                robot.arm.pivotArm(0);
                armIsMoving = false;
            } else if (robot.arm.armMotor.getCurrentPosition() <= ARM_TOP) {
                armStartedLocation = robot.arm.armMotor.getCurrentPosition();
                armIsMoving = true;
                robot.arm.pivotArm(power);
            } else {
                armStartedLocation = robot.arm.armMotor.getCurrentPosition();
                armIsMoving = true;
                robot.arm.pivotArm(power);
            }
        }
    }

    private void turretControl(double power) {

        //Define a low power zone at each end of turret range as a % of total range
        double slowPowerFence = 0.07 * robot.arm.TURRET_RANGE;

        double currentEncoder = robot.arm.spinnerMotor.getCurrentPosition();

        if (power == 0) {
            //Stop turret with zero power
            robot.arm.spinArm(0);
            turretIsMoving = false;
        } else if ((currentEncoder >= robot.arm.FORWARD_TURRET_LIMIT - slowPowerFence) && power < 0) {
            //Apply clockwise 100% stick power if near or even beyond the forward limit
            turretIsMoving = true;
            robot.arm.spinArm(power);
        } else if ((currentEncoder <= robot.arm.REVERSE_TURRET_LIMIT + slowPowerFence) && power > 0) {
            //Apply counterclockwise 100% stick power if near or even beyond the back limit
            armIsMoving = true;
            robot.arm.spinArm(power);
        } else if ((currentEncoder > (robot.arm.REVERSE_TURRET_LIMIT + slowPowerFence))
                && (currentEncoder < (robot.arm.FORWARD_TURRET_LIMIT - slowPowerFence))) {
            //Always apply full stick power in regions away form the range limits
            armIsMoving = true;
            robot.arm.spinArm(power);
        } else if ((currentEncoder > robot.arm.REVERSE_TURRET_LIMIT)
                && (currentEncoder < robot.arm.FORWARD_TURRET_LIMIT)) {
            //If approaching range limits, reduce speed to 50% stick power
            armIsMoving = true;
            robot.arm.spinArm(0.5 * power);
        } else {
            //If we reach this point then the arm is outside of range limits and is shut off
            robot.arm.spinArm(0);
            turretIsMoving = false;
        }
    }

    public double getElapsedTime(double start) {
        return clock.seconds() - start;
    }

}
