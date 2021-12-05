package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

public class Arm {
    public DcMotorEx armMotor;
    public DcMotorEx intakeMotor;
    public DcMotorEx spinnerMotor;
    public DcMotorEx extensionMotor;

    final double TICKS_PER_REV = 5281.1;

    public final int TOP_HUB_COUNTS = 2080;
    public final int MIDDLE_HUB_COUNTS = 1600;
    public final int BOTTOM_HUB_COUNTS = 1200;
    public final int INTAKE_ARM_COUNTS = 300;

    public final int TOP_EXTENSION_COUNTS = -465;
    public final int MIDDLE_EXTENSION_COUNTS = -350;
    public final int BOTTOM_EXTENSION_COUNTS = -310;
    public final int INTAKE_EXTENSION_COUNTS = -190;

    public final int FORWARD_TURRET_LIMIT = -100;
    public final int SIDE_TURRET_LIMIT = -3835;
    public final int BACK_TURRET_LIMIT = -7450;
    public final int REVERSE_TURRET_LIMIT = BACK_TURRET_LIMIT - 1;

    public final int TURRET_RANGE = FORWARD_TURRET_LIMIT - REVERSE_TURRET_LIMIT;

    private NanoClock clock = NanoClock.system();

    private double armStartClock;
    private double armTimeout;

    private double extensionStartClock;
    private double extensionTimeout;

    private double turretStartClock;
    private double turretTimeout;


    public enum IntakeState {
        OFF,
        IN,
        HOLD_IN,
        OUT,
        HOLD_OUT    //This value probably doesn't make any sense
    }

    public IntakeState intakeState = IntakeState.OFF;
    private double lastIntakePwr = 0;

    public enum HubLevel {
        TOP,
        MIDDLE,
        BOTTOM
    }

    public enum MovingMode {
        START,
        RUNNING,
        STOP
    }

    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spinnerMotor = hardwareMap.get(DcMotorEx.class, "spinnerMotor");
        spinnerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extensionMotor = hardwareMap.get(DcMotorEx.class, "extensionMotor");
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void moveArmtoPosition(double position) {
        int ticksToMove = (int) Math.round(position * TICKS_PER_REV);
        armMotor.setTargetPosition(ticksToMove);

        armMotor.setPower(0.5);
        while (armMotor.isBusy()) {

        }
        armMotor.setPower(0);
    }

    public void pivotArm(double power) {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(power);
    }

    public void spinArm(double power) {
        spinnerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinnerMotor.setPower(power);
    }

    public void useIntake(double power) {

        intakeMotor.setPower(power);
        lastIntakePwr = power;

        if (power < 0) {
            intakeState = IntakeState.IN;
        } else if (power > 0) {
            intakeState = IntakeState.OUT;
        } else {
            intakeState = IntakeState.OFF;
        }

    }

    public void extendArm(double power) {
        spinnerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extensionMotor.setPower(power);
    }

    public void resetArmEncoder() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetEncoder(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int getArmTarget(HubLevel hubLevel) {
        switch (hubLevel) {
            case TOP:
                return TOP_HUB_COUNTS;
            case MIDDLE:
                return MIDDLE_HUB_COUNTS;
            case BOTTOM:
                return BOTTOM_HUB_COUNTS;
        }
        return 0;
    }

    public boolean moveArmToTarget(MovingMode mode, int target, double power, double timeout) {

        if (mode == MovingMode.STOP) {
            armMotor.setPower(0);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armTimeout = 0;
            armStartClock = 0;
            return false;
        } else if (mode == MovingMode.START) {

            armMotor.setTargetPosition(target);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(power);
            armStartClock = clock.seconds();
            armTimeout = timeout;
            return true;
        } else if (mode == MovingMode.RUNNING) {
            if ((clock.seconds() - armStartClock) >= armTimeout || !armMotor.isBusy()) {
                armMotor.setPower(0);
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armTimeout = 0;
                armStartClock = 0;
                return false;
            } else {
                return true;
            }
        }
        return false;
    }

    public boolean armIsBusy() {
        return moveArmToTarget(MovingMode.RUNNING, 0, 0, 0);
    }

    public int getExtensionTarget(HubLevel hubLevel) {
        switch (hubLevel) {
            case TOP:
                return TOP_EXTENSION_COUNTS;
            case MIDDLE:
                return MIDDLE_EXTENSION_COUNTS;
            case BOTTOM:
                return BOTTOM_EXTENSION_COUNTS;
        }
        return 0;
    }

    public boolean moveExtensionToTarget(MovingMode mode, int target, double power, double timeout) {

        if (mode == MovingMode.STOP) {
            extensionMotor.setPower(0);
            extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extensionTimeout = 0;
            extensionStartClock = 0;
            return false;
        } else if (mode == MovingMode.START) {
            extensionMotor.setTargetPosition(target);
            extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extensionMotor.setPower(power);
            extensionStartClock = clock.seconds();
            extensionTimeout = timeout;
            return true;
        } else if (mode == MovingMode.RUNNING) {
            if ((clock.seconds() - extensionStartClock) >= extensionTimeout || !extensionMotor.isBusy()) {
                extensionMotor.setPower(0);
                extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extensionTimeout = 0;
                extensionStartClock = 0;
                return false;
            } else {
                return true;
            }
        }
        return false;
    }

    public boolean extensionIsBusy() {
        return moveArmToTarget(MovingMode.RUNNING, 0, 0, 0);
    }

    public HubLevel markerToLevel(TensorFlowObjectDetectionWebcam.MARKER_LOCATION markerLocation) {
        switch (markerLocation) {
            case LEFT:
                return HubLevel.BOTTOM;
            case CENTER:
                return HubLevel.MIDDLE;
            case RIGHT:
                return HubLevel.TOP;
        }
        return null;
    }

    public void spitIntake() {
        useIntake(0.8);
        sleep(500);
        useIntake(0);
    }

    public boolean intakeSense(double timeout) {
        double startTime = clock.seconds();
        useIntake(-0.8);
        while (intakeMotor.getCurrent(CurrentUnit.MILLIAMPS) < 1000 && (clock.seconds() - startTime) < timeout) {
        }
        if (intakeMotor.getCurrent(CurrentUnit.MILLIAMPS) > 1000) {
            useIntake(-0.2);
            return true;
        } else {
            useIntake(0);
            return false;
        }

    }

    public boolean intakeSenseAsync(double power) {
        /**
         * Note negative power is intake for "normal" arm position.
         */

        double HOLD_POWER = 0.2;

        switch (intakeState) {
            case OFF:
            case OUT:
                useIntake(power);
                return false;

            case IN:
                if (power <= 0) {
                    if (Math.abs(intakeMotor.getCurrent(CurrentUnit.MILLIAMPS)) > 2000) {
                        useIntake(-HOLD_POWER);
                        intakeState = IntakeState.HOLD_IN;
                        return true;
                    } else if (power < lastIntakePwr) {
                        useIntake(power);
                        return false;
                    } else {
                        return false;
                    }
                } else if (power > 0) {
                    useIntake(power);
                    return false;
                } else {
                    return false;
                }
            case HOLD_IN:
                if (power > 0) {
                    useIntake(power);
                    return false;
                }
                return true;
            default:
                return false;
        }

    }

    public boolean moveTurretToTarget(MovingMode mode, int target, double power, double timeout) {

        if (mode == MovingMode.STOP) {
            spinnerMotor.setPower(0);
            spinnerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turretTimeout = 0;
            turretStartClock = 0;
            return false;
        } else if (mode == MovingMode.START) {

            spinnerMotor.setTargetPosition(target);
            spinnerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spinnerMotor.setPower(power);
            turretStartClock = clock.seconds();
            turretTimeout = timeout;
            return true;
        } else if (mode == MovingMode.RUNNING) {
            if ((clock.seconds() - turretStartClock) >= turretTimeout || !spinnerMotor.isBusy()) {
                spinnerMotor.setPower(0);
                spinnerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turretTimeout = 0;
                turretStartClock = 0;
                return false;
            } else {
                return true;
            }
        }
        return false;
    }

    public boolean turretIsBusy() {
        return moveTurretToTarget(MovingMode.RUNNING, 0, 0, 0);
    }
}
