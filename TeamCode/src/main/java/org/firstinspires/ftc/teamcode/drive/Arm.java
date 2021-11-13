package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    public DcMotorEx armMotor;
    public DcMotorEx intakeMotor;
    public DcMotorEx spinnerMotor;
    public DcMotorEx extensionMotor;

    final double TICKS_PER_REV = 5281.1;

    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spinnerMotor = hardwareMap.get(DcMotorEx.class, "spinnerMotor");
        spinnerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extensionMotor = hardwareMap.get(DcMotorEx.class, "extensionMotor");
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void moveArmtoPosition(double position){
        int ticksToMove = (int)Math.round(position * TICKS_PER_REV);
        armMotor.setTargetPosition(ticksToMove);

        armMotor.setPower(0.5);
        while(armMotor.isBusy()){

        }
        armMotor.setPower(0);
    }

    public void pivotArm(double power){
        armMotor.setPower(power);
    }

    public void spinArm(double power){
        spinnerMotor.setPower(power);
    }

    public void useIntake(double power){
        intakeMotor.setPower(power);
    }

    public void extendArm(double power){
        extensionMotor.setPower(power);
    }

}
