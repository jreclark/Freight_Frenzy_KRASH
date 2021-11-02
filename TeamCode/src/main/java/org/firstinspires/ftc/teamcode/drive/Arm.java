package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    public DcMotorEx armMotor;
    public ServoCust gripperServo;

    public double gripper_closed = 0;
    public double gripper_open = 0.6;

    final double TICKS_PER_REV = 5281.1;

    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        gripperServo = new ServoCust(hardwareMap, "gripper_servo");
        gripperServo.setInitPos(gripper_closed);
        gripperServo.setPos1(gripper_open);
    }

    public void moveArmtoPosition(double position){
        int ticksToMove = (int)Math.round(position * TICKS_PER_REV);
        armMotor.setTargetPosition(ticksToMove);

        armMotor.setPower(0.5);
        while(armMotor.isBusy()){

        }
        armMotor.setPower(0);
    }



    public void gripperClose(){
        gripperServo.gotoInit();
    }

    public void gripperOpen(){
        gripperServo.gotoPos1();
    }

}
