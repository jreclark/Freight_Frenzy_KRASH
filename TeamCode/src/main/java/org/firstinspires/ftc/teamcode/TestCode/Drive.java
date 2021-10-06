package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive {
    public DcMotorEx leftFront, leftRear, rightFront, rightRear;

    public Drive(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftRear = hardwareMap.get(DcMotorEx.class, "left_rear");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        rightRear = hardwareMap.get(DcMotorEx.class, "right_rear");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setMotorPowers(double lfPwr, double lrPwr, double rfPwr, double rrPwr){
        leftFront.setPower(lfPwr);
        leftRear.setPower(lrPwr);
        rightFront.setPower(rfPwr);
        rightRear.setPower(rrPwr);
    }

    public void stopRobot(){
        setMotorPowers(0,0,0,0);
    }

    public double scalePower(double p1, double p2, double p3, double p4)
    {
        //Returns a scaling factor to normalize the input powers p1-p4 to a maximum magnitude of 1

        double maxValue = Math.abs(p1);
        double scaleFactor = 1;

        //Search for the largest power magnitude
        if (Math.abs(p2) > maxValue) {
            maxValue = Math.abs(p2);
        }
        if (Math.abs(p3) > maxValue) {
            maxValue = Math.abs(p3);
        }
        if (Math.abs(p4) > maxValue) {
            maxValue = Math.abs(p4);
        }

        //If maxValue is larger than 1 return a scale factor to limit it to +1 or -1
        if (maxValue > 1 )
        {
            scaleFactor = 1 / maxValue;
        }

        return  scaleFactor;
    }


}
