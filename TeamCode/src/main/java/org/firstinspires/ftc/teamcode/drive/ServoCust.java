package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoCust {
    private Servo servoCust;
    private double initPos = -1;
    private double pos1 = -1, pos2 = -1;

    public ServoCust(HardwareMap hardwareMap, String name) {
        servoCust = hardwareMap.get(Servo.class, name);
    }

    public void gotoInit(){
        if (initPos != -1) {
            servoCust.setPosition(initPos);
        }
    }

    public void gotoPos1(){
        if (pos1 != -1) {
            servoCust.setPosition(pos1);
        }
    }

    public void gotoPos2(){
        if (pos1 != -1) {
            servoCust.setPosition(pos2);
        }
    }

    public void setPosition(double position){
        servoCust.setPosition(position);
    }

    public double getPosition(){
        return servoCust.getPosition();
    }

    public double getInitPos() {
        return initPos;
    }

    public void setInitPos(double initPos) {
        this.initPos = initPos;
    }

    public double getPos1() {
        return pos1;
    }

    public void setPos1(double pos1) {
        this.pos1 = pos1;
    }

    public double getPos2() {
        return pos2;
    }

    public void setPos2(double pos2) {
        this.pos2 = pos2;
    }
}
