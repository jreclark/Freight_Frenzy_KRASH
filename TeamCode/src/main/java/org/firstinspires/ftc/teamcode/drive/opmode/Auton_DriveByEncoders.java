package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveByEncoders;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.TensorFlowObjectDetectionWebcam;

@Autonomous
public class Auton_DriveByEncoders extends LinearOpMode {

    public Robot robot;
    public DriveByEncoders autoDrive;
    public TensorFlowObjectDetectionWebcam tfod;

    public TensorFlowObjectDetectionWebcam.MARKER_LOCATION markerLocation = TensorFlowObjectDetectionWebcam.MARKER_LOCATION.LEFT;

    private double gyroTurnTimeout = 1;
    private double gyroHoldTimeout = 1.0;

    double gyroTurnPowr = 0.7;
    double gyroHoldPowr = 0.6;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        autoDrive = new DriveByEncoders(hardwareMap, robot.drive, telemetry);
        tfod = new TensorFlowObjectDetectionWebcam(hardwareMap, telemetry);

        tfod.initDetector();
        autoDrive.InitializeGryo();

        //TODO: Add vision handling.  Should result in markerLocation indicating marker position.
        while (!isStarted()){
            markerLocation = tfod.locateMarker();
        }


        while(opModeIsActive()){
            telemetry.addData("Left Front", robot.drive.leftFront.getCurrentPosition());
            telemetry.update();
        }

    }
}
