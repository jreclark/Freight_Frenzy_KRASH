package org.firstinspires.ftc.teamcode.TestCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Arm;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.TensorFlowObjectDetectionWebcam;

@Autonomous
public class Camera_Test extends LinearOpMode {

    public Robot robot;
    public TensorFlowObjectDetectionWebcam tfod;

    public TensorFlowObjectDetectionWebcam.MARKER_LOCATION markerLocation = TensorFlowObjectDetectionWebcam.MARKER_LOCATION.RIGHT;

    private Trajectory drop;
    private Trajectory carousel;
    private Trajectory park, park1, park2;

    public boolean parkInStorage = false;

    public Arm.HubLevel hubLevel = null;


    @Override
    public void runOpMode() throws InterruptedException {
        tfod = new TensorFlowObjectDetectionWebcam(hardwareMap, telemetry);
        tfod.initDetector();


        //TODO: Add vision handling.  Should result in markerLocation indicating marker position.
        while (!isStarted()) {
            markerLocation = tfod.locateMarker();
            //hubLevel = robot.arm.markerToLevel(markerLocation);
        }

    }
}
