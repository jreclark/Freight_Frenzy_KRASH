package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.TensorFlowObjectDetectionWebcam;

@Autonomous
public class Red_Carousel extends LinearOpMode {

    public Robot robot;
    public TensorFlowObjectDetectionWebcam tfod;

    public TensorFlowObjectDetectionWebcam.MARKER_LOCATION markerLocation = TensorFlowObjectDetectionWebcam.MARKER_LOCATION.LEFT;

    private Trajectory drop;
    private Trajectory carousel;
    private Trajectory park;


    Pose2d startingPose = new Pose2d(-30.5,-62.5,Math.toRadians(90));
    Pose2d carouselLocation = new Pose2d(-61.5, -58.5, Math.toRadians(90));
    Pose2d dropLocation = new Pose2d(-34.5, -35, Math.toRadians(40));
    Pose2d parkStorageLoc = new Pose2d(-65, -36, Math.toRadians(180)); //reversed

    Pose2d parkWarehouse0 = new Pose2d(-25, -50, Math.toRadians(-45)); //reversed
    Pose2d parkWarehouse1 = new Pose2d(0, -65, Math.toRadians(0));
    Pose2d parkWarehouse2 = new Pose2d(38, -65, Math.toRadians(0));
    Pose2d parkWarehouse3 = new Pose2d(40, -50, Math.toRadians(45));
    Pose2d parkWarehouseEnd = new Pose2d(64, -41, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        tfod = new TensorFlowObjectDetectionWebcam(hardwareMap, telemetry);

        //tfod.initDetector();

        robot.drive.getLocalizer().setPoseEstimate(startingPose);

        carousel = robot.drive.trajectoryBuilder(startingPose)
                .lineToConstantHeading(carouselLocation.vec())
                .build();

        drop = robot.drive.trajectoryBuilder(carousel.end())
                .splineTo(dropLocation.vec(), dropLocation.getHeading())
                .build();

        //Wharehouse park
        /*park = robot.drive.trajectoryBuilder(drop.end(), true)
                .splineTo(parkStorageLoc.vec(), parkStorageLoc.getHeading())
                .build();*/

        park = robot.drive.trajectoryBuilder(drop.end(), true)
                .splineTo(parkWarehouse0.vec(), parkWarehouse0.getHeading())
                .splineTo(parkWarehouse1.vec(), parkWarehouse1.getHeading())
                .splineTo(parkWarehouse2.vec(), parkWarehouse2.getHeading())
                .splineTo(parkWarehouse3.vec(), parkWarehouse3.getHeading())
                .splineTo(parkWarehouseEnd.vec(), parkWarehouseEnd.getHeading())
                .build();


        //TODO: Add vision handling.  Should result in markerLocation indicating marker position.
        while (!isStarted()){
            //markerLocation = tfod.locateMarker();
        }

        //Basic Drive
        robot.drive.followTrajectory(carousel);

        robot.drive.runCarousel(-1.0);
        sleep(3000);
        robot.drive.runCarousel(0);

        robot.drive.followTrajectory(drop);

        sleep(500);

        robot.drive.followTrajectory(park);



    }
}
