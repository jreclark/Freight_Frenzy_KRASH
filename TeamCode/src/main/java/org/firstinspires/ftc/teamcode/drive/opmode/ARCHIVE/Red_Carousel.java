package org.firstinspires.ftc.teamcode.drive.opmode.ARCHIVE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Arm;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.TensorFlowObjectDetectionWebcam;

@Autonomous
@Disabled
public class Red_Carousel extends LinearOpMode {

    public Robot robot;
    public TensorFlowObjectDetectionWebcam tfod;

    public TensorFlowObjectDetectionWebcam.MARKER_LOCATION markerLocation = TensorFlowObjectDetectionWebcam.MARKER_LOCATION.RIGHT;

    private Trajectory drop;
    private Trajectory carousel;
    private Trajectory park, park1, park2, park3;

    public boolean parkInStorage = false;

    public Arm.HubLevel hubLevel = null;


    Pose2d startingPose = new Pose2d(-30.5,-62.5,Math.toRadians(90));
    Pose2d carouselLocation = new Pose2d(-61.5, -57.5, Math.toRadians(90));
    Pose2d dropLocation = new Pose2d(-24, -48, Math.toRadians(70));
    Pose2d parkStorageLoc = new Pose2d(-65, -36, Math.toRadians(180)); //reversed

    Pose2d parkWarehouse0 = new Pose2d(-25, -50, Math.toRadians(-45));
    Pose2d parkWarehouse1 = new Pose2d(0, -65, Math.toRadians(0));
    Pose2d parkWarehouse2 = new Pose2d(39, -65, Math.toRadians(0));
    Pose2d parkWarehouse3 = new Pose2d(45, -45, Math.toRadians(45));
    Pose2d parkWarehouseEnd = new Pose2d(66, -39, Math.toRadians(-92));

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        tfod = new TensorFlowObjectDetectionWebcam(hardwareMap, telemetry);

        robot.arm.resetEncoder(robot.arm.armMotor);
        robot.arm.resetEncoder(robot.arm.extensionMotor);

        //robot.arm.useIntake(-0.2);

        tfod.initDetector();

        robot.drive.getLocalizer().setPoseEstimate(startingPose);

        carousel = robot.drive.trajectoryBuilder(startingPose)
                .lineToConstantHeading(carouselLocation.vec())
                .build();

        Trajectory backup = robot.drive.trajectoryBuilder(carousel.end())
                .back(1.5)
                .build();

        drop = robot.drive.trajectoryBuilder(backup.end())
                .splineTo(dropLocation.vec(), dropLocation.getHeading())
                .build();

        if (parkInStorage) {
            //Storage park
            park = robot.drive.trajectoryBuilder(drop.end(), true)
                    .splineTo(parkStorageLoc.vec(), parkStorageLoc.getHeading())
                    .build();
        } else {
            //Warehouse park
            park = robot.drive.trajectoryBuilder(drop.end())
                    .lineToLinearHeading(parkWarehouse1)
                    //.lineTo(parkWarehouse2.vec())
                    //.lineToLinearHeading(parkWarehouseEnd)
                    .build();
            park1 = robot.drive.trajectoryBuilder(park.end())
                    .lineTo(parkWarehouse2.vec())
                    .build();
            park2 = robot.drive.trajectoryBuilder(park1.end())
                    .lineToLinearHeading(parkWarehouse3)
                    .build();
            park3 = robot.drive.trajectoryBuilder(park2.end())
                    .lineToLinearHeading(parkWarehouseEnd)
                    .build();
        }

        //TODO: Add vision handling.  Should result in markerLocation indicating marker position.
        while (!isStarted()){
            markerLocation = tfod.locateMarker();
            hubLevel = robot.arm.markerToLevel(markerLocation);
        }

        //Basic Drive
        robot.drive.followTrajectory(carousel);

        robot.drive.followTrajectory(backup);

        robot.drive.runCarousel(-1.0);
        sleep(3200);
        robot.drive.runCarousel(0);

        //robot.drive.followTrajectory(drop);

        robot.arm.moveArmToTarget(Arm.MovingMode.START, robot.arm.getArmTarget(hubLevel), 0.8, 5);
        robot.arm.moveExtensionToTarget(Arm.MovingMode.START, robot.arm.getExtensionTarget(hubLevel), 0.8, 5);
        robot.drive.followTrajectoryAsync(drop);

        while(robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy()){
            robot.drive.update();
        }

        robot.arm.spitIntake();

        robot.arm.moveExtensionToTarget(Arm.MovingMode.START, -50, 0.8, 5);
        robot.drive.followTrajectoryAsync(park);

        while(robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy()){
            robot.drive.update();
        }

        if(!parkInStorage){
            robot.drive.followTrajectory(park1);
            robot.drive.followTrajectory(park2);

            robot.arm.moveArmToTarget(Arm.MovingMode.START, 300, 0.8, 5);
            robot.arm.moveExtensionToTarget(Arm.MovingMode.START, -300, 0.8, 5);
            robot.arm.useIntake(-0.8);

            while(robot.arm.armIsBusy() || robot.arm.extensionIsBusy()){
            }

            Trajectory grab = robot.drive.trajectoryBuilder(parkWarehouseEnd)
                    .forward(10)
                    .build();

            robot.drive.followTrajectory(grab);

            boolean gotIt = robot.arm.intakeSense(5);
            telemetry.addData("Got block:", gotIt);
            telemetry.update();

            if(gotIt){
                robot.arm.moveArmToTarget(Arm.MovingMode.START, 1500, 0.8, 5);
                while(robot.arm.armIsBusy() || robot.arm.extensionIsBusy()){
                }
            }
        }

        sleep(5000);



    }
}
