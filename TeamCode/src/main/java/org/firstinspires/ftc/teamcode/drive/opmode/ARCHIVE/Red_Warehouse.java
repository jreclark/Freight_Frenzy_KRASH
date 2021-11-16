package org.firstinspires.ftc.teamcode.drive.opmode.ARCHIVE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Arm;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.TensorFlowObjectDetectionWebcam;

@Autonomous
public class Red_Warehouse extends LinearOpMode {

    public Robot robot;
    public TensorFlowObjectDetectionWebcam tfod;

    public TensorFlowObjectDetectionWebcam.MARKER_LOCATION markerLocation = TensorFlowObjectDetectionWebcam.MARKER_LOCATION.RIGHT;

    private Trajectory drop;
    private Trajectory carousel;
    private Trajectory park, park1, park2;

    public boolean parkInStorage = false;

    public Arm.HubLevel hubLevel = null;


    Pose2d startingPose = new Pose2d(17.5, -62.5, Math.toRadians(90));
    Pose2d dropLocation = new Pose2d(0, -48, Math.toRadians(110));


    Pose2d parkWarehouse1 = new Pose2d(12, -65, Math.toRadians(0));
    Pose2d parkWarehouse2 = new Pose2d(38, -66, Math.toRadians(0));

    Pose2d parkWarehouseEnd = new Pose2d(66, -39, Math.toRadians(-92));

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        tfod = new TensorFlowObjectDetectionWebcam(hardwareMap, telemetry);

        robot.arm.resetEncoder(robot.arm.armMotor);
        robot.arm.resetEncoder(robot.arm.extensionMotor);

        tfod.initDetector();

        //robot.arm.useIntake(-0.2);

        robot.drive.getLocalizer().setPoseEstimate(startingPose);

        //Initial drop
        drop = robot.drive.trajectoryBuilder(startingPose)
                .splineTo(dropLocation.vec(), dropLocation.getHeading())
                .build();

        //Pickup
        park = robot.drive.trajectoryBuilder(drop.end())
                .lineToLinearHeading(parkWarehouse1)
                .build();
        park1 = robot.drive.trajectoryBuilder(park.end())
                .lineTo(parkWarehouse2.vec())
                .build();



        //TODO: Add vision handling.  Should result in markerLocation indicating marker position.
        while (!isStarted()) {
            markerLocation = tfod.locateMarker();
            hubLevel = robot.arm.markerToLevel(markerLocation);
        }

        //Initial Drop
        robot.arm.moveArmToTarget(Arm.MovingMode.START, robot.arm.getArmTarget(hubLevel), 0.8, 5);
        robot.arm.moveExtensionToTarget(Arm.MovingMode.START, robot.arm.getExtensionTarget(hubLevel), 0.8, 5);
        robot.drive.followTrajectoryAsync(drop);

        while (robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy()) {
            robot.drive.update();
        }

        robot.arm.spitIntake();
        sleep(500);

        robot.drive.followTrajectory(park);

        robot.arm.moveArmToTarget(Arm.MovingMode.START, 300, 0.8, 5);
        robot.arm.moveExtensionToTarget(Arm.MovingMode.START, -300, 0.8, 5);
        robot.arm.useIntake(-0.8);
        robot.drive.followTrajectoryAsync(park1);

        while (robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy()) {
            robot.drive.update();
        }

        Trajectory grab = robot.drive.trajectoryBuilder(park1.end())
                .forward(10)
                .build();

        robot.drive.followTrajectory(grab);

        boolean gotIt = robot.arm.intakeSense(3);
        telemetry.addData("Got block:", gotIt);
        telemetry.update();

        if (gotIt) {
            Trajectory backout = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate(), true)
                    .lineToLinearHeading(new Pose2d(parkWarehouse1.vec(), parkWarehouse1.getHeading() - Math.toRadians(180)))
                    .build();
            robot.arm.moveArmToTarget(Arm.MovingMode.START, 1500, 0.8, 5);
            robot.drive.followTrajectoryAsync(backout);
            while (robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy()) {
                robot.drive.update();
            }

            Trajectory dropAgain = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .lineToLinearHeading(dropLocation)
                    .build();

            robot.arm.moveArmToTarget(Arm.MovingMode.START, robot.arm.getArmTarget(hubLevel), 0.8, 5);
            robot.arm.moveExtensionToTarget(Arm.MovingMode.START, robot.arm.getExtensionTarget(hubLevel), 0.8, 5);
            robot.drive.followTrajectoryAsync(dropAgain);

            while (robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy()) {
                robot.drive.update();
            }

            robot.arm.spitIntake();
            sleep(500);

        }

        sleep(5000);


    }
}
