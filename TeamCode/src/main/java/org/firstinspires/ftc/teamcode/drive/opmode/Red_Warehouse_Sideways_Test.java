package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Arm;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.TensorFlowObjectDetectionWebcam;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class Red_Warehouse_Sideways_Test extends LinearOpMode {

    public Robot robot;
    public TensorFlowObjectDetectionWebcam tfod;

    public TensorFlowObjectDetectionWebcam.MARKER_LOCATION markerLocation = TensorFlowObjectDetectionWebcam.MARKER_LOCATION.CENTER;

    private Trajectory drop;
    private Trajectory carousel;
    private Trajectory park, park1, park2, park3;

    public boolean parkInStorage = false;

    public Arm.HubLevel hubLevel = null;


    Pose2d startingPose = new Pose2d(12, -63.5, Math.toRadians(0));
    Pose2d dropLocation = new Pose2d(-2, -46, Math.toRadians(120));


    Pose2d parkWarehouse1 = new Pose2d(12, -65, Math.toRadians(0));
    Pose2d parkWarehouse2 = new Pose2d(38, -66, Math.toRadians(0));
    Pose2d parkWarehouse3 = new Pose2d(45, -45, Math.toRadians(-45));
    Pose2d parkWarehouseEnd = new Pose2d(66, -39, Math.toRadians(-92));

    Pose2d grab1 = new Pose2d(48, -66, Math.toRadians(0));

    Pose2d backup1 = new Pose2d(10, -65, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        tfod = new TensorFlowObjectDetectionWebcam(hardwareMap, telemetry);

        robot.arm.resetEncoder(robot.arm.armMotor);
        robot.arm.resetEncoder(robot.arm.extensionMotor);

        //tfod.initDetector();

        robot.arm.useIntake(-0.2);

        robot.drive.getLocalizer().setPoseEstimate(startingPose);

        //Initial drop
        drop = robot.drive.trajectoryBuilder(startingPose)
                .lineToLinearHeading(dropLocation)
                .build();

        //Pickup
        park = robot.drive.trajectoryBuilder(drop.end())
                .lineToLinearHeading(parkWarehouse1)
                .build();
        park1 = robot.drive.trajectoryBuilder(park.end())
                .lineTo(parkWarehouse2.vec())
                .build();
        TrajectorySequence parkSequence = robot.drive.trajectorySequenceBuilder(drop.end())
                .lineToLinearHeading(parkWarehouse1)
                .lineTo(parkWarehouse2.vec())
                .build();

        Trajectory grab = robot.drive.trajectoryBuilder(park1.end())
                .splineTo(grab1.vec(), grab1.getHeading())
                .build();



        //TODO: Add vision handling.  Should result in markerLocation indicating marker position.
        while (!isStarted()) {
            //markerLocation = tfod.locateMarker();
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

                robot.arm.moveArmToTarget(Arm.MovingMode.START, 300, 0.8, 5);
        robot.arm.moveExtensionToTarget(Arm.MovingMode.START, -300, 0.8, 5);
        robot.arm.useIntake(-0.8);
        robot.drive.followTrajectorySequenceAsync(parkSequence);

        while (robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy()) {
            robot.drive.update();
        }

        robot.drive.followTrajectory(grab);

        boolean gotIt = robot.arm.intakeSense(3);
        telemetry.addData("Got block:", gotIt);
        telemetry.update();

        if (gotIt) {
            Trajectory backout = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate(), true)
                    .lineToLinearHeading(backup1)
                    .build();
            robot.arm.moveArmToTarget(Arm.MovingMode.START, 1500, 0.8, 5);
            robot.drive.followTrajectoryAsync(backout);
            while (robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy()) {
                robot.drive.update();
            }

            Trajectory dropAgain = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .lineToLinearHeading(dropLocation)
                    .build();

            robot.arm.moveArmToTarget(Arm.MovingMode.START, robot.arm.getArmTarget(Arm.HubLevel.TOP), 0.8, 5);
            robot.arm.moveExtensionToTarget(Arm.MovingMode.START, robot.arm.getExtensionTarget(Arm.HubLevel.TOP), 0.8, 5);
            robot.drive.followTrajectoryAsync(dropAgain);

            while (robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy()) {
                robot.drive.update();
            }

            robot.arm.spitIntake();
            sleep(500);

            park = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .lineToLinearHeading(parkWarehouse1)
                    .build();

            robot.drive.followTrajectory(park);
        }


        parkSequence = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .lineTo(parkWarehouse2.vec())
                .lineToLinearHeading(parkWarehouse3)
                .lineToLinearHeading(parkWarehouseEnd)
                .build();

        robot.drive.followTrajectorySequence(parkSequence);

        robot.arm.moveArmToTarget(Arm.MovingMode.START, 300, 0.8, 5);
        robot.arm.moveExtensionToTarget(Arm.MovingMode.START, -300, 0.8, 5);
        robot.arm.useIntake(-0.8);

        while(robot.arm.armIsBusy() || robot.arm.extensionIsBusy()){
        }

        grab = robot.drive.trajectoryBuilder(parkWarehouseEnd)
                .forward(10)
                .build();

        robot.drive.followTrajectory(grab);

        gotIt = robot.arm.intakeSense(5);
        telemetry.addData("Got block:", gotIt);
        telemetry.update();

        if(gotIt){
            robot.arm.moveArmToTarget(Arm.MovingMode.START, 1500, 0.8, 5);
            while(robot.arm.armIsBusy() || robot.arm.extensionIsBusy()){
            }
        }

        tfod.shutdown();

    }



}
