package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Arm;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.TensorFlowObjectDetectionWebcam;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.KRASHMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.drive.KRASHMecanumDrive.getVelocityConstraint;

@Autonomous
public class Red_Warehouse_Sideways extends LinearOpMode {

    public Robot robot;
    public TensorFlowObjectDetectionWebcam tfod;

    public TensorFlowObjectDetectionWebcam.MARKER_LOCATION markerLocation = TensorFlowObjectDetectionWebcam.MARKER_LOCATION.CENTER;

    private Trajectory drop;
    private Trajectory carousel;
    private Trajectory park, park1, park2, park3;

    public boolean parkInStorage = false;

    public Arm.HubLevel hubLevel = null;

    private static final TrajectoryVelocityConstraint SLOW_CONSTRAINT = getVelocityConstraint(MAX_VEL * 0.5, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint SLOW_ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);



    Pose2d startingPose = new Pose2d(10, -63.5, Math.toRadians(0));
    Pose2d dropLocation = new Pose2d(-2, -46, Math.toRadians(120));
    Pose2d secondDropLocation = new Pose2d(-4, -44, Math.toRadians(120));

    Pose2d outsideWarehouse = new Pose2d(12, -65, Math.toRadians(0));
    Pose2d insideWarehouse = new Pose2d(38, -66, Math.toRadians(0));
    Pose2d midPointParking = new Pose2d(45, -45, Math.toRadians(-45));
    Pose2d finalWarehousePosition = new Pose2d(66, -39, Math.toRadians(-92));

    Pose2d grab1 = new Pose2d(48, -66, Math.toRadians(0));

    //Pose2d backup1 = new Pose2d(10, -65, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        tfod = new TensorFlowObjectDetectionWebcam(hardwareMap, telemetry);

        robot.arm.resetEncoder(robot.arm.armMotor);
        robot.arm.resetEncoder(robot.arm.extensionMotor);
        robot.arm.resetEncoder(robot.arm.spinnerMotor);

        tfod.initDetector();

        robot.arm.useIntake(0.2);

        robot.drive.getLocalizer().setPoseEstimate(startingPose);

        //Initial drop
        drop = robot.drive.trajectoryBuilder(startingPose)
                .lineToLinearHeading(dropLocation)
                .build();

        //Pickup
        TrajectorySequence outsideWarehouseSequence = robot.drive.trajectorySequenceBuilder(drop.end())
                .lineToLinearHeading(outsideWarehouse)
                .strafeRight(5)
                .build();

        park = robot.drive.trajectoryBuilder(drop.end())
                .lineToLinearHeading(outsideWarehouse)
                .build();
        park1 = robot.drive.trajectoryBuilder(park.end())
                .lineTo(insideWarehouse.vec())
                .build();




        while (!isStarted() && !isStopRequested()) {
            markerLocation = tfod.locateMarker();
            hubLevel = robot.arm.markerToLevel(markerLocation);
            telemetry.addData("Marker Location:", markerLocation);
            telemetry.update();
        }

        //Initial Drop
        robot.arm.moveArmToTarget(Arm.MovingMode.START, robot.arm.getArmTarget(hubLevel), 0.8, 5);
        robot.arm.moveExtensionToTarget(Arm.MovingMode.START, robot.arm.getExtensionTarget(hubLevel), 0.8, 5);
        robot.drive.followTrajectoryAsync(drop);

        while (robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy()) {
            robot.drive.update();
        }

        robot.arm.spitIntake(-0.5);
        sleep(500);

        //Move arm high to avoid hitting anything and move to wall near warehouse
        robot.arm.moveArmToTarget(Arm.MovingMode.START, robot.arm.SAFE_HIGH_ARM, 1.0, 5);
        robot.drive.followTrajectorySequenceAsync(outsideWarehouseSequence);
        while (robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy()) {
            robot.drive.update();
        }

        robot.arm.moveArmToTarget(Arm.MovingMode.START, 300, 0.8, 5);
        robot.arm.moveExtensionToTarget(Arm.MovingMode.START, -300, 0.8, 5);
        robot.arm.useIntake(-0.8);
        //robot.drive.followTrajectoryAsync(park1);

        while (robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy()) {
            robot.drive.update();
        }

        Trajectory grab = robot.drive.trajectoryBuilder(outsideWarehouseSequence.end())
                .splineTo(grab1.vec(), grab1.getHeading(),SLOW_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
                .build();

        robot.drive.followTrajectory(grab);

        boolean gotIt = robot.arm.intakeSense(3);
        telemetry.addData("Got block:", gotIt);
        telemetry.update();

        //Uncomment the line below to skip cycling and park in the warehouse near the shared hub
        //gotIt = false;

        if (gotIt) {
            TrajectorySequence backout = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                    .strafeRight(2)
                    .back(38)
                    .build();
            robot.arm.moveArmToTarget(Arm.MovingMode.START, robot.arm.SAFE_HIGH_ARM, 0.8, 5);
            robot.drive.followTrajectorySequence(backout);
            while (robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy()) {
                robot.drive.update();
            }

            Trajectory dropAgain = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .lineToLinearHeading(secondDropLocation)
                    .build();

            robot.arm.moveArmToTarget(Arm.MovingMode.START, robot.arm.TOP_NORMAL_HUB_COUNTS, 0.8, 5);
            robot.arm.moveExtensionToTarget(Arm.MovingMode.START, robot.arm.getExtensionTarget(Arm.HubLevel.TOP), 0.8, 5);
            robot.drive.followTrajectoryAsync(dropAgain);

            while (robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy()) {
                robot.drive.update();
            }

            robot.arm.spitIntake(0.8);
            sleep(500);

            robot.arm.moveArmToTarget(Arm.MovingMode.START, robot.arm.SAFE_HIGH_ARM, 1.0, 5);
            robot.drive.followTrajectorySequenceAsync(outsideWarehouseSequence);
            while (robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy()) {
                robot.drive.update();
            }
        } else {
            robot.arm.moveExtensionToTarget(Arm.MovingMode.START, -50, 0.8, 5);
            robot.arm.moveArmToTarget(Arm.MovingMode.START, robot.arm.SAFE_HIGH_ARM, 1.0, 5);

            while (robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy()) {
            }
        }

        TrajectorySequence finalParkSeq = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .lineTo(insideWarehouse.vec())
                .lineToLinearHeading(midPointParking)
                .lineToLinearHeading(finalWarehousePosition)
                .build();

        robot.drive.followTrajectorySequence(finalParkSeq);
        while(robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy() || robot.arm.turretIsBusy()){
            robot.drive.update();
        }

        robot.arm.moveArmToTarget(Arm.MovingMode.START, 300, 0.8, 5);
        robot.arm.moveExtensionToTarget(Arm.MovingMode.START, -300, 0.8, 5);
        robot.arm.useIntake(-0.8);

        while(robot.arm.armIsBusy() || robot.arm.extensionIsBusy()){
        }

        grab = robot.drive.trajectoryBuilder(finalWarehousePosition)
                .forward(10)
                .build();

        robot.drive.followTrajectory(grab);

        gotIt = robot.arm.intakeSense(5);
        telemetry.addData("Got block:", gotIt);
        telemetry.update();

        if(gotIt){
            robot.arm.moveArmToTarget(Arm.MovingMode.START, 2500, 1.0, 5);
            while(robot.arm.armIsBusy() || robot.arm.extensionIsBusy()){
            }
        }

        tfod.shutdown();
    }




}
