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
public class Blue_Carousel_Sideways extends LinearOpMode {

    public Robot robot;
    public TensorFlowObjectDetectionWebcam tfod;

    public TensorFlowObjectDetectionWebcam.MARKER_LOCATION markerLocation = TensorFlowObjectDetectionWebcam.MARKER_LOCATION.RIGHT;

    private Trajectory drop;
    private Trajectory carousel;
    private Trajectory lineupOutsideWarehouse, park1, park2, park3;

    public boolean parkInStorage = false;

    public Arm.HubLevel hubLevel = null;

    private static final TrajectoryVelocityConstraint SLOW_CONSTRAINT = getVelocityConstraint(MAX_VEL * 0.5, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint SLOW_ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    Pose2d startingPose = new Pose2d(-36, 63.5, Math.toRadians(179.99));
    Pose2d carouselLocation = new Pose2d(-62.5, 55, Math.toRadians(0));
    Pose2d dropLocation = new Pose2d(-19.5, 45.0, Math.toRadians(-75));
    Pose2d parkStorageLoc = new Pose2d(-67, 34, Math.toRadians(0)); //reversed

    //Pose2d parkWarehouse0 = new Pose2d(-25, -50, Math.toRadians(-45));
    Pose2d outsideWarehouse = new Pose2d(0, 65, Math.toRadians(0));
    Pose2d insideWarehouse = new Pose2d(38, 66, Math.toRadians(0));
    Pose2d midPointParking = new Pose2d(45, 45, Math.toRadians(45));
    Pose2d finalWarehousePosition = new Pose2d(66, 39, Math.toRadians(-90));


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

        TrajectorySequence carouselSequence = robot.drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(carouselLocation)
                .strafeLeft(3, SLOW_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
                .build();

        drop = robot.drive.trajectoryBuilder(carouselSequence.end())
                .lineToLinearHeading(dropLocation)
                .build();

        //Storage park
        Trajectory parkStore = robot.drive.trajectoryBuilder(drop.end(), true)
                .lineToLinearHeading(parkStorageLoc)
                .build();

        //Warehouse park
        lineupOutsideWarehouse = robot.drive.trajectoryBuilder(drop.end())
                .lineToLinearHeading(outsideWarehouse)
                //.lineTo(parkWarehouse2.vec())
                //.lineToLinearHeading(parkWarehouseEnd)
                .build();

        TrajectorySequence goInsideWarehouse = robot.drive.trajectorySequenceBuilder(lineupOutsideWarehouse.end())
                .strafeLeft(5)
                .lineTo(insideWarehouse.vec())
                .build();


            /*park1 = robot.drive.trajectoryBuilder(park.end())
                    .lineTo(parkWarehouse2.vec())
                    .build();
            park2 = robot.drive.trajectoryBuilder(park1.end())
                    .lineToLinearHeading(parkWarehouse3)
                    .build();
            park3 = robot.drive.trajectoryBuilder(park2.end())
                    .lineToLinearHeading(parkWarehouseEnd)
                    .build();*/


        //TODO: Add vision handling.  Should result in markerLocation indicating marker position.
        while (!isStarted() && !isStopRequested()) {
            String parkText;
            markerLocation = tfod.locateMarker();
            hubLevel = robot.arm.markerToLevel(markerLocation);

            if (gamepad1.dpad_up) {
                parkInStorage = true;
                telemetry.addLine("Park in Storage");
                parkText = "Park in Storage";
            } else if (gamepad1.dpad_down) {
                parkInStorage = false;
                telemetry.addLine("Park in Warehouse");
            }


            telemetry.addData("Marker Location:", markerLocation);
            telemetry.update();
        }

        //Basic Drive

        /** Move to carousel and get duck */
        robot.drive.followTrajectorySequence(carouselSequence);

        robot.drive.runCarousel(1.0);
        sleep(4200);
        robot.drive.runCarousel(0);


        /** Move to drop location and drop freight */
        robot.arm.moveArmToTarget(Arm.MovingMode.START, robot.arm.getArmTarget(hubLevel), 0.8, 5);
        robot.arm.moveExtensionToTarget(Arm.MovingMode.START, robot.arm.getExtensionTarget(hubLevel), 0.8, 5);
        robot.drive.followTrajectoryAsync(drop);

        while (robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy()) {
            robot.drive.update();
        }

        robot.arm.spitIntake(-0.5);


        /** Raise arm to get it out of the way and move to park location or begin moving towards warehouse*/
        robot.arm.moveExtensionToTarget(Arm.MovingMode.START, -50, 0.8, 5);
        robot.arm.moveArmToTarget(Arm.MovingMode.START, robot.arm.SAFE_HIGH_ARM, 1.0, 5);

        if (parkInStorage) {
            robot.drive.followTrajectoryAsync(parkStore);
        } else {
            robot.drive.followTrajectoryAsync(lineupOutsideWarehouse);
        }


        while (robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy()) {
            robot.drive.update();
        }

        if (!parkInStorage) {
            /** You can insert a delay here if the other team needs time to move first
             * Uncomment the sleep line below */
            //sleep(1000);  //This will sleep 1 second

            robot.drive.followTrajectorySequence(goInsideWarehouse);  //Comment out everything AFTER this line to just stop in the entrance to the warehouse


            TrajectorySequence finalParkSeq = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                    .lineToLinearHeading(midPointParking)
                    .turn(Math.toRadians(180))
                    .lineToLinearHeading(finalWarehousePosition)
                    .build();

            //Move turret for 3 seconds to give it a head start.  This turret may not finish before timeout
            robot.arm.moveTurretToTarget(Arm.MovingMode.START, robot.arm.BACK_TURRET_LIMIT, 1.0, 3);
            robot.drive.followTrajectorySequence(finalParkSeq);
            while (robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy() || robot.arm.turretIsBusy()) {
                robot.drive.update();
            }


            //Flip arm to back intake position
            robot.arm.moveTurretToTarget(Arm.MovingMode.START, robot.arm.BACK_TURRET_LIMIT, 1.0, 5);
            robot.arm.moveArmToTarget(Arm.MovingMode.START, 300, 0.8, 5);
            robot.arm.moveExtensionToTarget(Arm.MovingMode.START, -300, 0.8, 5);
            robot.arm.useIntake(-0.8);

            while (robot.arm.armIsBusy() || robot.arm.extensionIsBusy() || robot.arm.turretIsBusy()) {
            }

            Trajectory grab = robot.drive.trajectoryBuilder(finalWarehousePosition)
                    .back(10)
                    .build();

            robot.drive.followTrajectory(grab);

            boolean gotIt = robot.arm.intakeSense(5);
            telemetry.addData("Got block:", gotIt);
            telemetry.update();

            if (gotIt) {
                robot.arm.moveArmToTarget(Arm.MovingMode.START, 2500, 1.0, 5);
                while (robot.arm.armIsBusy() || robot.arm.extensionIsBusy()) {
                }
            }

            tfod.shutdown();

        }
    }
}
