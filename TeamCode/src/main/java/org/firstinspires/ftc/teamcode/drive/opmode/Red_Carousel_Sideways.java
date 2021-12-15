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
public class Red_Carousel_Sideways extends LinearOpMode {

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

    Pose2d startingPose = new Pose2d(-38,-63.5,Math.toRadians(0));
    Pose2d carouselLocation = new Pose2d(-61.5, -57.5, Math.toRadians(95));
    Pose2d dropLocation = new Pose2d(-21.5, -46.5, Math.toRadians(70));
    Pose2d parkStorageLoc = new Pose2d(-65, -31, Math.toRadians(0)); //reversed

    //Pose2d parkWarehouse0 = new Pose2d(-25, -50, Math.toRadians(-45));
    Pose2d outsideWarehouse = new Pose2d(0, -65, Math.toRadians(0));
    Pose2d insideWarehouse = new Pose2d(38, -66, Math.toRadians(0));
    Pose2d midPointParking = new Pose2d(45, -45, Math.toRadians(-45));
    Pose2d finalWarehousePosition = new Pose2d(66, -39, Math.toRadians(-92));

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        tfod = new TensorFlowObjectDetectionWebcam(hardwareMap, telemetry);

        robot.arm.resetEncoder(robot.arm.armMotor);
        robot.arm.resetEncoder(robot.arm.extensionMotor);
        robot.arm.resetEncoder(robot.arm.spinnerMotor);

        robot.arm.useIntake(0.2);

        tfod.initDetector();

        robot.drive.getLocalizer().setPoseEstimate(startingPose);

        TrajectorySequence carouselSequence = robot.drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(carouselLocation)
                .back(2.5, SLOW_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
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
                .strafeRight(5)
                .lineTo(insideWarehouse.vec())
                .build();

/*            park1 = robot.drive.trajectoryBuilder(park.end())
                    .lineTo(parkWarehouse2.vec())
                    .build();
            park2 = robot.drive.trajectoryBuilder(park1.end())
                    .lineToLinearHeading(parkWarehouse3)
                    .build();
            park3 = robot.drive.trajectoryBuilder(park2.end())
                    .lineToLinearHeading(parkWarehouseEnd)
                    .build();*/


        //TODO: Add vision handling.  Should result in markerLocation indicating marker position.
        while (!isStarted() && !isStopRequested()){
            markerLocation = tfod.locateMarker();
            hubLevel = robot.arm.markerToLevel(markerLocation);

            if (gamepad1.dpad_up){
                parkInStorage = true;
                telemetry.addLine("Park in Storage");
            } else if (gamepad1.dpad_down){
                parkInStorage = false;
                telemetry.addLine("Park in Warehouse");
            }

            telemetry.addData("Marker Location:", markerLocation);
            telemetry.update();
        }

        //Basic Drive

        /** Move to carousel and get duck */

        robot.drive.followTrajectorySequence(carouselSequence);

        robot.drive.runCarousel(-1.0);
        sleep(4200);
        robot.drive.runCarousel(0);

        /** Move to drop location and drop freight */
        robot.arm.moveArmToTarget(Arm.MovingMode.START, robot.arm.getArmTarget(hubLevel), 0.8, 5);
        robot.arm.moveExtensionToTarget(Arm.MovingMode.START, robot.arm.getExtensionTarget(hubLevel), 0.8, 5);
        robot.drive.followTrajectoryAsync(drop);

        while(robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy()){
            robot.drive.update();
        }

        robot.arm.moveArmToTarget(Arm.MovingMode.STOP,0,0,1);

        robot.arm.spitIntake(robot.arm.dropPower(hubLevel));


        /** Raise arm to get it out of the way and move to park location or begin moving towards warehouse*/
        robot.arm.moveExtensionToTarget(Arm.MovingMode.START, -50, 0.8, 5);
        robot.arm.moveArmToTarget(Arm.MovingMode.START, robot.arm.SAFE_HIGH_ARM, 1.0, 5);

        if(parkInStorage) {
            robot.drive.followTrajectoryAsync(parkStore);
        } else {
            robot.drive.followTrajectoryAsync(lineupOutsideWarehouse);
        }

        while(robot.drive.isBusy() || robot.arm.armIsBusy() || robot.arm.extensionIsBusy()){
            robot.drive.update();
        }



        if(!parkInStorage){
            /** You can insert a delay here if the other team needs time to move first
             * Uncomment the sleep line below */
            //sleep(1000);  //This will sleep 1s

            robot.drive.followTrajectorySequence(goInsideWarehouse);  //Comment out everything AFTER this line to just stop in the entrance to the warehouse

            TrajectorySequence finalParkSeq = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                    .lineToLinearHeading(midPointParking)
                    .lineToLinearHeading(finalWarehousePosition)
                    .build();

            robot.drive.followTrajectorySequence(finalParkSeq);


            /** Move arm to intake position */
            robot.arm.moveArmToTarget(Arm.MovingMode.START, 300, 0.8, 5);
            robot.arm.moveExtensionToTarget(Arm.MovingMode.START, -300, 0.8, 5);
            robot.arm.useIntake(-0.8);

            while(robot.arm.armIsBusy() || robot.arm.extensionIsBusy()){
            }

            Trajectory grab = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .forward(10)
                    .build();

            robot.drive.followTrajectory(grab);

            boolean gotIt = robot.arm.intakeSense(5);
            telemetry.addData("Got block:", gotIt);
            telemetry.update();

            if(gotIt){
                robot.arm.moveArmToTarget(Arm.MovingMode.START, 2500, 1.0, 5);
                while(robot.arm.armIsBusy() || robot.arm.extensionIsBusy()){
                }
            }
        }

        tfod.shutdown();

    }
}
