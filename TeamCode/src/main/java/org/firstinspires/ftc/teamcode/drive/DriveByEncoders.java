package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.util.Sleep.sleep;

public class DriveByEncoders {
    public boolean CompetitionMode = true;

    public KRASHMecanumDrive drive;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    private final String IMU_GYRO = "imu";

    private final int MEC_DRIVE_CARDINAL_COUNTS_PER_INCH = 46;
    private final int MEC_DRIVE_DIAG_COUNTS_PER_INCH = 46;

    public double STRAFE_CORRECTION_FACTOR = 0;
    public double AUTON_DRIVE_RAMP_RATE = 0.25; //Rate per second.  1 = 100% power in 1 s.
    public double AUTON_DRIVE_START_FACTOR = 0.5;

    public enum MEC_DRIVE_AUTO_DIRECTION
    {
        FORWARD,
        REVERSE,
        LEFT,
        RIGHT,
        DIAG_FORWARD_RIGHT,
        DIAG_FORWARD_LEFT,
        DIAG_REVERSE_RIGHT,
        DIAG_REVERSE_LEFT
    }


    // private references that need to be kept locally
    private ElapsedTime m_elapsedTime = new ElapsedTime();

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double HEADING_THRESHOLD = 1 ; // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.085;    // Larger is more responsive, but also less stable (default was .09)
    static final double P_DRIVE_COEFF = 0.15;   // Larger is more responsive, but also less stable (default was .15)

    BNO055IMU Gyro = null;
    Orientation GyroAngles = null;
    Acceleration GyroGravity = null;
    BNO055IMU.Parameters GyroParameters = null;

    public DriveByEncoders(HardwareMap hardwareMap, KRASHMecanumDrive drive, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.drive = drive;
        this.telemetry = telemetry;
    }

    public void InitializeGryo()
    {
        // initialize the gyro
        telemetry.addData("Status"," Starting Gyro Init");
        telemetry.update();

        if (Gyro == null)
        {
            telemetry.addData("Status"," Null Gyro");
            telemetry.update();
            sleep(10000);
        }
        else
        {
            // finish intializing the gyro
            GyroParameters = new BNO055IMU.Parameters();
            GyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            GyroParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            GyroParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            GyroParameters.loggingEnabled = true;
            GyroParameters.loggingTag = "IMU";
            GyroParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            telemetry.addData("Status","Initializing gyro...");
            telemetry.update();
            Gyro.initialize(GyroParameters);

            telemetry.addData("Status","Starting acceleration integration on gyro...");
            telemetry.update();
            Gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);

            telemetry.addData("Status","Composing telemetry...");
            telemetry.update();
            ComposeGyroTelemetry();
            telemetry.update();
        }
    }

    public void RecalibrateGryo()
    {
        // initialize the gyro
        if (Gyro == null)
        {
            //m_opModeConfig.LogError("Gyro is null in RecalibrateGryo()");
            telemetry.addData("Status"," Null Gyro");
            telemetry.update();
        }
        else
        {
            // just make a call that uses the gyro, and it will self initialize
            Gyro.initialize(GyroParameters);
            while (!Gyro.isGyroCalibrated())
            {
                sleep(50);
            }

            ComposeGyroTelemetry();
        }
    }

    public void ComposeGyroTelemetry()
    {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the GyroAngles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            GyroAngles = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            GyroGravity = Gyro.getGravity();
        }
        });

        /**
        //The following can be useful for logging info or issues outside of competition
        //Note that it is configured for an older version of Howell FTC code and may need
        //some updating to work correctly.
         */
        /*if (!CompetitionMode)
        {
            m_opModeConfig.Telemetry.addLine()
                    .addData("status", new Func<String>() {
                        @Override
                        public String value() {
                            return Gyro.getSystemStatus().toShortString();
                        }
                    })
                    .addData("calib", new Func<String>() {
                        @Override
                        public String value() {
                            return Gyro.getCalibrationStatus().toString();
                        }
                    });

            m_opModeConfig.Telemetry.addLine()
                    .addData("heading", new Func<String>() {
                        @Override
                        public String value() {
                            return FormatGyroAngle(GyroAngles.angleUnit, GyroAngles.firstAngle);
                        }
                    })
                    .addData("roll", new Func<String>() {
                        @Override
                        public String value() {
                            return FormatGyroAngle(GyroAngles.angleUnit, GyroAngles.secondAngle);
                        }
                    })
                    .addData("pitch", new Func<String>() {
                        @Override
                        public String value() {
                            return FormatGyroAngle(GyroAngles.angleUnit, GyroAngles.thirdAngle);
                        }
                    });
        }*/
    }

    public String FormatGyroAngle(AngleUnit angleUnit, double angle)
    {
        return FormatGyroDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String FormatGyroDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void TurnByEncoder(double degree, double power, double timeout)
    {
        //m_opModeConfig.LogInfo("Autonomously turning, direction: " + degree +
        //        ", power: " + power + ", timeout: " + timeout + "...");
        power = Range.clip(Math.abs(power), -1.0, 1.0);
        TrySetDriveMotorsRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TrySetDriveMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        int targetCounts = Math.abs((int) (degree * 11.4));
        if (degree < 0) {
            // turn the front to the left
            //                          rightFront   rightRear      leftFront      LeftRear
            TrySetDriveMotorsToPosition(targetCounts, targetCounts, -targetCounts, -targetCounts);
            TrySetDriveMotorsPower(power, power, -power, -power);
        }
        else {
            // turn the front to the right
            TrySetDriveMotorsToPosition(-targetCounts, -targetCounts, targetCounts, targetCounts);
            TrySetDriveMotorsPower(-power, -power, power, power);
        }

        m_elapsedTime.reset();
        while ((m_elapsedTime.seconds() < timeout) &&
                (IsBusy(drive.leftFront) || IsBusy(drive.leftRear) || IsBusy(drive.rightFront) || IsBusy(drive.rightRear)))
        {
            // log something if you want
        }

        StopRobot();
        //m_opModeConfig.LogInfo("Completed autonomous step, direction: " + degree);
    }

    public boolean IsBusy(DcMotor motor)
    {
        int allowedVariance = 60;
        if (motor != null)
        {
            int currentPosition = motor.getCurrentPosition();
            int targetPosition = motor.getTargetPosition();

            if (((currentPosition - allowedVariance) <= targetPosition) && (targetPosition <= (currentPosition + allowedVariance)))
            {
                return false;
            }else
            {
                return motor.isBusy();
            }
        }
        else
        {
            return false;
        }
    }

    public void AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION direction, double inchesToTravel, double power, double timeout)
    {
        double driftCorrection = 0;
        power = Range.clip(Math.abs(power), -1.0, 1.0);
        //m_opModeConfig.LogInfo("Autonomously driving, direction: " + direction.toString() + ", inchedToTravel: " + inchesToTravel +
        //    ", power: " + power + ", timeout: " + timeout + "...");
        TrySetDriveMotorsRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TrySetDriveMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        int targetCardinalEncoderCount = (int) (inchesToTravel * MEC_DRIVE_CARDINAL_COUNTS_PER_INCH);
        int targetDiagonalEncoderCount = (int) (inchesToTravel * MEC_DRIVE_DIAG_COUNTS_PER_INCH);
        //m_opModeConfig.LogInfo("targetCardinalEncoderCount: " + targetCardinalEncoderCount +
        //        ", targetDiagonalEncoderCount: " + targetDiagonalEncoderCount);
        double pwr = power*AUTON_DRIVE_START_FACTOR;
        switch (direction) {
            case FORWARD:
                TrySetDriveMotorsToPosition(targetCardinalEncoderCount);
                TrySetDriveMotorsPower(pwr);
                m_elapsedTime.reset();
                while ((m_elapsedTime.seconds() < timeout) &&
                        (IsBusy(drive.leftFront) && IsBusy(drive.leftRear) &&
                                IsBusy(drive.rightFront) && IsBusy(drive.rightRear)))
                //    (leftFrontMotor.isBusy() || leftRearMotor.isBusy() ||
                //            rightFrontMotor.isBusy() || rightRearMotor.isBusy()))
                {
                    // Ramp to target power
                    if (pwr < power){
                        pwr = pwr + m_elapsedTime.seconds() * AUTON_DRIVE_RAMP_RATE;
                        if (pwr > power){
                            pwr = power;
                        }
                        TrySetDriveMotorsPower(pwr);
                    }
                }
                if (m_elapsedTime.seconds() >= timeout)
                {
                    /*m_opModeConfig.LogError("Timeout exceeded: " + timeout +
                        ", leftFrontMotor: " + targetCardinalEncoderCount + "/" + leftFrontMotor.getCurrentPosition() +
                        ", leftRearMotor: " + targetCardinalEncoderCount + "/" + leftRearMotor.getCurrentPosition() +
                        ", rightFrontMotor: " + targetCardinalEncoderCount + "/" + rightFrontMotor.getCurrentPosition() +
                        ", rightRearMotor: " + targetCardinalEncoderCount + "/" + rightRearMotor.getCurrentPosition());*/
                }
                break;
            case REVERSE:
                TrySetDriveMotorsToPosition(-targetCardinalEncoderCount);
                TrySetDriveMotorsPower(-pwr);
                m_elapsedTime.reset();
                while ((m_elapsedTime.seconds() < timeout) &&
                        (IsBusy(drive.leftFront) && IsBusy(drive.leftRear) &&
                                IsBusy(drive.rightFront) && IsBusy(drive.rightRear)))
                //    (leftFrontMotor.isBusy() || leftRearMotor.isBusy() ||
                //            rightFrontMotor.isBusy() || rightRearMotor.isBusy()))
                {
                    // Ramp to target power
                    if (pwr < power){
                        pwr = pwr + m_elapsedTime.seconds() * AUTON_DRIVE_RAMP_RATE;
                        if (pwr > power){
                            pwr = power;
                        }
                        TrySetDriveMotorsPower(-pwr);
                    }
                }
                if (m_elapsedTime.seconds() >= timeout)
                {
                    /*m_opModeConfig.LogError("Timeout exceeded: " + timeout +
                            ", leftFrontMotor: " + -targetCardinalEncoderCount + "/" + leftFrontMotor.getCurrentPosition() +
                            ", leftRearMotor: " + -targetCardinalEncoderCount + "/" + leftRearMotor.getCurrentPosition() +
                            ", rightFrontMotor: " + -targetCardinalEncoderCount + "/" + rightFrontMotor.getCurrentPosition() +
                            ", rightRearMotor: " + -targetCardinalEncoderCount + "/" + rightRearMotor.getCurrentPosition());*/
                }
                break;
            case LEFT:
                driftCorrection = 0.1;
                TrySetDriveMotorsToPosition(targetCardinalEncoderCount, -targetCardinalEncoderCount, -targetCardinalEncoderCount, targetCardinalEncoderCount);
                TrySetDriveMotorsPower(pwr*(1+STRAFE_CORRECTION_FACTOR) + driftCorrection,
                        -pwr*(1+STRAFE_CORRECTION_FACTOR) + driftCorrection,
                        -pwr*(1-STRAFE_CORRECTION_FACTOR)+driftCorrection,
                        pwr*(1-STRAFE_CORRECTION_FACTOR)+driftCorrection);
                m_elapsedTime.reset();
                while ((m_elapsedTime.seconds() < timeout) &&
                        (IsBusy(drive.leftFront) && IsBusy(drive.leftRear) &&
                                IsBusy(drive.rightFront) && IsBusy(drive.rightRear)))
                //    (leftFrontMotor.isBusy() || leftRearMotor.isBusy() ||
                //            rightFrontMotor.isBusy() || rightRearMotor.isBusy()))
                {
                    // Ramp to target power
                    if (pwr < power){
                        pwr = pwr + m_elapsedTime.seconds() * AUTON_DRIVE_RAMP_RATE;
                        if (pwr > power){
                            pwr = power;
                        }
                        TrySetDriveMotorsPower(pwr*(1+STRAFE_CORRECTION_FACTOR), -pwr*(1+STRAFE_CORRECTION_FACTOR), -pwr*(1-STRAFE_CORRECTION_FACTOR), pwr*(1-STRAFE_CORRECTION_FACTOR));
                    }
                }
                if (m_elapsedTime.seconds() >= timeout)
                {
                    /*m_opModeConfig.LogError("Timeout exceeded: " + timeout +
                            ", leftFrontMotor: " + -targetCardinalEncoderCount + "/" + leftFrontMotor.getCurrentPosition() +
                            ", leftRearMotor: " + targetCardinalEncoderCount + "/" + leftRearMotor.getCurrentPosition() +
                            ", rightFrontMotor: " + targetCardinalEncoderCount + "/" + rightFrontMotor.getCurrentPosition() +
                            ", rightRearMotor: " + -targetCardinalEncoderCount + "/" + rightRearMotor.getCurrentPosition());*/
                }
                break;
            case RIGHT:
                driftCorrection = -0.1;
                TrySetDriveMotorsToPosition(-targetCardinalEncoderCount, targetCardinalEncoderCount, targetCardinalEncoderCount, -targetCardinalEncoderCount);
                TrySetDriveMotorsPower(
                        -pwr * (1 - STRAFE_CORRECTION_FACTOR) + driftCorrection,
                        pwr * (1 - STRAFE_CORRECTION_FACTOR) + driftCorrection,
                        pwr * (1 + STRAFE_CORRECTION_FACTOR) + driftCorrection,
                        -pwr * (1 + STRAFE_CORRECTION_FACTOR) + driftCorrection);
                m_elapsedTime.reset();
                while ((m_elapsedTime.seconds() < timeout) &&
                        (IsBusy(drive.leftFront) && IsBusy(drive.leftRear) &&
                                IsBusy(drive.rightFront) && IsBusy(drive.rightRear)))
                //    (leftFrontMotor.isBusy() || leftRearMotor.isBusy() ||
                //            rightFrontMotor.isBusy() || rightRearMotor.isBusy()))
                {
                    // Ramp to target power
                    if (pwr < power){
                        pwr = pwr + m_elapsedTime.seconds() * AUTON_DRIVE_RAMP_RATE;
                        if (pwr > power){
                            pwr = power;
                        }
                        TrySetDriveMotorsPower(
                                -pwr*(1-STRAFE_CORRECTION_FACTOR) + driftCorrection,
                                pwr*(1-STRAFE_CORRECTION_FACTOR) + driftCorrection,
                                pwr*(1+STRAFE_CORRECTION_FACTOR) + driftCorrection,
                                -pwr*(1+STRAFE_CORRECTION_FACTOR) + driftCorrection);
                    }
                }
                if (m_elapsedTime.seconds() >= timeout)
                {
                    /*m_opModeConfig.LogError("Timeout exceeded: " + timeout +
                            ", leftFrontMotor: " + targetCardinalEncoderCount + "/" + leftFrontMotor.getCurrentPosition() +
                            ", leftRearMotor: " + -targetCardinalEncoderCount + "/" + leftRearMotor.getCurrentPosition() +
                            ", rightFrontMotor: " + -targetCardinalEncoderCount + "/" + rightFrontMotor.getCurrentPosition() +
                            ", rightRearMotor: " + targetCardinalEncoderCount + "/" + rightRearMotor.getCurrentPosition());*/
                }
                break;
            case DIAG_REVERSE_LEFT:
                TrySetDriveMotorsToPosition(0,-targetDiagonalEncoderCount , -targetDiagonalEncoderCount, 0);
                TrySetDriveMotorsPower(0, -power, -power, 0);
                m_elapsedTime.reset();
                while ((m_elapsedTime.seconds() < timeout) &&
                        (drive.leftFront.isBusy() || drive.rightRear.isBusy()))
                {
                    // log something if you want
                }
                if (m_elapsedTime.seconds() >= timeout)
                {
                    /*m_opModeConfig.LogError("Timeout exceeded: " + timeout +
                            ", leftFrontMotor: " + -targetCardinalEncoderCount + "/" + leftFrontMotor.getCurrentPosition() +
                            ", rightRearMotor: " + -targetCardinalEncoderCount + "/" + rightRearMotor.getCurrentPosition());*/
                }
                break;
            case DIAG_REVERSE_RIGHT:
                TrySetDriveMotorsToPosition(-targetDiagonalEncoderCount, 0,  0, -targetDiagonalEncoderCount);
                TrySetDriveMotorsPower(-power, 0, 0, -power);
                m_elapsedTime.reset();
                while ((m_elapsedTime.seconds() < timeout) &&
                        (drive.leftRear.isBusy() || drive.rightFront.isBusy() ))
                {
                    // log something if you want
                }
                if (m_elapsedTime.seconds() >= timeout)
                {
                    /*m_opModeConfig.LogError("Timeout exceeded: " + timeout +
                            ", leftRearMotor: " + -targetCardinalEncoderCount + "/" + leftRearMotor.getCurrentPosition() +
                            ", rightFrontMotor: " + -targetCardinalEncoderCount + "/" + rightFrontMotor.getCurrentPosition());*/
                }
                break;
            case DIAG_FORWARD_LEFT:
                TrySetDriveMotorsToPosition(targetDiagonalEncoderCount,0 , 0, targetDiagonalEncoderCount);
                TrySetDriveMotorsPower(power, 0, 0, power);
                while ((m_elapsedTime.seconds() < timeout) &&
                        (drive.leftRear.isBusy() || drive.rightFront.isBusy() ))
                {
                    // log something if you want
                }
                if (m_elapsedTime.seconds() >= timeout)
                {
                    /*m_opModeConfig.LogError("Timeout exceeded: " + timeout +
                            ", leftRearMotor: " + targetCardinalEncoderCount + "/" + leftRearMotor.getCurrentPosition() +
                            ", rightFrontMotor: " + targetCardinalEncoderCount + "/" + rightFrontMotor.getCurrentPosition());*/
                }
                break;
            case DIAG_FORWARD_RIGHT:
                TrySetDriveMotorsToPosition(0, targetDiagonalEncoderCount, targetDiagonalEncoderCount, 0);
                TrySetDriveMotorsPower(0, power, power, 0);
                m_elapsedTime.reset();
                while ((m_elapsedTime.seconds() < timeout) &&
                        (drive.leftFront.isBusy() ||  drive.rightRear.isBusy()))
                {
                    // log something if you want
                }
                if (m_elapsedTime.seconds() >= timeout)
                {
                    /*m_opModeConfig.LogError("Timeout exceeded: " + timeout +
                            ", leftFrontMotor: " + targetCardinalEncoderCount + "/" + leftFrontMotor.getCurrentPosition() +
                            ", rightRearMotor: " + targetCardinalEncoderCount + "/" + rightRearMotor.getCurrentPosition());*/
                }
                break;
        }
        // stop the robot when done
        StopRobot();
        /*m_opModeConfig.LogInfo("Completed autonomous step, direction: " + direction.toString());*/
    }

    public void DriveForward(double inchesToTravel, double power, double timeout)
    {
        AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION.FORWARD, inchesToTravel, power, timeout);
    }

    public void DriveBackward(double inchesToTravel, double power, double timeout)
    {
        AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION.REVERSE, inchesToTravel, power, timeout);
    }

    public void StrafeLeft(double inchesToTravel, double power, double timeout)
    {
        AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION.LEFT, inchesToTravel, power, timeout);
    }

    public void StrafeRight(double inchesToTravel, double power, double timeout)
    {

        AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION.RIGHT, inchesToTravel, power, timeout);
    }

    public void StrafeDiagonalForwardRight(double inchesToTravel, double power, double timeout)
    {
        AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION.DIAG_FORWARD_RIGHT, inchesToTravel, power, timeout);
    }

    public void StrafeDiagonalForwardLeft(double inchesToTravel, double power, double timeout)
    {
        AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION.DIAG_FORWARD_LEFT, inchesToTravel, power, timeout);
    }

    public void StafeDiagonalReverseLeft(double inchesToTravel, double power, double timeout)
    {
        AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION.DIAG_REVERSE_LEFT, inchesToTravel, power, timeout);
    }

    public void StrafeDiagonalReverseRight(double inchesToTravel, double power, double timeout)
    {
        AutonomousDrive(MEC_DRIVE_AUTO_DIRECTION.DIAG_REVERSE_RIGHT, inchesToTravel, power, timeout);
    }

    public void TrySetDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior powerBehavior)
    {
        // turn on motor break mode or float mode
        //m_opModeConfig.LogInfo("Set:" + powerBehavior.toString());
        try
        {
            drive.rightFront.setZeroPowerBehavior(powerBehavior);
            drive.rightRear.setZeroPowerBehavior(powerBehavior);
            drive.leftFront.setZeroPowerBehavior(powerBehavior);
            drive.leftRear.setZeroPowerBehavior(powerBehavior);
        }
        catch (Exception ex)
        {
            // something went wrong
            //m_opModeConfig.LogError("Setting ZeroDriveMotorsPowerBehavior Exception: " + ex.getMessage() );
        }
    }

    public void TrySetDriveMotorsRunMode(DcMotor.RunMode runMode)
    {
        //m_opModeConfig.LogInfo("Set:" + runMode.toString());
        try
        {
            drive.rightFront.setMode(runMode);
            drive.rightRear.setMode(runMode);
            drive.leftFront.setMode(runMode);
            drive.leftRear.setMode(runMode);
        }
        catch (Exception ex)
        {
            // something went wrong
            //m_opModeConfig.LogError("Setting SetDriveMotorsRunMode Exception: " + ex.getMessage() );
        }
    }

    public void TrySetDriveMotorsPower(double power)
    {
        power = Range.clip(power, -1.0, 1.0);
        //m_opModeConfig.LogInfo("Setting all motor powers to: " + power);
        try
        {
            drive.rightFront.setPower(power);
            drive.rightRear.setPower(power);
            drive.leftFront.setPower(power);
            drive.leftRear.setPower(power);
        }
        catch (Exception ex)
        {
            // something went wrong
            //m_opModeConfig.LogError("Setting SetDriveMotorsPower Exception: " + ex.getMessage() );
        }
    }

    public void TrySetDriveMotorsPower(double rightFront, double rightRear, double leftFront, double leftRear)
    {
        /*m_opModeConfig.LogInfo("Setting motor powers, rightFront: " + rightFront + ", rightRear: " + rightRear
                + ", leftFront: " + leftFront + ", leftRear: " + leftRear);*/
        rightFront = Range.clip(rightFront, -1.0, 1.0);
        rightRear = Range.clip(rightRear, -1.0, 1.0);
        leftFront = Range.clip(leftFront, -1.0, 1.0);
        leftRear = Range.clip(leftRear, -1.0, 1.0);
        try
        {
            drive.rightFront.setPower(rightFront);
            drive.rightRear.setPower(rightRear);
            drive.leftFront.setPower(leftFront);
            drive.leftRear.setPower(leftRear);
        }
        catch (Exception ex)
        {
            // something went wrong
            //m_opModeConfig.LogError("Setting SetDriveMotorsPower Exception: " + ex.getMessage() );
        }
    }

    public void TrySetDriveMotorsToPosition(int position)
    {
        //m_opModeConfig.LogInfo("Setting all motor target positions to: " + position);
        try
        {
            drive.rightFront.setTargetPosition(position);
            drive.rightRear.setTargetPosition(position);
            drive.leftFront.setTargetPosition(position);
            drive.leftRear.setTargetPosition(position);
        }
        catch (Exception ex)
        {
            // something went wrong
            //m_opModeConfig.LogError("Setting TrySetDriveMotorsToPosition Exception: " + ex.getMessage() );
        }
    }

    public void TrySetDriveMotorsToPosition(int rightFront, int rightRear, int leftFront, int leftRear)
    {
        /*m_opModeConfig.LogInfo("Setting motor positions, rightFront: " + rightFront + ", rightRear: " + rightRear
                + ", leftFront: " + leftFront + ", leftRear: " + leftRear);*/
        try
        {
            drive.rightFront.setTargetPosition(rightFront);
            drive.rightRear.setTargetPosition(rightRear);
            drive.leftFront.setTargetPosition(leftFront);
            drive.leftRear.setTargetPosition(leftRear);
        }
        catch (Exception ex)
        {
            // something went wrong
            //m_opModeConfig.LogError("Setting TrySetDriveMotorsToPosition Exception: " + ex.getMessage() );
        }
    }

    public void StopRobot()
    {
//        m_opModeConfig.LogInfo("Entry");
        TrySetDriveMotorsPower(0);
    }

    /* getError determines the error between the target angle and the robot's current heading
        @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
        @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
        +ve error means the robot should turn LEFT (CCW) to reduce error.
    */
    public double getError(double targetAngle)
    {
        double robotError;

        // calculate error in -179 to +180 range
        if (Gyro == null)
        {
            //m_opModeConfig.LogError( "Gyro is null in getError()");
        }
        robotError = targetAngle - Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /* returns desired steering force. +/- 1 range. +ve = steer left
        @param error Error angle in robot relative degrees *
        @param PCoeff Proportional Gain Coefficient * @return
    */
    public double getSteer(double error, double PCoeff)
    {
        return Range.clip(error * PCoeff, -1, 1);
    }

    /*  Method to spin on central axis to point in a new direction.
        Move will stop if either of these conditions occur:
        1) Move gets to the heading (angle)
        2) Driver stops the opmode running.
            @param speed Desired speed of turn.
            @param angle Absolute Angle (in Degrees) relative to last gyro reset.
            0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
            If a relative angle is required, add/subtract from current heading.
    */
    public void gyroTurn (double speed, double angle, double holdTime)
    {
        TrySetDriveMotorsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // keep looping while we are still active, and not on heading.
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        while ((elapsedTime.time() < holdTime)
                && !onHeading(speed, angle, P_TURN_COEFF))
        {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
            sleep(10);
        }
    }

    /* Method to obtain & hold a heading for a finite amount of time
        Move will stop once the requested time has elapsed
        @param speed Desired speed of turn.
        @param angle Absolute Angle (in Degrees) relative to last gyro reset.
            0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
            If a relative angle is required, add/subtract from current heading.
        @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime)
    {
        TrySetDriveMotorsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // keep looping while we have time remaining.
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        while ((elapsedTime.time() < holdTime))
        {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
            sleep(10);
        }

        // Stop all motion;
        drive.leftFront.setPower(0);
        drive.rightFront.setPower(0);
        drive.leftRear.setPower(0);
        drive.rightRear.setPower(0);
    }

    /*  Perform one cycle of closed loop heading control.
        @param speed Desired speed of turn.
        @param angle Absolute Angle (in Degrees) relative to last gyro reset.
        0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
        If a relative angle is required, add/subtract from current heading.
        @param PCoeff Proportional Gain coefficient
        @return
    */
    public boolean onHeading(double speed, double angle, double PCoeff)
    {
        //m_opModeConfig.LogInfo("Entry");

        double error ;
        double steer ;
        boolean onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);
        if (Math.abs(error) <= HEADING_THRESHOLD)
        {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else
        {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        //                   rightFront   rightRear      leftFront      LeftRear
        TrySetDriveMotorsPower(rightSpeed, rightSpeed, leftSpeed, leftSpeed);

        // Display it for the driver.
        if (!CompetitionMode)
        {
            //TBD - use other logging
            telemetry.addData("Target", "%5.2f", angle);
            telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
            telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
            telemetry.update();

        }
        return onTarget;
    }

}
