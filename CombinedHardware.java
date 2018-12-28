package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CombinedHardware {

    //Define Drivetrain
    public DcMotor  left_front_drive   = null;
    public DcMotor  right_front_drive  = null;
    public DcMotor  left_back_drive    = null;
    public DcMotor  right_back_drive   = null;

    //Define other subsystems/motors
    public HangArm hangArm;
    public ExtenderArm extenderArm;
    public RotateArm rotateArm;
    public Collector collector;

    //Define Servos

    //Define Sensors
    public BNO055IMU adafruitIMU;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;


    /* Constructor */
    public CombinedHardware()
    {
    }

    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        left_front_drive = ahwMap.dcMotor.get("left_front_drive");
        right_front_drive = ahwMap.dcMotor.get("right_front_drive");
        left_back_drive = ahwMap.dcMotor.get("left_back_drive");
        right_back_drive = ahwMap.dcMotor.get("right_back_drive");

        left_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        left_back_drive.setDirection(DcMotor.Direction.FORWARD);
        right_back_drive.setDirection(DcMotor.Direction.FORWARD);

        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hangArm = new HangArm("hang_arm",/*"upper_touch",*/ ahwMap);
        extenderArm = new ExtenderArm("extend_arm", ahwMap);
        rotateArm = new RotateArm("rotate_arm", ahwMap);
        collector = new Collector("collector", ahwMap);

        //Sensor Setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        adafruitIMU = ahwMap.get(BNO055IMU.class, "imu 1");
        adafruitIMU.initialize(parameters);
    }

    private ElapsedTime period  = new ElapsedTime();

    /*Value definitions to be used for distance conversion*/
    static final double     COUNTS_PER_MOTOR_REV    = 653 ;    // eg: Rev Hex Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 6.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH_LandR   = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1416);
    static final double     COUNTS_PER_INCH_FandB   = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1416) /(2);

    //Used to power the right side of the drive for the Gyro Auto Driver
    public void rightDrivePower(double power){
        left_front_drive.setPower(power);
        right_front_drive.setPower(power);
    }
    //Used to power the left side of the drivetrain for the Gyro Auto Driver
    public void leftDrivePower(double power){
        left_back_drive.setPower(power);
        right_back_drive.setPower(power);
    }
    //Used to set power for both sides of the drivetrain at once
    public void drivePowers(double leftSpeed, double rightSpeed){
        leftDrivePower(leftSpeed);
        rightDrivePower(rightSpeed);
    }

    //Set Translate Powers
    public void powerTranslate(double power){
        left_front_drive.setPower(power);
        left_back_drive.setPower(power);
        right_back_drive.setPower(-power);
        right_front_drive.setPower(-power);
    }


    //Stops the DriveTrain
    public void stopDrive(){
        drivePowers(0.0, 0.0);
    }


    /*Functions used in current autonomous programs*/
    public void driveLeft (LinearOpMode opMode, double inches, double speed, double timeoutS)
    {
        driveRight(opMode, -inches, speed, timeoutS);
    }
    public void driveRight (LinearOpMode opMode, double inches, double speed, double timeoutS)
    {
        int newLeftFrontTarget  = left_front_drive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH_LandR);
        int newRightFrontTarget = right_front_drive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH_FandB);
        left_front_drive.setTargetPosition(newLeftFrontTarget);
        right_front_drive.setTargetPosition(newRightFrontTarget);
        int newLeftBackTarget  = left_back_drive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH_LandR);
        int newRightBackTarget = right_back_drive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH_FandB);
        left_back_drive.setTargetPosition(newLeftBackTarget);
        right_back_drive.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion
        ElapsedTime     runtime = new ElapsedTime();
        runtime.reset();
        left_front_drive.setPower(Math.abs(speed));
        right_front_drive.setPower(Math.abs(speed));
        left_back_drive.setPower(Math.abs(speed));
        right_back_drive.setPower(Math.abs(speed));

        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (left_front_drive.isBusy() && right_front_drive.isBusy()))
        {
        }

        // Stop all motion;
        left_front_drive.setPower(0);
        right_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_back_drive.setPower(0);

        // Turn off RUN_TO_POSITION
        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void translateBackward(LinearOpMode opMode, double inches, double speed, double timeoutS)
    {
        translateForward(opMode, -inches, speed, timeoutS);
    }

    public void translateForward(LinearOpMode opMode, double inches, double speed, double timeoutS)
    {
        int newLeftFrontTarget  = left_front_drive.getCurrentPosition() + (int)(-inches * COUNTS_PER_INCH_LandR);
        int newRightFrontTarget = right_front_drive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH_FandB);
        left_front_drive.setTargetPosition(newLeftFrontTarget);
        right_front_drive.setTargetPosition(newRightFrontTarget);
        int newLeftBackTarget  = left_back_drive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH_LandR);
        int newRightBackTarget = right_back_drive.getCurrentPosition() + (int)(-inches * COUNTS_PER_INCH_FandB);
        left_back_drive.setTargetPosition(newLeftBackTarget);
        right_back_drive.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion
        ElapsedTime     runtime = new ElapsedTime();
        runtime.reset();
        left_front_drive.setPower(Math.abs(speed));
        right_front_drive.setPower(Math.abs(speed));
        left_back_drive.setPower(Math.abs(speed));
        right_back_drive.setPower(Math.abs(speed));

        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (left_front_drive.isBusy() && right_front_drive.isBusy()))
        {
        }

        // Stop all motion;
        stopDrive();

        // Turn off RUN_TO_POSITION
        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnLeft(LinearOpMode opMode, double inches, double speed, double timeoutS){

    }

    public void turnRight(){}

    //Sleep while catching InteruptedException
    public void pause(long millis )
    {
        try {
            Thread.sleep(millis );
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
