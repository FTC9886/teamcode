package org.firstinspires.ftc.teamcode.Extensions;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.ExtenderArm;
import org.firstinspires.ftc.teamcode.Subsystems.HangArm;
import org.firstinspires.ftc.teamcode.Subsystems.RotateArm;

public class AutoHardware {
    LinearOpMode opMode;

    //Define Drivetrain
    public DcMotor left_front_drive   = null;
    public DcMotor  right_front_drive  = null;
    public DcMotor  left_back_drive    = null;
    public DcMotor  right_back_drive   = null;

    //Define other subsystems/motors
    public HangArm hangArm;
    public ExtenderArm extenderArm;
    public RotateArm rotateArm;

    //Define Servos

    //Define Sensors
    public BNO055IMU adafruitIMU;



    AutoHardware(LinearOpMode opMode){
        this.opMode = opMode;

        DcMotor left_front_drive = opMode.hardwareMap.dcMotor.get("left_front_drive");
        DcMotor right_front_drive = opMode.hardwareMap.dcMotor.get("right_front_drive");
        DcMotor left_back_drive = opMode.hardwareMap.dcMotor.get("left_back_drive");
        DcMotor right_back_drive = opMode.hardwareMap.dcMotor.get("right_back_drive");

        left_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        left_back_drive.setDirection(DcMotor.Direction.FORWARD);
        right_back_drive.setDirection(DcMotor.Direction.FORWARD);

        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hangArm = new HangArm("hang_arm", opMode.hardwareMap);
        extenderArm = new ExtenderArm("extend_arm", opMode.hardwareMap);
        rotateArm = new RotateArm("rotate_arm", opMode.hardwareMap);

        //Sensor Setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        adafruitIMU = opMode.hardwareMap.get(BNO055IMU.class, "imu 1");
        adafruitIMU.initialize(parameters);




    }


    public void rightDrivePower(double power){
        left_front_drive.setPower(power);
        right_front_drive.setPower(power);
    }

    public void leftDrivePower(double power){
        left_back_drive.setPower(power);
        right_back_drive.setPower(power);
    }

    public void drivePowers(double leftSpeed, double rightSpeed){
        leftDrivePower(leftSpeed);
        rightDrivePower(rightSpeed);
    }

    public void stopDrive(){
        drivePowers(0.0, 0.0);
    }

    protected long inchesToEncoderCounts(double distance){
        return Math.round(((distance / (Math.PI * 6.0)) / 1.1) * 1120);
    }


    void init(){

    }
}
