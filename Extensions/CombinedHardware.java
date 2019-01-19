package org.firstinspires.ftc.teamcode.Extensions;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Collector;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.ExtenderArm;
import org.firstinspires.ftc.teamcode.Extensions.GyroAutoDriver;
import org.firstinspires.ftc.teamcode.Subsystems.HangArm;
import org.firstinspires.ftc.teamcode.Subsystems.MarkerDeploy;
import org.firstinspires.ftc.teamcode.Subsystems.RotateArm;

public class CombinedHardware {



    //Define other subsystems/motors
    public DriveTrain driveTrain;
    public HangArm hangArm;
    public ExtenderArm extenderArm;
    public RotateArm rotateArm;
    public Collector collector;
    public MarkerDeploy markerDeploy;
    public AnalogInput potentiometer   = null;

    public GyroAutoDriver gyroAutoDriver;

    //Define Servos

    //Define Sensors
    public BNO055IMU adafruitIMU;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;


    /* Constructor */
    public CombinedHardware()
    {
    }

    //Function that creates new objects for each subsystem and creates a hardware map. Used in the init sections of OpModes
    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        driveTrain = new DriveTrain("left_front_drive","left_back_drive","right_front_drive","right_back_drive", ahwMap);
        hangArm = new HangArm("hang_arm",/*"upper_touch",*/ ahwMap);
        extenderArm = new ExtenderArm("extend_arm", ahwMap);
        rotateArm = new RotateArm("rotate_arm", ahwMap);
        collector = new Collector("collector", ahwMap);
        markerDeploy = new MarkerDeploy("marker_servo", ahwMap);
        potentiometer = ahwMap.analogInput.get("arm_angle");

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


        gyroAutoDriver = new GyroAutoDriver(this);
    }




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
