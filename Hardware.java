package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
*/

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *  NEED TO UPDATE THE FOLLOWING NOTES ....
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Hardware
{
    /* Public OpMode members. */
    public DcMotor  left_front_drive   = null;
    public DcMotor  right_front_drive  = null;
    public DcMotor  left_back_drive    = null;
    public DcMotor  right_back_drive   = null;

    public DcMotor hang_arm;
    public DcMotor extend_arm;
    public DcMotor rotate_arm;
    public DcMotor collector;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1106 ;    // eg: Rev Hex Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 6.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH_LandR   = (COUNTS_PER_MOTOR_REV * 1.042 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1416);
    static final double     COUNTS_PER_INCH_FandB   = (COUNTS_PER_MOTOR_REV * 0.958 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1416);

    /* Constructor */
    public Hardware()
    {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        left_front_drive  = hwMap.dcMotor.get("left_front_drive");
        right_front_drive = hwMap.dcMotor.get("right_front_drive");
        left_back_drive   = hwMap.dcMotor.get("left_back_drive");
        right_back_drive  = hwMap.dcMotor.get("right_back_drive");

        hang_arm = hwMap.dcMotor.get("hang_arm");
        extend_arm = hwMap.dcMotor.get("extend_arm");
        rotate_arm = hwMap.dcMotor.get("rotate_arm");
        collector = hwMap.dcMotor.get("collector");

        left_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        left_back_drive.setDirection(DcMotor.Direction.REVERSE);
        right_back_drive.setDirection(DcMotor.Direction.REVERSE);

        hang_arm.setDirection(DcMotor.Direction.REVERSE);
        extend_arm.setDirection(DcMotor.Direction.FORWARD);
        rotate_arm.setDirection(DcMotor.Direction.FORWARD);
        collector.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        left_front_drive.setPower(0);
        right_front_drive.setPower(0);
        left_back_drive.setPower(0);
        right_back_drive.setPower(0);
        hang_arm.setPower(0);
        extend_arm.setPower(0);
        rotate_arm.setPower(0);
        collector.setPower(0);

        // Stop and reset all encoders
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run with / without encoders
        left_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang_arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
    public void driveRight (LinearOpMode opMode, double inches, double speed, double timeoutS)
    {
        driveLeft(opMode, -inches, speed, timeoutS);
    }
    public void driveLeft (LinearOpMode opMode, double inches, double speed, double timeoutS)
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

    public void driveBackward (LinearOpMode opMode, double inches, double speed, double timeoutS)
    {
        driveForward(opMode, -inches, speed, timeoutS);
    }

    public void driveForward (LinearOpMode opMode, double inches, double speed, double timeoutS)
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


    public void lowerRobotUsingTouchSensor () {

//        //TODO touch sensors for the min and max of the lowering arms travel (touch sensor raised, rouch sensor lowered)
//        //start lowering down
//        hang_arm.setPower(1.0);
//        //wait until contracted ouch sensor is pressed
//        do {
//            waitForTick(1);
//        }
//        while (!touchSensorContracted.ispressed());
//        //stop lowing the robot
//        hang_arm.setPower(0.0);



    }

    public void pause(long millis )
    {
        try {
            Thread.sleep(millis );
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}


