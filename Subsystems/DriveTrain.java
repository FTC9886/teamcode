package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveTrain {
    //Define Drivetrain
    public DcMotor  left_front_drive   = null;
    public DcMotor  right_front_drive  = null;
    public DcMotor  left_back_drive    = null;
    public DcMotor  right_back_drive   = null;

    private enum ExtenderArmEnum{
        EXTENDING,
        RETRACTING,
        STOPPED
    }

    private ExtenderArmEnum extenderArmState;

    public DriveTrain(String leftFront, String leftBack,String rightFront, String rightBack, HardwareMap hardwareMap){
        this.left_front_drive = hardwareMap.dcMotor.get(leftFront);
        this.left_back_drive = hardwareMap.dcMotor.get(leftBack);
        this.right_front_drive = hardwareMap.dcMotor.get(rightFront);
        this.right_back_drive = hardwareMap.dcMotor.get(rightBack);

        left_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        left_back_drive.setDirection(DcMotor.Direction.REVERSE);
        right_back_drive.setDirection(DcMotor.Direction.REVERSE);

        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stopDrive();
    }

    public void teleTranslate(double power){
        left_front_drive.setPower (power);
        right_front_drive.setPower(power);
        left_back_drive.setPower  (power);
        right_back_drive.setPower (power);
    }





    public void resetEncoders(){
        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private ElapsedTime period  = new ElapsedTime();

    /*Value definitions to be used for distance conversion*/
    static final double     COUNTS_PER_MOTOR_REV    = 1106 ;    // eg: Rev Hex Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 6.0 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH_LandR   = (COUNTS_PER_MOTOR_REV * 1.042 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1416);
    public static final double     COUNTS_PER_INCH_FandB   = (COUNTS_PER_MOTOR_REV * 0.958 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1416);

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

    public void stickPower(double leftPower, double rightPower){
        left_front_drive.setPower (leftPower);
        right_back_drive.setPower (-rightPower);
        right_front_drive.setPower(rightPower);
        left_back_drive.setPower  (-leftPower);
    }


    public void forwards(double power){
        left_front_drive.setPower (-power);
        right_back_drive.setPower (-power);
        right_front_drive.setPower(power);
        left_back_drive.setPower  (power);

    }

    public void backwards(double power){
        left_front_drive.setPower (power);
        right_back_drive.setPower (power);
        right_front_drive.setPower(-power);
        left_back_drive.setPower  (-power);
    }

    public void left(double power){
        left_front_drive.setPower (power);
        right_back_drive.setPower (power);
        right_front_drive.setPower(power);
        left_back_drive.setPower  (power);
    }

    public void right(double power){
        left_front_drive.setPower (-power);
        right_back_drive.setPower (-power);
        right_front_drive.setPower(-power);
        left_back_drive.setPower  (-power);

    }


    //Stops the DriveTrain
    public void stopDrive(){
        drivePowers(0.0, 0.0);
    }


    /*Functions used in current autonomous programs*/
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



    protected long inchesToEncoderCounts(double distance){
        return Math.round(((distance / (Math.PI * WHEEL_DIAMETER_INCHES)) / DRIVE_GEAR_REDUCTION) * COUNTS_PER_MOTOR_REV);
    }
    @Override
    public String toString(){
        switch (extenderArmState){
            case EXTENDING:
                return "Extending";
            case RETRACTING:
                return "Retracting";
            case STOPPED:
                return "Stopped";
            default:
                return "Unknown";
        }
    }
}
