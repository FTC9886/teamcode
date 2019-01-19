package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//Subsystem class. Defines motors/servos/sensors used only by the subsystem, an enumeration outlining the states the subsystem can be in, and functions that control those states

public class DriveTrain {
    //Define Drivetrain motors
    public DcMotor  left_front_drive;
    public DcMotor  right_front_drive;
    public DcMotor  left_back_drive;
    public DcMotor  right_back_drive;

    //Creates an enumeration with values corresponding to the states the drive train can be in.
    //Forwards is considered to the be side of the robot that the Collector faces.
    private enum DriveTrainEnum{
        FORWARDS,
        BACKWARDS,
        LEFT,
        RIGHT,
        TURNING,
        STOPPED
    }

    //Creates a new DriveTrainEnum
    private DriveTrainEnum driveTrainState;

    //Defines the DriveTrain constructor. It is called in the CombinedHardware class/extension where it joins the other subsystems in the robot object and hardware map.
    //Sets default state of the DriveTrain
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

    //Function that takes a power value and applies it to the drive motors. Used in TeleOp to drive left and right using triggers on a controller.
    //Considers front of the robot to be side that the Collector is facing
    public void teleTranslate(double power){
        left_front_drive.setPower (power);
        right_front_drive.setPower(power);
        left_back_drive.setPower  (power);
        right_back_drive.setPower (power);
    }




//Function that resets the motor encoders. Used during autonomous to ensure more consistent results when navigating.
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

    //Defines new ElapsedTime object
    private ElapsedTime period  = new ElapsedTime();

    /*Value definitions to be used for distance conversion*/
    static final double     COUNTS_PER_MOTOR_REV    = 1106 ;    // eg: Rev Hex Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 6.0 ;     // For figuring circumference
    //Double that represents the number of encoder counts per inch of movement for the robot
    public static final double     COUNTS_PER_INCH_LandR   = (COUNTS_PER_MOTOR_REV * 1.042 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1416);
    public static final double     COUNTS_PER_INCH_FandB   = (COUNTS_PER_MOTOR_REV * 0.958 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1416);

    //Function to set power for the right side of the drive for the GyroAutoDriver.
    // Considers front of the robot to be side facing direction the robot moves when positive power is passed to drive train motors
    public void rightDrivePower(double power){
        left_front_drive.setPower(power);
        right_front_drive.setPower(power);
    }

    //Function to set power for the left side of the drivetrain for the GyroAutoDriver.
    // Considers front of the robot to be side facing direction the robot moves when positive power is passed to drive train motors
    public void leftDrivePower(double power){
        left_back_drive.setPower(power);
        right_back_drive.setPower(power);
    }

    //Function to set power for both sides of the drivetrain at once for the GyroAutoDriver
    // Considers front of the robot to be side facing direction the robot moves when positive power is passed to drive train motors
    public void drivePowers(double leftSpeed, double rightSpeed){
        leftDrivePower(leftSpeed);
        rightDrivePower(rightSpeed);
        if (leftSpeed > 0 && rightSpeed > 0){
            driveTrainState = DriveTrainEnum.LEFT;
        } else if(leftSpeed <0 && rightSpeed<0){
            driveTrainState = DriveTrainEnum.RIGHT;
        } else if(leftSpeed == 0 && rightSpeed == 0){
            driveTrainState = DriveTrainEnum.STOPPED;
        }else {
            driveTrainState = DriveTrainEnum.TURNING;
        }
    }

    //Function used to set DriveTrain power using joysticks. Intended to be used for tank drive.
    // Considers front of the robot to be side of the robot that the Collector faces.
    public void stickPower(double leftPower, double rightPower){
        left_front_drive.setPower (leftPower);
        right_back_drive.setPower (-rightPower);
        right_front_drive.setPower(rightPower);
        left_back_drive.setPower  (-leftPower);
        if (leftPower >0 && rightPower>0){
            driveTrainState = DriveTrainEnum.FORWARDS;
        } else if (leftPower < 0 && rightPower < 0){
            driveTrainState = DriveTrainEnum.BACKWARDS;
        } else if(leftPower == 0 && rightPower == 0){
            driveTrainState = DriveTrainEnum.STOPPED;
        }else {
            driveTrainState = DriveTrainEnum.TURNING;
        }
    }

    //Function that takes a power value and drives the robot forwards. Intended to be used in TeleOp in conjunction with a button or D-pad directoin
    //Considers front of the robot to be side of the robot that the Collector faces.
    public void forwards(double power){
        left_front_drive.setPower (-power);
        right_back_drive.setPower (-power);
        right_front_drive.setPower(power);
        left_back_drive.setPower  (power);
        driveTrainState = DriveTrainEnum.FORWARDS;
    }

    //Function that takes a power value and drives the robot backwards. Intended to be used in TeleOp in conjunction with a button or D-pad directoin
    //Considers front of the robot to be side of the robot that the Collector faces.
    public void backwards(double power){
        left_front_drive.setPower (power);
        right_back_drive.setPower (power);
        right_front_drive.setPower(-power);
        left_back_drive.setPower  (-power);
        driveTrainState = DriveTrainEnum.BACKWARDS;
    }

    //Function that takes a power value and drives the robot to the left. Intended to be used in TeleOp in conjunction with a button or D-pad directoin
    //Considers front of the robot to be side of the robot that the Collector faces.
    public void left(double power){
        left_front_drive.setPower (power);
        right_back_drive.setPower (power);
        right_front_drive.setPower(power);
        left_back_drive.setPower  (power);
        driveTrainState = DriveTrainEnum.LEFT;
    }

    //Function that takes a power value and drives the robot to the right. Intended to be used in TeleOp in conjunction with a button or D-pad directoin
    //Considers front of the robot to be side of the robot that the Collector faces.
    public void right(double power){
        left_front_drive.setPower (-power);
        right_back_drive.setPower (-power);
        right_front_drive.setPower(-power);
        left_back_drive.setPower  (-power);
        driveTrainState = DriveTrainEnum.RIGHT;
    }


    //Function that stops the DriveTrain by calling the drivePowers function and passing it 0 speed
    public void stopDrive(){
        drivePowers(0.0, 0.0);
        driveTrainState = DriveTrainEnum.STOPPED;
    }


    /*Functions used in current autonomous programs*/

    //Function that takes a distance in inches, a speed, and a max runtime in seconds and drives the robot to the left. Intended to be used in Autonomous for navigation
    //Considers front of the robot to be side of the robot that the Collector faces.
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
            if (inches > 0){
                driveTrainState = DriveTrainEnum.LEFT;
            } else if (inches <0){
                driveTrainState = DriveTrainEnum.RIGHT;
            }
        }

        // Stop all motion;
        stopDrive();

        // Turn off RUN_TO_POSITION
        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //Function that takes a distance in inches, a speed, and a max runtime in seconds and drives the robot to the Right. Intended to be used in Autonomous for navigation
    //Calls the driveLeft function and passes it reverse distance
    //Considers front of the robot to be side of the robot that the Collector faces.
    public void driveRight (LinearOpMode opMode, double inches, double speed, double timeoutS)
    {
        driveLeft(opMode, -inches, speed, timeoutS);
    }

    //Function that takes a distance in inches, a speed, and a max runtime in seconds and drives the robot forwards. Intended to be used in Autonomous for navigation
    //Considers front of the robot to be side of the robot that the Collector faces.
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
            if(inches >0){
                driveTrainState = DriveTrainEnum.FORWARDS;
            } else if(inches <0){
                driveTrainState = DriveTrainEnum.BACKWARDS;
            }
        }

        // Stop all motion;
        stopDrive();

        // Turn off RUN_TO_POSITION
        left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Function that takes a distance in inches, a speed, and a max runtime in seconds and drives the robot forwards. Intended to be used in Autonomous for navigation
    //Calls the translateForward function and passes it reverse inches
    //Considers front of the robot to be side of the robot that the Collector faces.
    public void translateBackward(LinearOpMode opMode, double inches, double speed, double timeoutS)
    {
        translateForward(opMode, -inches, speed, timeoutS);
    }

    //Alternative formula for converting inches into encoder counts for the drivetrain
    protected long inchesToEncoderCounts(double distance){
        return Math.round(((distance / (Math.PI * WHEEL_DIAMETER_INCHES)) / DRIVE_GEAR_REDUCTION) * COUNTS_PER_MOTOR_REV);
    }

    @Override
    public String toString(){
        switch (driveTrainState){
            case FORWARDS:
                return "Driving Forwards";
            case BACKWARDS:
                return "Driving Backwards";
            case LEFT:
                return "Driving Left";
            case RIGHT:
                return "Driving Right";
            case TURNING:
                return "Turning";
            case STOPPED:
                return "Stopped";
            default:
                return "Unknown";
        }
    }
}
