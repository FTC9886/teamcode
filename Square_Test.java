package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Square_Test", group="Autonomous")
//@Disabled

public class Square_Test extends LinearOpMode {
    public void runOpMode() {


        /* Declare OpMode members. */
        Hardware robot = new Hardware(); // use the class created to define a Pushbot's hardware
        // could also use HardwarePushbotMatrix class.

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        waitForStart();

        robot.driveBackward(this, 24, 0.3, 5);
                telemetry.addData("Path0", "Starting at %7d : %7d : %7d : %7d",
                robot.left_front_drive.getCurrentPosition(),
                robot.right_front_drive.getCurrentPosition(),
                robot.left_back_drive.getCurrentPosition(),
                robot.right_back_drive.getCurrentPosition());
        telemetry.update();
        robot.pause(10000);

        robot.driveRight(this, 24, 0.3, 5);
        telemetry.addData("Path0", "Starting at %7d : %7d : %7d : %7d",
                robot.left_front_drive.getCurrentPosition(),
                robot.right_front_drive.getCurrentPosition(),
                robot.left_back_drive.getCurrentPosition(),
                robot.right_back_drive.getCurrentPosition());
        telemetry.update();
        robot.pause(10000);

        robot.driveForward(this, 24, 0.3, 5);
        telemetry.addData("Path0", "Starting at %7d : %7d : %7d : %7d",
                robot.left_front_drive.getCurrentPosition(),
                robot.right_front_drive.getCurrentPosition(),
                robot.left_back_drive.getCurrentPosition(),
                robot.right_back_drive.getCurrentPosition());
        telemetry.update();
        robot.pause(10000);

        robot.driveLeft(this, 24, 0.3, 5);
        telemetry.addData("Path0", "Starting at %7d : %7d : %7d : %7d",
                robot.left_front_drive.getCurrentPosition(),
                robot.right_front_drive.getCurrentPosition(),
                robot.left_back_drive.getCurrentPosition(),
                robot.right_back_drive.getCurrentPosition());
        telemetry.update();
        robot.pause(10000);
    }
}