package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Blue_Crater_BC_Fast", group="Autonomous")
//@Disabled

public class Blue_Crater_BC_Fast extends LinearOpMode {
    public void runOpMode() {


        /* Declare OpMode members. */
        Hardware robot       = new Hardware(); // use the class created to define a Pushbot's hardware
        // could also use HardwarePushbotMatrix class.

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        Michaels_tensor_flow tensor_flow = new Michaels_tensor_flow();
        tensor_flow.init(this );
        waitForStart();

        //Lower robot
        //12500 Encoder ticks lower down the robot until all wheels touch the ground and the latch is disengaged
        robot.hang_arm.setPower(-0.75);
        while (robor.hang_arm.getCurrentPosition() <= 12500);
        {

        }

        //Older code
//        robot.hang_arm.setPower(-0.75);
//        robot.pause(1000);
//        robot.hang_arm.setPower(0.0);


        //Move robot away from lander latch

        robot.driveRight(this, 3, 0.25, 3);

        //Drive right to clear lander

        tensor_flow.activate();
        //FIND GOLD
        Michaels_tensor_flow.goldfinder goldPosition = tensor_flow.getGoldPosition(telemetry);

        //Robot chooses which path to take, Left, Right, Middle, or Unknown

        switch (goldPosition)
        {
            case LEFT:
                //drive 2" from minerals
                robot.driveRight(this, 30, 0.50, 5);
                //drive to left block
                robot.driveForward(this, 14, 0.50, 5);
                //push left block
                robot.driveRight(this, 4, 0.50, 3);
                //turn robot to the left to face the wall
                    //robot.turnLeft(90*);
                //drive toward the wall
                robot.driveForward(this, 17, 0.30, 5);
                //drive towards wall
                robot.driveForward(this, 33.9, 0.50, 5);
                //turn to be parallel with wall
                    //robot.turnRight(45*);
                //drive to depot
                robot.driveLeft(this, 60, 0.50, 10);
                //deposit marker
                    //robot.markerServo.setPosition(x);
                //drive to crater
                robot.driveRight(this, 74, 0.75, 10);
                break;

            case RIGHT:
                //drive 2" from minerals
                robot.driveRight(this, 30, 0.50, 5);
                //drive to right block
                robot.driveBackward(this, 11, 0.50, 5);
                //push right block
                robot.driveRight(this, 4, 0.50, 3);
                //back up
                robot.driveLeft(this, 6, 0.50, 3);
                //turn robot left to face the wall
                    //robot.turnLeft(90*);
                //drive toward the wall
                robot.driveForward(this, 100, 0.75, 10);
                //turn to be parallel with the wall
                    //robot.turnLeft(45*);
                //drive to depot
                robot.driveLeft(this, 36, 0.50, 5);
                //drive to crater
                robot.driveRight( this, 74, 0.75, 10);
                break;

            case MIDDLE:
                //drive 2" from minerals (30") then push block (4")
                robot.driveRight(this, 34, 0.50, 10);
                //back up
                robot.driveLeft(this, 6, 0.50, 3);
                //drive toward the wall
                robot.driveForward(this, 67, 0.75, 10);
                //turn to be parallel with the wall
                    //robot.turnRight(45*);
                //drive to depot
                robot.driveLeft( this 36, 0.50, 5);
                //drive to crater
                robot.driveRight(this, 74, 0.75, 10);
                break;

            case UNKNOWN:
            default: //like unknown
                //drive strait and hope the block is there
                break;


        }

        tensor_flow.deactivate();

        telemetry.addData("Path0", "Starting at %7d : %7d : %7d : %7d",
                robot.left_front_drive.getCurrentPosition(),
                robot.right_front_drive.getCurrentPosition(),
                robot.left_back_drive.getCurrentPosition(),
                robot.right_back_drive.getCurrentPosition());
        telemetry.update();

        //lower down
        robot.driveRight(this, 36, 0.25, 5);
        telemetry.addData("Path0", "Starting at %7d : %7d : %7d : %7d",
                robot.left_front_drive.getCurrentPosition(),
                robot.right_front_drive.getCurrentPosition(),
                robot.left_back_drive.getCurrentPosition(),
                robot.right_back_drive.getCurrentPosition());
        telemetry.update();
        robot.pause(15000);

        robot.driveLeft(this, 36, 0.25, 5);
        telemetry.addData("Path0", "Starting at %7d : %7d : %7d : %7d",
                robot.left_front_drive.getCurrentPosition(),
                robot.right_front_drive.getCurrentPosition(),
                robot.left_back_drive.getCurrentPosition(),
                robot.right_back_drive.getCurrentPosition());
        telemetry.update();
        robot.pause(15000);

        robot.driveForward(this, 36, 0.25, 5);
        telemetry.addData("Path0", "Starting at %7d : %7d : %7d : %7d",
                robot.left_front_drive.getCurrentPosition(),
                robot.right_front_drive.getCurrentPosition(),
                robot.left_back_drive.getCurrentPosition(),
                robot.right_back_drive.getCurrentPosition());
        telemetry.update();
        robot.pause(15000);

        robot.driveBackward(this, 36, 0.25, 5);
        telemetry.addData("Path0", "Starting at %7d : %7d : %7d : %7d",
                robot.left_front_drive.getCurrentPosition(),
                robot.right_front_drive.getCurrentPosition(),
                robot.left_back_drive.getCurrentPosition(),
                robot.right_back_drive.getCurrentPosition());
        telemetry.update();
        robot.pause(15000);

        //robot.driveForward(this, 10, 0.5, 5);

        //robot.driveBackward(this, 10, 0.5, 5);

        //delatch
        //scan for the minearals
        //move gold minearal
        //move to depot
        //deposit marker
        //drive to crater
    }
}