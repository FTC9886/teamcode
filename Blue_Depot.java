package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.CameraDevice;

@Autonomous(name="Blue_Depot", group="Autonomous")
//@Disabled

public class Blue_Depot extends LinearOpMode {
    public void runOpMode() {


        /* Declare OpMode members. */
        Hardware robot = new Hardware(); // use the class created to define a Pushbot's hardware
        // could also use HardwarePushbotMatrix class.

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        Michaels_tensor_flow tensor_flow = new Michaels_tensor_flow();
        tensor_flow.init(this);
        waitForStart();
        //change to use a more robust function later

        robot.hang_arm.setTargetPosition(12634);
        robot.hang_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hang_arm.setPower(0.75);
        while (robot.hang_arm.isBusy())
        {
        }
        robot.hang_arm.setPower(0);
        robot.pause(4000);
        //Move robot away from lander latch

        robot.driveBackward(this, 5, 0.5, 3);
        robot.pause(2000);
        //Drive right to clear lander

        CameraDevice.getInstance().setFlashTorchMode(true);

        tensor_flow.activate();
        robot.pause(10000);
        //FIND GOLD
        Michaels_tensor_flow.goldfinder goldPosition = tensor_flow.getGoldPosition(telemetry);
        robot.pause(2000);
        CameraDevice.getInstance().setFlashTorchMode(false);
        robot.pause(10000);

        tensor_flow.deactivate();

        telemetry.addData("Path0", "Starting at %7d : %7d : %7d : %7d",
                robot.left_front_drive.getCurrentPosition(),
                robot.right_front_drive.getCurrentPosition(),
                robot.left_back_drive.getCurrentPosition(),
                robot.right_back_drive.getCurrentPosition());
        telemetry.update();
        robot.pause(10000);

        robot.driveRight(this, 17.5, 0.3, 5);
        telemetry.addData("Path0", "Starting at %7d : %7d : %7d : %7d",
                robot.left_front_drive.getCurrentPosition(),
                robot.right_front_drive.getCurrentPosition(),
                robot.left_back_drive.getCurrentPosition(),
                robot.right_back_drive.getCurrentPosition());
        telemetry.update();
        robot.pause(10000);

        robot.driveForward(this, 20, 0.3, 5);
        telemetry.addData("Path0", "Starting at %7d : %7d : %7d : %7d",
                robot.left_front_drive.getCurrentPosition(),
                robot.right_front_drive.getCurrentPosition(),
                robot.left_back_drive.getCurrentPosition(),
                robot.right_back_drive.getCurrentPosition());
        telemetry.update();
        robot.pause(10000);

        robot.driveRight(this, 6, 0.3, 2);
        telemetry.addData("Path0", "Starting at %7d : %7d : %7d : %7d",
                robot.left_front_drive.getCurrentPosition(),
                robot.right_front_drive.getCurrentPosition(),
                robot.left_back_drive.getCurrentPosition(),
                robot.right_back_drive.getCurrentPosition());
        telemetry.update();
        robot.pause(10000);

        robot.driveBackward(this, 39, 0.3, 10);
        telemetry.addData("Path0", "Starting at %7d : %7d : %7d : %7d",
                robot.left_front_drive.getCurrentPosition(),
                robot.right_front_drive.getCurrentPosition(),
                robot.left_back_drive.getCurrentPosition(),
                robot.right_back_drive.getCurrentPosition());
        telemetry.update();
        robot.pause(10000);

        robot.driveForward(this, 74, 0.3, 15);
        telemetry.addData("Path0", "Starting at %7d : %7d : %7d : %7d",
                robot.left_front_drive.getCurrentPosition(),
                robot.right_front_drive.getCurrentPosition(),
                robot.left_back_drive.getCurrentPosition(),
                robot.right_back_drive.getCurrentPosition());
        telemetry.update();
        robot.pause(10000);

        robot.extend_arm.setPower(0.5);
        robot.pause(2000);
        robot.extend_arm.setPower(0);
    }
}