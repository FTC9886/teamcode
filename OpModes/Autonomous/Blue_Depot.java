package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.CameraDevice;

@Autonomous(name="Blue_Depot", group="Autonomous")
@Disabled

public class Blue_Depot extends LinearOpMode {
    public void runOpMode() {

        /* Declare OpMode members. */
        CombinedHardware robot = new CombinedHardware(); // use the class created to define a Pushbot's hardware
        // could also use HardwarePushbotMatrix class.

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        Michaels_tensor_flow tensor_flow = new Michaels_tensor_flow();
        tensor_flow.init(this);
        waitForStart();
        //change to use a more robust function later

/*        robot.hang_arm.setTargetPosition(12634);
        robot.hang_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hang_arm.setPower(0.75);
        while (robot.hang_arm.isBusy())
        {
        }
        robot.hang_arm.setPower(0);
        robot.pause(4000);


        //Move robot away from lander latch
*/
        while (!robot.hangArm.isFullyLowered() && opModeIsActive()) {
            robot.hangArm.lower();
            telemetry.addData("hang arm",robot.hangArm);
            telemetry.update();
        }
        robot.driveTrain.translateBackward(this, 5, 0.5, 3);
        robot.pause(500);
        //Drive right to clear lander

        CameraDevice.getInstance().setFlashTorchMode(true);

        tensor_flow.activate();
        robot.pause(1000);
        //Find the gold mineral
        Michaels_tensor_flow.goldfinder goldPosition = tensor_flow.getGoldPosition(telemetry);
        robot.pause(500);
        CameraDevice.getInstance().setFlashTorchMode(false);
        tensor_flow.deactivate();

        robot.pause(1000);

        telemetry.addData("Path01", "Starting at %7d : %7d : %7d : %7d",
                robot.driveTrain.left_front_drive.getCurrentPosition(),
                robot.driveTrain.right_front_drive.getCurrentPosition(),
                robot.driveTrain.left_back_drive.getCurrentPosition(),
                robot.driveTrain.right_back_drive.getCurrentPosition());
        telemetry.update();
        robot.pause(1000);

        //Drive toward the minerals
        robot.driveTrain.driveRight(this, 16.5, 0.3, 5);
        telemetry.addData("Path02", "Starting at %7d : %7d : %7d : %7d",
                robot.driveTrain.left_front_drive.getCurrentPosition(),
                robot.driveTrain.right_front_drive.getCurrentPosition(),
                robot.driveTrain.left_back_drive.getCurrentPosition(),
                robot.driveTrain.right_back_drive.getCurrentPosition());
        telemetry.update();
        robot.pause(10000);

        //Drive past the minerals
        robot.driveTrain.translateForward(this, 20, 0.3, 5);
        telemetry.addData("Path03", "Starting at %7d : %7d : %7d : %7d",
                robot.driveTrain.left_front_drive.getCurrentPosition(),
                robot.driveTrain.right_front_drive.getCurrentPosition(),
                robot.driveTrain.left_back_drive.getCurrentPosition(),
                robot.driveTrain.right_back_drive.getCurrentPosition());
        telemetry.update();
        robot.pause(10000);

        //Need to add a 45 deg CCW turn here #######
            robot.gyroAutoDriver.turn(45, 0.3, 5);
        //Drive toward and align against the wall
        robot.driveTrain.driveRight(this, 6, 0.3, 2);
        telemetry.addData("Path04", "Starting at %7d : %7d : %7d : %7d",
                robot.driveTrain.left_front_drive.getCurrentPosition(),
                robot.driveTrain.right_front_drive.getCurrentPosition(),
                robot.driveTrain.left_back_drive.getCurrentPosition(),
                robot.driveTrain.right_back_drive.getCurrentPosition());
        telemetry.update();
        robot.pause(10000);

        //Drive to the depot
        robot.driveTrain.translateBackward(this, 39, 0.3, 10);
        telemetry.addData("Path05", "Starting at %7d : %7d : %7d : %7d",
                robot.driveTrain.left_front_drive.getCurrentPosition(),
                robot.driveTrain.right_front_drive.getCurrentPosition(),
                robot.driveTrain.left_back_drive.getCurrentPosition(),
                robot.driveTrain.right_back_drive.getCurrentPosition());
        telemetry.update();
        robot.pause(10000);

        //Release the team marker ########
        robot.markerDeploy.deploy();
        robot.pause(250);
        robot.markerDeploy.retract();

        //Drive toward the opposing crater
        robot.driveTrain.translateForward(this, 74, 0.3, 15);
        telemetry.addData("Path06", "Starting at %7d : %7d : %7d : %7d",
                robot.driveTrain.left_front_drive.getCurrentPosition(),
                robot.driveTrain.right_front_drive.getCurrentPosition(),
                robot.driveTrain.left_back_drive.getCurrentPosition(),
                robot.driveTrain.right_back_drive.getCurrentPosition());
        telemetry.update();
        robot.pause(10000);

        //Extend the arm for the ending position
        robot.extenderArm.extend();
        robot.pause(2000);
        robot.extenderArm.stop();
    }
}