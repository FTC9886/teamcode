package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.Extensions.CombinedHardware;
import org.firstinspires.ftc.teamcode.Extensions.Michaels_tensor_flow;

@Autonomous(name="CraterAuto", group="Autonomous")
//@Disabled

public class CraterAuto extends LinearOpMode {
    //Declares new objects that will be used in the OpMode
    CombinedHardware robot = new CombinedHardware();
    Michaels_tensor_flow tensor_flow = new Michaels_tensor_flow();
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //initializes the robot hardware map
        robot.init(hardwareMap);

        //Defines double that sets the most commonly used speed for navigation, can be changed to easily change speed of autonomous program
        double defaultSpeed = 0.4;

        //Initializes tensorflow
        tensor_flow.init(this);
        //Turns camera light on
        CameraDevice.getInstance().setFlashTorchMode(true);

        //subs for waitForStart()
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }


        while (opModeIsActive() && !isStopRequested()) {

            //change to use a more robust function later
            while (!robot.hangArm.isFullyLowered()) {
                robot.hangArm.lower();
            }
            robot.hangArm.stop();
            robot.pause(1000);

            //Move robot away from lander latch
            robot.driveTrain.translateBackward(this, 4, 0.3, 3);
            robot.pause(500);

            tensor_flow.activate();
            //FIND GOLD
            Michaels_tensor_flow.goldfinder goldPosition = Michaels_tensor_flow.goldfinder.UNKNOWN;
            timer.reset();
            while (goldPosition == Michaels_tensor_flow.goldfinder.UNKNOWN && timer.seconds() < 5) {
                goldPosition = tensor_flow.getGoldPosition(telemetry);
            }
            telemetry.addData("Position", goldPosition);
            telemetry.update();
            robot.pause(1000);
            tensor_flow.deactivate();

            //Robot chooses which path to take, Left, Right, Middle, or Unknown

            switch (goldPosition) {
                case LEFT:
                    //drive 2" from minerals
                    robot.driveTrain.driveRight(this, 14, defaultSpeed, 3);
                    robot.pause(250);
                    //drive to left block
                    robot.driveTrain.translateForward(this, 21, defaultSpeed, 5);
                    robot.pause(250);
                    //push left block
                    robot.driveTrain.driveRight(this, 10, defaultSpeed, 5);
                    robot.pause(250);
                    //Pull back from block
                    robot.driveTrain.driveLeft(this, 9, defaultSpeed, 3);
                    //turn robot to the left to face the wall
                    robot.gyroAutoDriver.turn(125, 0.25, 5);
                    robot.pause(250);
                    //drive toward the wall
                    robot.driveTrain.translateBackward(this, 28, defaultSpeed, 5);
                    robot.pause(250);
                    //drive away from the wall
                    robot.driveTrain.translateForward(this, 2.5, 0.5, 5);
                    robot.pause(100);
                    //drive to depot
                    robot.gyroAutoDriver.driveForwards(110, 0.5);
                    robot.pause(250);
                    //deposit marker
                    robot.markerDeploy.deploy();
                    robot.pause(250);
                    //Release marker
                    robot.driveTrain.driveRight(this, 1, 0.2, 3);
                    robot.pause(250);
                    //retract marker servo
                    robot.markerDeploy.retract();
                    robot.pause(250);
                    //drive to crater
                    robot.gyroAutoDriver.driveBackwards(130, 0.5);
                    break;

                case RIGHT:
                    //drive 2" from minerals
                    robot.driveTrain.driveRight(this, 14, defaultSpeed, 3);
                    robot.pause(250);
                    //drive to right block
                    robot.driveTrain.translateBackward(this, 13.5, 0.2, 5);
                    robot.pause(250);
                    //push right block
                    robot.driveTrain.driveRight(this, 6, defaultSpeed, 5);
                    robot.pause(250);
                    //back up
                    robot.driveTrain.driveLeft(this, 5, defaultSpeed, 3);
                    robot.pause(250);
                    //drive toward the wall
                    robot.driveTrain.translateForward(this, 60, defaultSpeed, 10);
                    robot.pause(250);
                    //turn to be parallel with the wall
                    robot.gyroAutoDriver.turn(118, 0.25, 5);
                    robot.pause(250);
                    //drive toward the wall
                    robot.driveTrain.translateBackward(this, 16, defaultSpeed, 5);
                    robot.pause(250);
                    //drive away from the wall
                    robot.driveTrain.translateForward(this, 2.5, 0.5, 5);
                    robot.pause(100);
                    //drive to depot
                    robot.gyroAutoDriver.driveForwards(75, 0.5);
                    robot.pause(500);
                    //deposit marker
                    robot.markerDeploy.deploy();
                    robot.pause(250);
                    //Release marker
                    robot.driveTrain.driveRight(this, 1, 0.2, 3);
                    robot.pause(250);
                    //retract marker servo
                    robot.markerDeploy.retract();
                    robot.pause(250);
                    //drive to crater
                    robot.gyroAutoDriver.driveBackwards(130, 0.5);
                    break;

                case MIDDLE:
                    //Drive to and push middle block
                    robot.driveTrain.driveRight(this, 24, defaultSpeed, 10);
                    robot.pause(250);
                    //back up
                    robot.driveTrain.driveLeft(this, 6, defaultSpeed, 3);
                    robot.pause(250);
                    //drive toward the wall
                    robot.driveTrain.translateForward(this, 30, defaultSpeed, 10);
                    robot.pause(250);
                    //turn to be parallel with the wall
                    robot.gyroAutoDriver.turn(115, defaultSpeed, 5);
                    robot.pause(250);
                    //drive toward the wall
                    robot.driveTrain.translateBackward(this, 20, defaultSpeed, 5);
                    robot.pause(250);
                    //drive away from the wall
                    robot.driveTrain.translateForward(this, 2.5, defaultSpeed, 5);
                    robot.pause(250);
                    //drive to depot
                    robot.gyroAutoDriver.driveForwards(90, 0.5);
                    robot.pause(500);
                    //deposit marker
                    robot.markerDeploy.deploy();
                    robot.pause(250);
                    //Release marker
                    robot.driveTrain.driveRight(this, 1, 0.2, 3);
                    robot.pause(250);
                    //retract marker servo
                    robot.markerDeploy.retract();
                    robot.pause(250);
                    //drive to crater
                    robot.gyroAutoDriver.driveBackwards(130, 0.5);
                    break;

                case UNKNOWN:
                    //Drive to and push middle block
                    robot.driveTrain.driveRight(this, 24, defaultSpeed, 10);
                    robot.pause(250);
                    //back up
                    robot.driveTrain.driveLeft(this, 6, defaultSpeed, 3);
                    robot.pause(250);
                    //drive toward the wall
                    robot.driveTrain.translateForward(this, 30, defaultSpeed, 10);
                    robot.pause(250);
                    //turn to be parallel with the wall
                    robot.gyroAutoDriver.turn(115, defaultSpeed, 5);
                    robot.pause(250);
                    //drive toward the wall
                    robot.driveTrain.translateBackward(this, 20, defaultSpeed, 5);
                    robot.pause(250);
                    //drive away from the wall
                    robot.driveTrain.translateForward(this, 2.5, defaultSpeed, 5);
                    robot.pause(250);
                    //drive to depot
                    robot.gyroAutoDriver.driveForwards(90, 0.5);
                    robot.pause(500);
                    //deposit marker
                    robot.markerDeploy.deploy();
                    robot.pause(250);
                    //Release marker
                    robot.driveTrain.driveRight(this, 1, 0.2, 3);
                    robot.pause(250);
                    //retract marker servo
                    robot.markerDeploy.retract();
                    robot.pause(250);
                    //drive to crater
                    robot.gyroAutoDriver.driveBackwards(130, 0.5);
                    break;
                default: //like unknown
                    //drive straight and hope the block is there
                    break;
            }
            break;
        }
    }
}