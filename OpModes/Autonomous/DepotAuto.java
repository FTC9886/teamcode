package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.Extensions.CombinedHardware;
import org.firstinspires.ftc.teamcode.Extensions.Michaels_tensor_flow;

@Autonomous(name="DepotAuto", group="Autonomous")
//@Disabled

public class DepotAuto extends LinearOpMode {
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
        CameraDevice.getInstance().setFlashTorchMode(true);
        double defaultSpeed = 0.4;
        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        //change to use a more robust function later
        while(!robot.hangArm.isFullyLowered()) {
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
        timer.startTime();
        while(goldPosition == Michaels_tensor_flow.goldfinder.UNKNOWN) {
            goldPosition = tensor_flow.getGoldPosition(telemetry);
//            if (timer.seconds() > 5){
//                break;
//            }
        }
        telemetry.addData("Position", goldPosition);
        telemetry.update();
        robot.pause(1000);
        tensor_flow.deactivate();

        //Robot chooses which path to take, Left, Right, Middle, or Unknown

        switch (goldPosition) {
            case LEFT:
                    robot.driveTrain.driveRight(this, 14, defaultSpeed, 3);
                    robot.pause(250);
                //drive 2" from minerals
                    robot.driveTrain.translateForward(this, 21, defaultSpeed, 5);
                    robot.pause(250);
                //drive to left block
                    robot.driveTrain.driveRight(this, 10, defaultSpeed, 5);
                    robot.pause(250);
                //Pull back from block
                    robot.driveTrain.driveLeft(this, 8, defaultSpeed, 3);
                    robot.pause(250);
                //turn robot to the left to face the wall
                    robot.gyroAutoDriver.turn(-30, 0.2, 5);
                    robot.pause(250);
                //drive toward the wall
                    robot.driveTrain.translateForward(this, 26, defaultSpeed, 5);
                    robot.pause(250);
                //drive away from the wall
                    robot.driveTrain.translateBackward(this, 5, 0.3, 5);
                    robot.pause(100);
                //drive to depot
                    robot.gyroAutoDriver.driveForwards(40, 0.5);
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
                    robot.gyroAutoDriver.driveBackwards(120, 0.5);
                break;

            case RIGHT:
                    robot.driveTrain.driveRight(this, 14, defaultSpeed, 3);
                    robot.pause(250);
                //drive 2" from minerals
                    robot.driveTrain.translateBackward(this, 13.5, 0.2, 5);
                    robot.pause(250);
                //drive to right block
                    robot.driveTrain.driveRight(this, 6, defaultSpeed, 5);
                    robot.pause(250);
                //back up
                    robot.driveTrain.driveLeft(this, 5, defaultSpeed, 3);
                    robot.pause(250);
                //drive toward the wall
                    robot.driveTrain.translateForward(this, 60, defaultSpeed, 10);
                    robot.pause(250);
                //turn to be parallel with the wall
                    robot.gyroAutoDriver.turn(-30, 0.20, 5);
                    robot.pause(250);
                //drive toward the wall
                    robot.driveTrain.translateForward(this, 26, defaultSpeed, 5);
                    robot.pause(250);
                //drive away from the wall
                    robot.driveTrain.translateBackward(this, 5, 0.5, 5);
                    robot.pause(100);
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
                    robot.gyroAutoDriver.driveBackwards(140, 0.5);
                break;

            case MIDDLE:
                //Drive to and push middle block to the depot
                    robot.driveTrain.driveRight(this, 55, defaultSpeed, 10);
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
                //back off of the block to avoid hitting it when turning.
                    robot.driveTrain.driveLeft(this, 5, 0.2, 3);
                    robot.pause(250);
                //turn to be parallel with wall
                    robot.gyroAutoDriver.turn(-30, defaultSpeed, 5);
                    robot.pause(250);
                //drive toward the wall
                    robot.driveTrain.translateForward(this, 20, defaultSpeed, 5);
                    robot.pause(250);
                //drive away from the wall
                    robot.driveTrain.translateBackward(this, 5, defaultSpeed, 5);
                    robot.pause(250);
                //drive to crater
                    robot.gyroAutoDriver.driveBackwards(125, 0.5);
                break;

            case UNKNOWN:
                //Drive to and push middle block to the depot
                robot.driveTrain.driveRight(this, 55, defaultSpeed, 10);
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
                //back off of the block to avoid hitting it when turning.
                robot.driveTrain.driveLeft(this, 5, 0.2, 3);
                robot.pause(250);
                //turn to be parallel with wall
                robot.gyroAutoDriver.turn(-30, defaultSpeed, 5);
                robot.pause(250);
                //drive toward the wall
                robot.driveTrain.translateForward(this, 20, defaultSpeed, 5);
                robot.pause(250);
                //drive away from the wall
                robot.driveTrain.translateBackward(this, 5, defaultSpeed, 5);
                robot.pause(250);
                //drive to crater
                robot.gyroAutoDriver.driveBackwards(125, 0.5);
                break;
            default: //like unknown
                //drive straight and hope the block is there
                break;
        }
    }
}