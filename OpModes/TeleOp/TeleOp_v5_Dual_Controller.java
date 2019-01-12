package org.firstinspires.ftc.teamcode.OpModes.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Extensions.CombinedHardware;

/*
TeleOp_V5_Dual_Controller is our team's primary TeleOp OpMode, it is built to split control of the robot between two drivers.
The first driver controls the DriveTrain using both joysticks, the d-pad, and triggers and the HangArm using the 'A' and 'Y' buttons.
The second driver controls the ExtenderArm using the 'A' and 'Y' buttons, the RotateArm using the d-pad and left joystick, and the Collector using the bumpers.
 */

@TeleOp(name = "Mec Tank v5", group = "Mec Tank v5")
public class TeleOp_v5_Dual_Controller extends OpMode {
    //Call Combined hardware as new object 'robot'
    CombinedHardware robot = new CombinedHardware();


    @Override
    public void init(){
        //initialize hardware map and set motors to run without encoders
        robot.init(hardwareMap);
        robot.driveTrain.left_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveTrain.right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveTrain.left_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveTrain.right_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    public void init_loop() {
        // If you are using Motorola E4 phones,
        // you should send telemetry data while waiting for start.
        telemetry.addData("status", "loop test... waiting for start");
        telemetry.update();
    }



    @Override
    public void loop(){
        // ********************************************************
        // CONTROLLER #1 DRIVER
        // ********************************************************
        // Tank drive, translate controls, and hang controls
        double gamepad1_left  = 0.0;
        double gamepad1_right = 0.0;
        gamepad1_left = gamepad1.left_stick_y;
        gamepad1_right = -gamepad1.right_stick_y;
        if(gamepad1_left > 0.1 || gamepad1_left < -0.1 || gamepad1_right > 0.1 || gamepad1_right < -0.1)
        {
            robot.driveTrain.stickPower(gamepad1_left,gamepad1_right);
        }
        //Translate left and right
        else if (gamepad1.right_trigger > 0.1)
        {
            robot.driveTrain.teleTranslate(-gamepad1.right_trigger/1.5);
        }
        else if (gamepad1.left_trigger > 0.1)
        {
            robot.driveTrain.teleTranslate(gamepad1.left_trigger/1.5);
        }
        //D-pad Driving
        else if (gamepad1.dpad_up)
        {
            robot.driveTrain.forwards(0.3);
        }
        else if (gamepad1.dpad_down)
        {
            robot.driveTrain.backwards(0.3);
        }
        else if (gamepad1.dpad_left)
        {
            robot.driveTrain.left(0.3);

        }
        else if (gamepad1.dpad_right)
        {
            robot.driveTrain.right(0.3);

        }
        else
        {
            robot.driveTrain.stopDrive();
        }

        //Hang lift
        if(gamepad1.a)
        {
            robot.hangArm.lower();
        }
        else if(gamepad1.y)
        {
            robot.hangArm.lift();
        }
        else
        {
            robot.hangArm.stop();
        }


        // ********************************************************
        // CONTROLLER #2 Gunner
        // ********************************************************
        // Collector, arm extension, arm rotation


        //Collector arm extend/retract
        if (gamepad2.y)
        {
            robot.extenderArm.extend();
        }
        else if (gamepad2.a)
        {
            robot.extenderArm.retract();
        }
        else
        {
            robot.extenderArm.stop();
        }

        //Rotate Collector Arm
        if (gamepad2.dpad_up)
        {
            robot.rotateArm.angleUp();
        }
        else if(gamepad2.dpad_down)
        {
            robot.rotateArm.angleDown();
        }
        else
        {
            robot.rotateArm.changeAngle(gamepad2.left_stick_y);
        }
//        //read arm position
//        //parallel to ground is 3.22 Volts, Perpendicular 1.52 Volts, Back about 25 degrees 1.28 Volts
//        double arm_position = 0;
//        arm_position = robot.potentiometer.getVoltage();
//        telemetry.addData("arm_position =     ", arm_position);

        //Collector
        if (gamepad2.left_bumper)
        {
            robot.collector.eject();
        }
        else if(gamepad2.right_bumper)
        {
            robot.collector.collect();
        }
        else
        {
            robot.collector.stop();
        }


        telemetry.update();
    }

    @Override
    public void stop(){
        robot.driveTrain.stopDrive();
        robot.collector.stop();
        robot.extenderArm.stop();
        robot.rotateArm.stop();
        robot.hangArm.stop();
    }
}
