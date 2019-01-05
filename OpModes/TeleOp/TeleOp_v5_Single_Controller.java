package org.firstinspires.ftc.teamcode.OpModes.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Extensions.CombinedHardware;

@TeleOp(name = "Mec Tank v5 Single Controller", group = "Mec Tank v5")
@Disabled
public class TeleOp_v5_Single_Controller extends OpMode {
    CombinedHardware robot = new CombinedHardware();



    @Override
    public void init(){
        robot.init(hardwareMap);
    }

    @Override
    public void loop(){
        // ********************************************************
        // CONTROLLER #1 DRIVER
        // ********************************************************
        // Tank drive and translate controls
        double gamepad1_left  = 0.0;
        double gamepad1_right = 0.0;
        gamepad1_left = gamepad1.left_stick_y;
        gamepad1_right = -gamepad1.right_stick_y;
        if(gamepad1_left > 0.1 || gamepad1_left < -0.1 || gamepad1_right > 0.1 || gamepad1_right < -0.1)
        {
            robot.driveTrain.left_front_drive.setPower (gamepad1_left);
            robot.driveTrain.right_front_drive.setPower(gamepad1_right);
            robot.driveTrain.left_back_drive.setPower  (-gamepad1_left);
            robot.driveTrain.right_back_drive.setPower (-gamepad1_right);
        }
        //Translate left and right
        else if (gamepad1.right_trigger > 0.1)
        {
            robot.driveTrain.left_front_drive.setPower (-gamepad1.right_trigger/1.5);
            robot.driveTrain.right_front_drive.setPower(-gamepad1.right_trigger/1.5);
            robot.driveTrain.left_back_drive.setPower  (-gamepad1.right_trigger/1.5);
            robot.driveTrain.right_back_drive.setPower (-gamepad1.right_trigger/1.5);
        }
        else if (gamepad1.left_trigger > 0.1)
        {
            robot.driveTrain.left_front_drive.setPower (gamepad1.left_trigger/1.5);
            robot.driveTrain.right_front_drive.setPower(gamepad1.left_trigger/1.5);
            robot.driveTrain.left_back_drive.setPower  (gamepad1.left_trigger/1.5);
            robot.driveTrain.right_back_drive.setPower (gamepad1.left_trigger/1.5);
        }
        //D-pad Driving
        else if (gamepad1.dpad_up)
        {
            robot.driveTrain.left_front_drive.setPower (-0.3);
            robot.driveTrain.right_front_drive.setPower(0.3);
            robot.driveTrain.left_back_drive.setPower  (0.3);
            robot.driveTrain.right_back_drive.setPower (-0.3);
            //robot.drivePowers(0.25, -0.25);
        }
        else if (gamepad1.dpad_down)
        {
            robot.driveTrain.left_front_drive.setPower (0.3);
            robot.driveTrain.right_front_drive.setPower(-0.3);
            robot.driveTrain.left_back_drive.setPower  (-0.3);
            robot.driveTrain.right_back_drive.setPower (0.3);
            //robot.drivePowers(-0.25, 0.25);
        }
        else if (gamepad1.dpad_left)
        {
            robot.driveTrain.left_front_drive.setPower (0.3);
            robot.driveTrain.right_front_drive.setPower(0.3);
            robot.driveTrain.left_back_drive.setPower  (0.3);
            robot.driveTrain.right_back_drive.setPower (0.3);

            //robot.powerTranslate(0.25);
        }
        else if (gamepad1.dpad_right)
        {
            robot.driveTrain.left_front_drive.setPower (-0.3);
            robot.driveTrain.right_front_drive.setPower(-0.3);
            robot.driveTrain.left_back_drive.setPower  (-0.3);
            robot.driveTrain.right_back_drive.setPower (-0.3);
            //robot.powerTranslate(-0.25);
        }
        else
        {
            robot.driveTrain.stopDrive();
        }

        //Hang lift
        if (gamepad1.back)
        {
            robot.hangArm.lower();
        }
        else if (gamepad1.start)
        {
            robot.hangArm.lift();
        }
        else
        {
            robot.hangArm.stop();
        }

        //Collector arm extend/retract
        if (gamepad1.b)
        {
            robot.extenderArm.extend();
        }
        else if (gamepad1.x)
        {
            robot.extenderArm.retract();
        }
        else
        {
            robot.extenderArm.stop();
        }

        //Rotate Collector Arm
        if (gamepad1.y)
        {
            robot.rotateArm.angleUp();
        }
        else if (gamepad1.a)
        {
            robot.rotateArm.angleDown();
        }
        else
        {
            robot.rotateArm.stop();
        }

        //Collector
        if (gamepad1.left_bumper)
        {
            robot.collector.eject();
        }
        else if (gamepad1.right_bumper)
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
