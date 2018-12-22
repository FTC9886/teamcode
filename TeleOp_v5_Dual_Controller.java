package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Mec Tank v5 Dual Controller", group = "Mec Tank v5")
public class TeleOp_v5_Dual_Controller extends OpMode {
    CombinedHardware robot = new CombinedHardware();



    @Override
    public void init(){
        robot.init(hardwareMap);

        updateTelemetry(telemetry);

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
            robot.left_front_drive.setPower (gamepad1_left);
            robot.right_front_drive.setPower(gamepad1_right);
            robot.left_back_drive.setPower  (gamepad1_left);
            robot.right_back_drive.setPower (gamepad1_right);
        }
        //Translate left and right
        else if (gamepad1.right_trigger > 0.1)
        {
            robot.left_front_drive.setPower (-gamepad1.right_trigger/1.5);
            robot.right_front_drive.setPower(-gamepad1.right_trigger/1.5);
            robot.left_back_drive.setPower  (gamepad1.right_trigger/1.5);
            robot.right_back_drive.setPower (gamepad1.right_trigger/1.5);
        }
        else if (gamepad1.left_trigger > 0.1)
        {
            robot.left_front_drive.setPower (gamepad1.left_trigger/1.5);
            robot.right_front_drive.setPower(gamepad1.left_trigger/1.5);
            robot.left_back_drive.setPower  (-gamepad1.left_trigger/1.5);
            robot.right_back_drive.setPower (-gamepad1.left_trigger/1.5);
        }
        else
        {
            robot.left_front_drive.setPower (0f);
            robot.right_front_drive.setPower(0f);
            robot.left_back_drive.setPower  (0f);
            robot.right_back_drive.setPower (0f);
        }

        //D-pad Driving
        if (gamepad1.dpad_up){
            robot.powerTranslate(0.25);
        }else if (gamepad1.dpad_down){
            robot.powerTranslate(-0.25);
        }else if(gamepad1.dpad_left){
            robot.drivePowers(0.25, 0.25);
        }else if(gamepad1.dpad_right){
            robot.drivePowers(-0.25, -0.25);
        }

        //Hang lift
        if(gamepad1.a){
            robot.hangArm.lower();
        }else if(gamepad1.y){
            robot.hangArm.lift();
        }else{
            robot.hangArm.stop();
        }


        // ********************************************************
        // CONTROLLER #2 Gunner
        // ********************************************************
        // Collector, arm extension, arm rotation


        //Collector arm extend/retract
        if (gamepad2.b){
            robot.extenderArm.extend();
        }else if(gamepad2.x){
            robot.extenderArm.retract();
        } else {
            robot.extenderArm.stop();
        }

        //Rotate Collector Arm
        if (gamepad2.y){
            robot.rotateArm.angleUp();
        }else if(gamepad2.a){
            robot.rotateArm.angleDown();
        }else{
            robot.rotateArm.stop();
        }

        //Collector
        if (gamepad2.left_bumper){
            robot.collector.eject();
        }else if(gamepad2.right_bumper){
            robot.collector.collect();
        }else{
            robot.collector.stop();
        }

        telemetry.update();
    }

    @Override
    public void stop(){
        robot.stopDrive();
        robot.collector.stop();
        robot.extenderArm.stop();
        robot.rotateArm.stop();
        robot.hangArm.stop();
    }
}
