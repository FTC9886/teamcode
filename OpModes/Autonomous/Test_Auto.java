package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.CombinedHardware;

@Autonomous(name="Test_Auto", group="Autonomous")
@Disabled
public class Test_Auto extends LinearOpMode{
    public void runOpMode(){
        CombinedHardware robot = new CombinedHardware();
        robot.init(hardwareMap);


        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a) {
                robot.driveTrain.drivePowers(0.5, 0.5);
            } else if (gamepad1.b){
                robot.driveTrain.drivePowers(-0.5, -0.5);
        }else if(gamepad1.x){
                robot.gyroAutoDriver.driveForwards(5, 0.5);
         }else if(gamepad1.y){
                robot.gyroAutoDriver.driveBackwards(5, 0.5);

            }else if(gamepad1.right_bumper){
                robot.gyroAutoDriver.driveForwards(10, 0.5);
            }else if(gamepad1.left_bumper){
                robot.gyroAutoDriver.driveBackwards(10, 0.5);
            }else{
                robot.driveTrain.stopDrive();
            }


        }


    }
}
