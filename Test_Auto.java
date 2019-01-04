package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="Test_Auto", group="Autonomous")
@Disabled
public class Test_Auto extends LinearOpMode{
    public void runOpMode(){
        CombinedHardware robot = new CombinedHardware();
        robot.init(hardwareMap);
        GyroAutoDriver driver = new GyroAutoDriver(robot);

        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a) {
                robot.drivePowers(0.5, 0.5);
            } else if (gamepad1.b){
                robot.drivePowers(-0.5, -0.5);
        }else if(gamepad1.x){
                robot.gyroAutoDriver.driveForwards(5, 0.5);
         }else if(gamepad1.y){
                robot.gyroAutoDriver.driveBackwards(5, 0.5);

            }else if(gamepad1.right_bumper){
                robot.gyroAutoDriver.driveForwards(10, 0.5);
            }else if(gamepad1.left_bumper){
                robot.gyroAutoDriver.driveBackwards(10, 0.5);
            }else{
                robot.stopDrive();
            }

         telemetry.addData("Angle ", driver.Heading());
         telemetry.update();
        }


    }
}
