package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name="Blue_Crater_BC_Fast", group="Autonomous")
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
        robot.hang_arm.setPower(0.75);
        robot.pause(5000);
        robot.hang_arm.setPower(0.0);


        tensor_flow.activate();
        //FIND GOLD
        Michaels_tensor_flow.goldfinder goldPosition = tensor_flow.getGoldPosition(telemetry);

        tensor_flow.deactivate();

        //lower down
        robot.driveRight(this, 10, 0.5, 5);

        try {
            robot.wait(30000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


        robot.driveLeft(this, 10, 0.5, 5);

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
