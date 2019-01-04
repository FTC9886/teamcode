/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mec_Tank_v4", group="Mec_Tank_v4")
@Disabled
public class Teleop_Tank_v4 extends OpMode{

    /* Declare OpMode members. */
    Hardware robot       = new Hardware(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.

    /*
     * Code to run ONCE when the driver hits INIT
     * Code to run ONCE when the driver hits INIT
     * Code to run ONCE when the driver hits INIT
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "hello Driver");    //
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

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
            robot.left_front_drive.setPower (gamepad1_left);
            robot.right_front_drive.setPower(gamepad1_right);
            robot.left_back_drive.setPower  (-gamepad1_left);
            robot.right_back_drive.setPower (-gamepad1_right);
        }

//        else if (gamepad1.dpad_up)
//        {
//            robot.left_front_drive.setPower (0.3);
//            robot.right_front_drive.setPower(0.3);
//        }
//        else if (gamepad1.dpad_down)
//        {
//            robot.left_front_drive.setPower (-0.3);
//            robot.right_front_drive.setPower(-0.3);
//        }

        //Translate left and right
        else if (gamepad1.right_trigger > 0.1)
        {
            robot.left_front_drive.setPower (-gamepad1.right_trigger/1.5);
            robot.right_front_drive.setPower(-gamepad1.right_trigger/1.5);
            robot.left_back_drive.setPower  (-gamepad1.right_trigger/1.5);
            robot.right_back_drive.setPower (-gamepad1.right_trigger/1.5);
        }
        else if (gamepad1.left_trigger > 0.1)
        {
            robot.left_front_drive.setPower (gamepad1.left_trigger/1.5);
            robot.right_front_drive.setPower(gamepad1.left_trigger/1.5);
            robot.left_back_drive.setPower  (gamepad1.left_trigger/1.5);
            robot.right_back_drive.setPower (gamepad1.left_trigger/1.5);
        }
        else
        {
            robot.left_front_drive.setPower (0f);
            robot.right_front_drive.setPower(0f);
            robot.left_back_drive.setPower  (0f);
            robot.right_back_drive.setPower (0f);
        }


        // to move the lift/hang arm up and down. left bumper is up and right bumper is down.
        if (gamepad1.left_bumper){
            robot.hang_arm.setPower(-0.75);
        }else if (gamepad1.right_bumper){
            robot.hang_arm.setPower(0.75);
        } else {
            robot.hang_arm.setPower(0);
        }

        telemetry.addData("lift encoder position",robot.hang_arm.getCurrentPosition());

        // to extend and contract the collection arm. "b" is extending and "x" is contracting.
        if (gamepad1.b){
            robot.extend_arm.setPower(0.80);
        }else if (gamepad1.x){
            robot.extend_arm.setPower(-0.80);
        } else {
            robot.extend_arm.setPower(0);
        }

        // to rotate the collection arm up and down. "a" is to rotate up and "y" is to rotate downward.
        if (gamepad1.a){
            robot.rotate_arm.setPower(0.75);
        }else if (gamepad1.y){
            robot.rotate_arm.setPower(-0.75);
        } else {
            robot.rotate_arm.setPower(0);
        }

        // to rotate the collection system. "start" turns it on and "back" turns it off.
        if (gamepad1.start){
            robot.collector.setPower(0.50);
        }else if (gamepad1.back){
            robot.collector.setPower(-0.50);
        } else {
            robot.collector.setPower(0);
        }

        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {
    }
}
