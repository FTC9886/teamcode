package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

//Subsystem class. Defines motors/servos/sensors used only by the subsystem, an enumeration outlining the states the subsystem can be in, and functions that control those states

public class HangArm {
    //Declare a new DcMotor object
    private DcMotor linearDrive;

    //Creates integer which is set to target encoder count for the linearDrive when lowering from the Lander during autonomous
    private int MaxEncoderValue = -12634;
    //Crates integer that stores the current encoder count of the LinearDrive
    private int CurrentEncoderValue = 0;

    //Creates a new enumeration with values corresponding to the states the HangArm can be in
    private enum HangArmEnum {
        RAISING,
        LOWERING,
        STOPPED
    }

    //Declares a new HangArmEnum
    private HangArmEnum hangArmState;

    //Defines the HangArm Constructor. It is called in the CombinedHardware class/extension where it joins the other subsystems in the robot object and hardware map.
    //Sets the default state of the HangArm
    public HangArm(String linearDriveName,  HardwareMap hardwareMap) {
        this.linearDrive = hardwareMap.dcMotor.get(linearDriveName);
        linearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         stop();
    }

    //Creates boolean that determines if the robot has been completely lowered from the Lander during autonomous
    public boolean isFullyLowered() {
        int currentPosition = linearDrive.getCurrentPosition();
        boolean isDown = currentPosition < MaxEncoderValue + 20;
        return isDown;
    }

    //Function that drives the linearDrive so that the HangArm is pulled into the robot, when hooked to the lander this lifts the robot
    public void lift() {
        linearDrive.setPower(0.75);
        hangArmState = HangArmEnum.RAISING;
    }

    //Function that drives the linearDrive to that the HangArm is extended from the robot, when hooked to the lander this lowers the robot to the field
    public void lower() {
        if (isFullyLowered()) {
            stop();
        } else {
            linearDrive.setPower(-0.75);
            hangArmState = HangArmEnum.LOWERING;
        }
    }

    //Function that stops the linearDrive
    public void stop() {
        linearDrive.setPower(0);
        hangArmState = HangArmEnum.STOPPED;
    }

    //String that uses the values of the HangArmEnum, can be printed as telemetry
    @Override
    public String toString() {
        switch (hangArmState) {
            case RAISING:
                return "Raising";

            case LOWERING:
                return "Lowering " + linearDrive.getCurrentPosition();

            case STOPPED:
                String telemetry;
                if (isFullyLowered()) {
                    telemetry = "Stopped: Fully Lowered";
                }else{
                    telemetry = "Stopped: Not Fully Lowered";
                }
                return telemetry;

            default:
                return "Unknown";
        }
    }
}
