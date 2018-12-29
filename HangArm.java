package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HangArm {
    private DcMotor linearDrive;
    private int MaxEncoderValue = -12034;
    private int CurrentEncoderValue = 0;


    private enum HangArmEnum {
        RAISING,
        LOWERING,
        STOPPED
    }

    private HangArmEnum hangArmState;

    public HangArm(String linearDriveName,  HardwareMap hardwareMap) {
        this.linearDrive = hardwareMap.dcMotor.get(linearDriveName);
        linearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // linearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         stop();
    }

    public boolean isFullyLowered() {
        int currentPosition = linearDrive.getCurrentPosition();
        boolean isDown = currentPosition < MaxEncoderValue + 20;
        return isDown;
    }





    public void lift() {
        linearDrive.setPower(0.75);
        hangArmState = HangArmEnum.RAISING;
    }

    public void lower() {
        if (isFullyLowered()) {
            stop();
        } else {
            //linearDrive.setTargetPosition(MaxEncoderValue);
            linearDrive.setPower(-0.75);
            hangArmState = HangArmEnum.LOWERING;
        }
    }

    public void stop() {
        linearDrive.setPower(0);
        hangArmState = HangArmEnum.STOPPED;
    }

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
