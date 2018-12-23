package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HangArm {
    private DcMotor linearDrive;
    private DigitalChannel upperButton;


    private enum HangArmEnum {
        RAISING,
        LOWERING,
        STOPPED
    }

    private HangArmEnum hangArmState;

    public HangArm(String linearDrive, /*String upperTouchSensor,*/  HardwareMap hardwareMap) {
        this.linearDrive = hardwareMap.dcMotor.get(linearDrive);
        /*upperButton = hardwareMap.get(DigitalChannel.class, upperTouchSensor);
        upperButton.setMode(DigitalChannel.Mode.INPUT);*/



        stop();
    }

    public boolean isFullyLowered() {
        return !upperButton.getState();
    }




    public void lift() {
        linearDrive.setPower(-0.75);
        hangArmState = HangArmEnum.RAISING;
    }

    public void lower() {
        /*if (isFullyLowered()) {
            stop();
        } else {
            linearDrive.setPower(0.75);
            hangArmState = HangArmEnum.LOWERING;
        }*/

        linearDrive.setPower(0.75);
        hangArmState = HangArmEnum.LOWERING;
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
                return "Lowering";

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
