package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HangArm {
    private DcMotor linearDrive;
    private DigitalChannel upperButton;
    private DigitalChannel lowerButton;

    private enum HangArmEnum {
        RAISING,
        LOWERING,
        STOPPED
    }

    private HangArmEnum hangArmState;

    public HangArm(String linearDrive, String upperTouchSensor, String lowerTouchSensor, HardwareMap hardwareMap) {
        this.linearDrive = hardwareMap.dcMotor.get(linearDrive);
        upperButton = hardwareMap.get(DigitalChannel.class, upperTouchSensor);
        upperButton.setMode(DigitalChannel.Mode.INPUT);

        lowerButton = hardwareMap.get(DigitalChannel.class, lowerTouchSensor);
        lowerButton.setMode(DigitalChannel.Mode.INPUT);

        stop();
    }

    public boolean isFullyRaised() {
        return !upperButton.getState();
    }

    public boolean isFullyLowered() {
        return !lowerButton.getState();
    }


    public void lift() {
        if (isFullyRaised()) {
            stop();
        } else {
            linearDrive.setPower(-0.75);
            hangArmState = HangArmEnum.RAISING;
        }
    }

    public void lower() {
        if (isFullyLowered()) {
            stop();
        } else {
            linearDrive.setPower(0.75);
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
                return "Lowering";

            case STOPPED:
                String telemetry;
                if (isFullyRaised()) {
                    telemetry = "Stopped: Fully Raised";
                } else if (isFullyLowered()) {
                    telemetry = "Stopped: Fully Lowered";
                } else {
                    telemetry = "Stopped: Neither Fully Lowered or Raised";
                }
                return telemetry;

            default:
                return "Unknown";
        }
    }
}
