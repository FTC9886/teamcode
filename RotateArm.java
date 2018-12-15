package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RotateArm {
    private DcMotor rotateMotor;

    private enum RotateArmEnum{
        ROTATING_UP,
        ROTATING_DOWN,
        STOPPED
    }

    private RotateArmEnum rotateArmState;

    public RotateArm(String rotateMotor, HardwareMap hardwareMap){
        this.rotateMotor = hardwareMap.dcMotor.get(rotateMotor);

        stop();
    }

    public void angleUp(){
        rotateMotor.setPower(0.5);
        rotateArmState = RotateArm.RotateArmEnum.ROTATING_UP;
    }

    public void angleDown(){
        rotateMotor.setPower(-0.5);
        rotateArmState = RotateArm.RotateArmEnum.ROTATING_DOWN;
    }

    public void stop(){
        rotateMotor.setPower(0);
        rotateArmState = RotateArm.RotateArmEnum.STOPPED;
    }


    @Override
    public String toString(){
        switch (rotateArmState){
            case ROTATING_UP:
                return "Rotate Extender Arm Up";
            case ROTATING_DOWN:
                return "Rotate Extender Arm Down";
            case STOPPED:
                return "Stopped Changing Extender Arm Angle";
            default:
                return "Unknown";
        }
    }
}
