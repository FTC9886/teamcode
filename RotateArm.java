package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RotateArm {
    private DcMotor rotateMotor;

    private enum RotateArmEnum{
        ROTATING_UP,
        ROTATING_DOWN,
        ROTATING,
        STOPPED
    }

    private RotateArmEnum rotateArmState;

    public RotateArm(String rotateMotor, HardwareMap hardwareMap){
        this.rotateMotor = hardwareMap.dcMotor.get(rotateMotor);
        this.rotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stop();
    }

    public void changeAngle(Float power){
        rotateMotor.setPower(power);
        rotateArmState = RotateArm.RotateArmEnum.ROTATING;
    }

    public void angleUp(){
        rotateMotor.setPower(0.2);
        rotateArmState = RotateArm.RotateArmEnum.ROTATING_UP;
    }

    public void angleDown(){
        rotateMotor.setPower(-0.2);
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
            case ROTATING:
                return "Rotating Extender Arm";
            case STOPPED:
                return "Stopped Changing Extender Arm Angle";
            default:
                return "Unknown";
        }
    }
}
