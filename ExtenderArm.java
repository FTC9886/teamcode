package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ExtenderArm {
    private DcMotor extensionMotor;

    private enum ExtenderArmEnum{
        EXTENDING,
        RETRACTING,
        STOPPED
    }

    private ExtenderArmEnum extenderArmState;

    public ExtenderArm(String extendMotor, HardwareMap hardwareMap){
        this.extensionMotor = hardwareMap.dcMotor.get(extendMotor);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stop();
    }

    public void extend(){
        extensionMotor.setPower(0.8);
        extenderArmState = ExtenderArmEnum.EXTENDING;
    }

    public void retract(){
        extensionMotor.setPower(-0.8);
        extenderArmState = ExtenderArmEnum.RETRACTING;
    }

    public void stop(){
        extensionMotor.setPower(0);
        extenderArmState = ExtenderArmEnum.STOPPED;
    }



    @Override
    public String toString(){
        switch (extenderArmState){
            case EXTENDING:
                return "Extending";
            case RETRACTING:
                return "Retracting";
            case STOPPED:
                return "Stopped";
            default:
                return "Unknown";
        }
    }
}
