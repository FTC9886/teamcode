package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ExtenderArm {
    private DcMotor extensionMotor;
    private DcMotor angleMotor;

    private enum ExtenderArmEnum{
        EXTENDING,
        RETRACTING,
        STOPPED,
        ANGLING_UP,
        ANGLING_DOWN,
        ANGLING_STOPPED
    }

    private ExtenderArmEnum extenderArmState;

    public ExtenderArm(String extendMotor, String angleMotor, HardwareMap hardwareMap){
        this.extensionMotor = hardwareMap.dcMotor.get(extendMotor);
        this.angleMotor = hardwareMap.dcMotor.get(angleMotor);

        stop();
        angleStop();
    }

    public void extend(){
        extensionMotor.setPower(0.5);
        extenderArmState = ExtenderArmEnum.EXTENDING;
    }

    public void retract(){
        extensionMotor.setPower(-0.5);
        extenderArmState = ExtenderArmEnum.RETRACTING;
    }

    public void stop(){
        extensionMotor.setPower(0);
        extenderArmState = ExtenderArmEnum.STOPPED;
    }

    public void angleUp(){
        angleMotor.setPower(0.5);
        extenderArmState = ExtenderArmEnum.ANGLING_UP;
    }

    public void angleDown(){
        angleMotor.setPower(-0.5);
        extenderArmState = ExtenderArmEnum.ANGLING_DOWN;
    }

    public void angleStop(){
        angleMotor.setPower(0);
        extenderArmState = ExtenderArmEnum.ANGLING_STOPPED;
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
            case ANGLING_UP:
                return "Angling Extender Arm Up";
            case ANGLING_DOWN:
                return "Angling Extender Arm Down";
            case ANGLING_STOPPED:
                return "Stopped Changing Extender Arm Angle";
            default:
                return "Unknown";
        }
    }
}
