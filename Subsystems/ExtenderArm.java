package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

//Subsystem class. Defines motors/servos/sensors used only by the subsystem, an enumeration outlining the states the subsystem can be in, and functions that control those states

public class ExtenderArm {
    //Declares a new DcMotor object
    private DcMotor extensionMotor;

    //Creates a new enumeration with the possible states of the ExtenderArm as its values
    private enum ExtenderArmEnum{
        EXTENDING,
        RETRACTING,
        STOPPED
    }

    //Declares a new ExtenderArmEnum
    private ExtenderArmEnum extenderArmState;

    //Defines the ExtenderArm Constructor. It is called in the CombinedHardware class/extension where it joins the other subsystems in the robot object and hardware map.
    //Sets the default state of the ExtenderArm
    public ExtenderArm(String extendMotor, HardwareMap hardwareMap){
        this.extensionMotor = hardwareMap.dcMotor.get(extendMotor);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stop();
    }

    //Function that drives the extensionMotor forwards, extending the arm
    public void extend(){
        extensionMotor.setPower(0.8);
        extenderArmState = ExtenderArmEnum.EXTENDING;
    }

    //Function that drives the extensionMotor backwards, retracting the arm
    public void retract(){
        extensionMotor.setPower(-0.8);
        extenderArmState = ExtenderArmEnum.RETRACTING;
    }

    //Function that passes a power of 0 to the extensionArm, stopping the arm
    public void stop(){
        extensionMotor.setPower(0);
        extenderArmState = ExtenderArmEnum.STOPPED;
    }


//String that uses the values of the ExtenderArmEnum, can be printed as telemetry
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
