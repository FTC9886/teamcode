package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RotateArm {
    //Declare a new DcMotor object
    private DcMotor rotateMotor;

    //Create an Enumeration with the states the RotateArm can be in
    private enum RotateArmEnum{
        ROTATING_UP,
        ROTATING_DOWN,
        ROTATING,
        STOPPED
    }

    //Declare a new RotateArmEnum
    private RotateArmEnum rotateArmState;

    //Defines the RotateArm constructor. It is called in the CombinedHardware class/extension where it joins the other subsystems in the robot object and hardware map.
    //Sets the default state of the RotateArm
    public RotateArm(String rotateMotor, HardwareMap hardwareMap){
        this.rotateMotor = hardwareMap.dcMotor.get(rotateMotor);
        this.rotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stop();
    }

    //Function which takes a float value and uses that value to set motor power. Currently used in TeleOp to control the RotateArm with a joystick
    public void changeAngle(Float power){
        rotateMotor.setPower(power);
        rotateArmState = RotateArm.RotateArmEnum.ROTATING;
    }

    //Function that rotates the arm up at a static power
    public void angleUp(){
        rotateMotor.setPower(-0.2);
        rotateArmState = RotateArm.RotateArmEnum.ROTATING_UP;
    }

    //Function that rotates the arm down at a static power
    public void angleDown(){
        rotateMotor.setPower(0.2);
        rotateArmState = RotateArm.RotateArmEnum.ROTATING_DOWN;
    }

    //Function that stops the arm
    public void stop(){
        rotateMotor.setPower(0);
        rotateArmState = RotateArm.RotateArmEnum.STOPPED;
    }

    //String that uses the values from the RotateArmEnum. Can be printed as telemetry
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
