package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MarkerDeploy {
    private Servo markerServo;

    private enum MarkerServoEnum{
        DEPLOY,
        RETRACT,
        STOPPED
    }

    private MarkerServoEnum markerServoState;

    public MarkerDeploy(String markerServo, HardwareMap hardwareMap){
        this.markerServo = hardwareMap.servo.get(markerServo);
        retract();
    }

    public void deploy(){
        markerServo.setPosition(0.5);
        markerServoState = MarkerServoEnum.DEPLOY;
    }

    public void retract(){
        markerServo.setPosition(0);
        markerServoState = MarkerServoEnum.RETRACT;
    }

    public void stop(){

        markerServoState = MarkerServoEnum.STOPPED;
    }



    @Override
    public String toString(){
        switch (markerServoState){
            case DEPLOY:
                return "Deploying";
            case RETRACT:
                return "Retracting";
            case STOPPED:
                return "Stopped";
            default:
                return "Unknown";
        }
    }
}
