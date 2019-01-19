package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MarkerDeploy {
    //Declares a new servo object
    private Servo markerServo;

    //Creates a new enumeration with the states the MarkerDeploy subsystem can be in
    private enum MarkerServoEnum{
        DEPLOY,
        RETRACT,
        STOPPED
    }

    //Creates a new MarkerServoEnum
    private MarkerServoEnum markerServoState;

    //Defines the MarkerDeploy constructor. It is called in the CombinedHardware class/extension where it joins the other subsystems in the robot object and hardware map.
    //Sets the default state of the MarkerDeploy
    public MarkerDeploy(String markerServo, HardwareMap hardwareMap){
        this.markerServo = hardwareMap.servo.get(markerServo);
        retract();
    }

    //Function that sets the markerServo to the position that will release the marker from the robot
    public void deploy(){
        markerServo.setPosition(0.5);
        markerServoState = MarkerServoEnum.DEPLOY;
    }

    //Function that sets the markerServo to the position that holds the marker in the robot, and retracts the markerServo arm back into the robot after deploying the marker
    public void retract(){
        markerServo.setPosition(0);
        markerServoState = MarkerServoEnum.RETRACT;
    }

    //Function that sets the markerServo state to stopped. Unused
    public void stop(){
        markerServoState = MarkerServoEnum.STOPPED;
    }



    //String that uses the values of the MarkerServoEnum. Can be printed as telemetry
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
