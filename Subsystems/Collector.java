package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

//Subsystem class. Defines motors/servos/sensors used only by the subsystem, an enumeration outlining the states the subsystem can be in, and functions that control those states

public class Collector {
    //Declare a new DcMotor object
    private DcMotor collector;

    //Create an Enumeration with the states the Collector can be in
    private enum CollectorEnum {
        COLLECTING,
        EJECTING,
        STOPPED
    }

    //Declare a new CollectorEnum
    private CollectorEnum collectorState;

    //Defines the Collector Constructor. It is called in the CombinedHardware class/extension where it joins the other subsystems in the robot object and hardware map.
    //Sets the default state of the Collector
    public Collector(String collector_motor, HardwareMap hardwareMap){
        this.collector = hardwareMap.dcMotor.get(collector_motor);
        stop();
    }

    //Function that sets motor power to set value to collect minerals
    public void collect(){
        collector.setPower(0.7);
        collectorState = CollectorEnum.COLLECTING;
    }

    //Function that sets motor power to set value to eject minerals from collection bin
    public void eject(){
        collector.setPower(-0.7);
        collectorState = CollectorEnum.EJECTING;
    }

    //Functtion that stops the Collector
    public void stop(){
        collector.setPower(0);
        collectorState = CollectorEnum.STOPPED;
    }

    //Strings for each CollectorEnum value, can be printed as telemetry
    @Override
    public String toString(){
        switch (collectorState){
            case COLLECTING:
                return "Collecting Minerals";
            case EJECTING:
                return "Ejecting Minerals from collection bin";
            case STOPPED:
                return "Collector not running";
            default:
                return "Collector state Unknown";
        }
    }

}
