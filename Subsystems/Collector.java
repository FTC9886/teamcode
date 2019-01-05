package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Collector {

    private DcMotor collector;


    private enum CollectorEnum {
        COLLECTING,
        EJECTING,
        STOPPED
    }

    private CollectorEnum collectorState;

    public Collector(String collector_motor, HardwareMap hardwareMap){
        this.collector = hardwareMap.dcMotor.get(collector_motor);

        stop();
    }

    public void collect(){
        collector.setPower(0.5);
        collectorState = CollectorEnum.COLLECTING;
    }

    public void eject(){
        collector.setPower(-0.5);
        collectorState = CollectorEnum.EJECTING;
    }

    public void stop(){
        collector.setPower(0);
        collectorState = CollectorEnum.STOPPED;
    }

}
