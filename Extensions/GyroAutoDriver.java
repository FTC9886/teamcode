package org.firstinspires.ftc.teamcode.Extensions;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Extensions.CombinedHardware;

public class GyroAutoDriver {
    CombinedHardware hw;

public GyroAutoDriver(CombinedHardware hw){
    this.hw = hw;
    angles = hw.adafruitIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);
}

private Orientation angles;



public double Heading(){
    angles = hw.adafruitIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);

    if(angles.secondAngle == 180)
    {
        angles.secondAngle -= 360;
    }
    return angles.secondAngle;
    }

public void driveBackwards(float distance, double power){
    double target = Heading();
    hw.driveTrain.resetEncoders();
    hw.pause(100);

    while(hw.driveTrain.left_front_drive.getCurrentPosition() < (hw.driveTrain.COUNTS_PER_INCH_LandR * (distance/2)) && hw.driveTrain.left_back_drive.getCurrentPosition() < (hw.driveTrain.COUNTS_PER_INCH_LandR * (distance/2)) /*&& hw.opMode.opModeIsActive()*/)
    {
        double currentHeading = Heading();
        hw.driveTrain.drivePowers(power + (currentHeading - target) / 50,
                power - (currentHeading - target) / 50);
    }
    hw.driveTrain.stopDrive();

}

public void driveForwards(float distance, double power){
    double target = Heading();
    hw.driveTrain.resetEncoders();
    hw.pause(100);

    while(hw.driveTrain.left_front_drive.getCurrentPosition() > -(hw.driveTrain.COUNTS_PER_INCH_LandR * (distance/2)) && hw.driveTrain.left_back_drive.getCurrentPosition() > -(hw.driveTrain.COUNTS_PER_INCH_LandR * (distance/2)) /*&& hw.opMode.opModeIsActive()*/)
    {
        double currentHeading = Heading();  //Current direction

        hw.driveTrain.drivePowers(-power + (currentHeading - target) / 50,
                -power - (currentHeading - target) / 50);

    }

    hw.driveTrain.stopDrive();
}


    public void turn(float anglechange, double speed, double timeoutS)
    {
        angles = hw.adafruitIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);
        double targetangle;
        double heading = Heading();

        targetangle = heading + anglechange;
        if (targetangle > 180) targetangle = targetangle -360;
        if (targetangle < -180) targetangle = targetangle +360;

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        if (anglechange < 0)
        {
            hw.driveTrain.drivePowers(speed, -speed);

        }
        else
        {
            hw.driveTrain.drivePowers(-speed,speed);
        }

        while ((runtime.seconds() < timeoutS) &&
                (Math.abs(heading - targetangle) > 5))
        {
            heading = Heading();
        }

        // Stop all motion;
        hw.driveTrain.stopDrive();
        hw.pause(250);
    }
}
