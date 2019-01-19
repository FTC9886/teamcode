package org.firstinspires.ftc.teamcode.Extensions;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Extensions.CombinedHardware;
//Class that uses the IMU built into the REV expansion hub to make movement during autonomous more accurate
public class GyroAutoDriver {
    //Calls the hardware map
    CombinedHardware hw;

    //Defines the constructor
public GyroAutoDriver(CombinedHardware hw){
    this.hw = hw;
    angles = hw.adafruitIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);
}
//New orientation object
private Orientation angles;


//Double that stores the angle the robot is facing
public double Heading(){
    angles = hw.adafruitIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);

    //Prevents robot from freaking out when rotating past the 180 degree point
    if(angles.secondAngle == 180)
    {
        angles.secondAngle -= 360;
    }
    return angles.secondAngle;
    }

    //Function that takes a distance in inches and a drive power. Drives backwards that distance, during motion robot automatically adjusts its motor powers down from the set power to maintain a straight path
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

//Function that takes a distance in inches and a drive power. Drives forwards that distance, during motion robot automatically adjusts its motor powers down from the set power to maintain a straight path
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

//Function that takes an angle, a speed, and a max time to turn.
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
