package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
    hw.resetEncoders();
    hw.pause(100);
//    hw.rightDrivePower(power);
//    hw.leftDrivePower(power);

    while(hw.left_front_drive.getCurrentPosition() < (hw.COUNTS_PER_INCH_LandR * (distance/2)) && hw.left_back_drive.getCurrentPosition() < (hw.COUNTS_PER_INCH_LandR * (distance/2)) /*&& hw.opMode.opModeIsActive()*/)
    {
        double currentHeading = Heading();
        hw.drivePowers(power + (currentHeading - target) / 50,
                power - (currentHeading - target) / 50);
    }
    hw.stopDrive();

}

public void driveForwards(float distance, double power){
    double target = Heading();
    hw.resetEncoders();
    hw.pause(100);

    while(hw.left_front_drive.getCurrentPosition() > -(hw.COUNTS_PER_INCH_LandR * (distance/2)) && hw.left_back_drive.getCurrentPosition() > -(hw.COUNTS_PER_INCH_LandR * (distance/2)) /*&& hw.opMode.opModeIsActive()*/)
    {
        double currentHeading = Heading();  //Current direction

        hw.drivePowers(-power + (currentHeading - target) / 50,
                -power - (currentHeading - target) / 50);

    }

    hw.stopDrive();
}

//    public void turn(int target, double power)
//    {
//        double initialValue = angles.firstAngle;  //Starting direction
//
//        double gyroTarget = initialValue + 90;
//
//
//
//
//        if(gyroTarget > 180)
//        {
//            gyroTarget -= 360;
//        }
//        else if(gyroTarget < -180)
//        {
//            gyroTarget += 360;
//        }
//
//        boolean shouldBeRunning = true;
//
//        while(shouldBeRunning)
//        {
//            float currentHeading = angles.firstAngle;  //Current direction
//
//            hw.drivePowers(-power + (currentHeading - gyroTarget) / 50,
//
//                    -power - (currentHeading - gyroTarget) / 50);
//        }
//
//        hw.stopDrive();
//
//    }
//
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
            hw.drivePowers(speed, -speed);

        }
        else
        {
            hw.drivePowers(-speed,speed);
        }

        while ((runtime.seconds() < timeoutS) &&
                (Math.abs(heading - targetangle) > 5))
        {
            heading = Heading();
        }

        // Stop all motion;
        hw.stopDrive();
        hw.pause(250);
    }
}
