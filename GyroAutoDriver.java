package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class GyroAutoDriver {
    AutoHardware hw;

public GyroAutoDriver(AutoHardware hw){
    this.hw = hw;
    angles = hw.adafruitIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);
}

public Orientation angles;



private double Heading(){
    if(angles.secondAngle == 180)
    {
        angles.secondAngle -= 360;
    }
    return angles.secondAngle;
    }

public void driveForwards(float distance, double power){
    double target = angles.firstAngle;

    hw.rightDrivePower(power);
    hw.leftDrivePower(power);

    int checkCount = 0;

    while(hw.left_front_drive.getCurrentPosition() < hw.inchesToEncoderCounts(distance) && hw.left_back_drive.getCurrentPosition() < hw.inchesToEncoderCounts(distance) && hw.opMode.opModeIsActive())
    {
        checkCount++;
        float currentHeading = angles.firstAngle;
        hw.drivePowers(power + (currentHeading - target) / 50,
                power - (currentHeading - target) / 50);

        hw.opMode.telemetry.addData("Times checked:", checkCount);
        hw.opMode.telemetry.addData("Gyro Target:", target);
        hw.opMode.telemetry.addData("Gyro Heading:", currentHeading);
        hw.opMode.telemetry.addData("Right Encoder", hw.left_front_drive.getCurrentPosition());
        hw.opMode.telemetry.addData("Left Encoder", hw.left_back_drive.getCurrentPosition());
        hw.opMode.telemetry.update();
    }
    hw.stopDrive();

}

public void driveBackwards(float distance, double power){
    double target = angles.firstAngle;

    while(hw.left_front_drive.getCurrentPosition() > -hw.inchesToEncoderCounts(distance) && hw.left_back_drive.getCurrentPosition() > -hw.inchesToEncoderCounts(distance) && hw.opMode.opModeIsActive())
    {
        float currentHeading = angles.firstAngle;  //Current direction

        hw.drivePowers(-power + (currentHeading - target) / 50,
                -power - (currentHeading - target) / 50);

    }

    hw.stopDrive();
}

    public void turn(int target, double power)
    {
        double initialValue = angles.firstAngle;  //Starting direction

        double gyroTarget = initialValue + 90;




        if(gyroTarget > 180)
        {
            gyroTarget -= 360;
        }
        else if(gyroTarget < -180)
        {
            gyroTarget += 360;
        }

        boolean shouldBeRunning = true;

        while(shouldBeRunning)
        {
            float currentHeading = angles.firstAngle;  //Current direction

            hw.drivePowers(-power + (currentHeading - gyroTarget) / 50,

                    -power - (currentHeading - gyroTarget) / 50);
        }

        hw.stopDrive();

    }

}
