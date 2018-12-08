//// code created by Philip I. Connor
//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//
///**
// * This code is built based upon the FTC team 9886 robot design which includes mecanum
// * wheels for driving.
// * The code uses the Hardware definition for the robot.
// * The code is structured as a LinearOpMode.
// */
//
//@Autonomous(name="blue_opposite", group="Mecanum")
////@Disabled
//public class blue_opposite extends LinearOpMode {
//
//    /* Declare OpMode members. */
//    Hardware                robot   = new Hardware();
//
//    public static final String TAG = "Vuforia VuMark Sample";
//    OpenGLMatrix lastLocation = null;
//    VuforiaLocalizer vuforia;
//
//    BNO055IMU imu;
//    Orientation angles;
//    Acceleration gravity;
//
//    @Override
//    public void runOpMode() {
//
//        robot.init(hardwareMap);
//
//        telemetry.addData("Status", "Resetting Encoders");
//        telemetry.update();
//
//        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        robot.left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        telemetry.addData("Path0", "Starting at %7d : %7d : %7d : %7d",
//                robot.left_front_drive.getCurrentPosition(),
//                robot.right_front_drive.getCurrentPosition(),
//                robot.left_back_drive.getCurrentPosition(),
//                robot.right_back_drive.getCurrentPosition())
//        ;
//
//        // *****************************************************************************
//        //
//        // VUFORIA
//        //
//        // *****************************************************************************
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        parameters.vuforiaLicenseKey = "AQhA50P/////AAAAGdplbAI6JkMjne2oVB4nJQwitGAjotV1w0OlU4E/zyJbZTcKEPxqOXlS/4AFlYaXAEAMeivPZ2jqb5n6T+Gph26xlmEIvK4YrXMJB+TmR2LNyF5jzYAWb0B2F8jDh/PhTJ1XMHjT3jI3jf1PeIYcIxzcdsL3c29iVrvpeN3Aae2LEJWww/f+oGYC+okZlX5qULXeIugMknV38LrnBygzq9XJCbxRXeT37joY+9qLWLNMfzV23QF+wz/pTHLfVsu+s3EGB6Az1W5Y2EPyWFRYtBdNZKft1GsRc+HWYgW0NAxSFMUX0hlju7BueOQ6zPbPz6zgqwFjtS44rwI2H36XLrYsvsB2KT2ew6QFkKqGUJof";
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//        VuforiaTrackable relicTemplate = relicTrackables.get(0);
//        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
//        // *****************************************************************************
//
//
//        // *****************************************************************************
//        //
//        // IMU
//        //
//        // *****************************************************************************
//        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
//        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters1.loggingEnabled = true;
//        parameters1.loggingTag = "IMU";
//        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters1);
//        // *****************************************************************************
//
//        telemetry.update();
//
//        waitForStart();
//
//        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
//
//        // close the grippers to grab the glyph.
//        //robot.low_left.setPosition  (0.42f);
//        //robot.low_right.setPosition (0.47f);
//        robot.low_left.setPosition  (0.35f);
//        robot.low_right.setPosition (0.47f);
//        robot.high_left.setPosition (0.51f);
//        robot.high_right.setPosition(0.40f);
//
//        // Take a Vuforia picture to determine where to put the Glyph
//        relicTrackables.activate();
//        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
//
//        int i = 0;
//        while (i <= 10) {
//            sleep(100);
//            i = i + 1;
//
//            vuMark = RelicRecoveryVuMark.from(relicTemplate);
//            if (vuMark != RelicRecoveryVuMark.UNKNOWN)
//            {
//                telemetry.addData("VuMark", "%s visible", vuMark);
//                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
//                telemetry.addData("Pose", format(pose));
//                i = 11;
//            }
//            else
//            {
//                telemetry.addData("VuMark", "not visible");
//            }
//        }
//
//        // Extend the color sensor arm to a position to read the Jewel color
//        robot.pole_servo.setPosition(0.25f);
//        sleep(1000);
//        robot.pole_servo.setPosition(0.15f);
//
//        // lift the glyph 4 in. so that it will not drag or get in the way.
//        robot.low_lift.setPower(0.55);
//        while (robot.low_lift.getCurrentPosition() <= 200);
//        {
//
//        }
//        robot.low_lift.setPower(0.0);
//        /*robot.low_lift.setPower(0.5f);
//        sleep(1000);
//        robot.low_lift.setPower(0.0f);
//        sleep(2000);*/
//        // Determine the color of the jewel to the front of the robot
//        // This logic assumes that we want to knock off the red Jewel
//        telemetry.addData("Red", "%d", robot.sensorColor.red());
//        telemetry.addData("Blue", "%d", robot.sensorColor.blue());
//        telemetry.update();
//
//        if (robot.sensorColor.red() >= robot.sensorColor.blue())
//        {
//            // Rotate left on balancing stone to knock of the red jewel.
//            turn(30, 0.15, 2);
//
//            // Raise the color sensor arm
//            robot.pole_servo.setPosition(0.75f);
//            sleep(250);
//
//            // Rotate back on the balancing stone
//            turn(-30, 0.15, 2);
//
//            // Drive off the balancing stone and far enough away to be able to freely turn.
//            robot.drive(this, 25, 0.4f, 5);
//        }
//        else
//        {
//            //knock the jewel off by moving 3 in. forward
//            robot.drive(this, 3, 0.4f, 1);
//
//            // Raise the color sensor arm
//            robot.pole_servo.setPosition(0.90f);
//            sleep(250);
//
//            // Drive off the balancing stone and far enough away to be able to freely turn.
//            robot.drive(this, 21, 0.4f, 5);
//        }
//
//        // Turn toward the proper column and drive forward toward the column
//        if (vuMark == RelicRecoveryVuMark.LEFT)
//        {
//            turn(-22, 0.15, 5.);
//            robot.drive(this, 7, 0.4f, 3);
//        }
//        else if (vuMark == RelicRecoveryVuMark.CENTER)
//        {
//            turn(-38, 0.15, 5.);
//            robot.drive(this, 9, 0.4f, 3);
//        }
//        else if (vuMark == RelicRecoveryVuMark.RIGHT)
//        {
//            turn(-50, 0.15, 5.);
//            robot.drive(this, 13, 0.4f, 3);
//        }
//        else
//        {
//            turn(-22, 0.15, 5.);
//            robot.drive(this, 8, 0.4f, 3);
//        }
//        sleep(500);
//
//        // Release the glyph.
//        robot.low_left.setPosition  (0.44f);
//        robot.low_right.setPosition (0.39f);
//        robot.high_left.setPosition (0.43f);
//        robot.high_right.setPosition(0.48f);
//        sleep(1000);
//
//        // back up so that the robot does not touch the glyph.
//        robot.drive(this, -6, 0.4f, 2);
//
//        robot.low_left.setPosition    (0.88f);
//        robot.low_right.setPosition   (0.00f);
//        robot.high_left.setPosition   (0.00f);
//        robot.high_right.setPosition  (0.88f);
//
//        //
//        turn(-180, 0.15, 7.);
//        robot.drive(this, -8, 0.4f, 3);
//        robot.drive(this, 4, 0.4f, 3);
//
//        sleep(250);
//    }
//
//    void turn(float anglechange, double speed, double timeoutS)
//    {
//        float targetangle;
//        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        float heading = angles.firstAngle;
//        targetangle = heading + anglechange;
//        if (targetangle > 180) targetangle = targetangle -360;
//        if (targetangle < -180) targetangle = targetangle +360;
//
//        ElapsedTime runtime = new ElapsedTime();
//        runtime.reset();
//
//        if (anglechange < 0)
//        {
//            robot.left_front_drive.setPower(speed);
//            robot.right_front_drive.setPower(-speed);
//            robot.left_back_drive.setPower(speed);
//            robot.right_back_drive.setPower(-speed);
//        }
//        else
//        {
//            robot.left_front_drive.setPower(-speed);
//            robot.right_front_drive.setPower(speed);
//            robot.left_back_drive.setPower(-speed);
//            robot.right_back_drive.setPower(speed);
//        }
//
//        while ((runtime.seconds() < timeoutS) &&
//                (Math.abs(heading - targetangle) > 5))
//        {
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            heading = angles.firstAngle;
//        }
//
//        // Stop all motion;
//        robot.left_front_drive.setPower(0);
//        robot.right_front_drive.setPower(0);
//        robot.left_back_drive.setPower(0);
//        robot.right_back_drive.setPower(0);
//        sleep(250);
//    }
//
//    String format(OpenGLMatrix transformationMatrix) {
//        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
//    }
//}
