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
////
//// import org.firstinspires.ftc.robotcore.external.navigation.RoverRuckusVuMark;
////
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
//@Autonomous(name="vision_test", group="Mecanum")
////@Disabled
//public class vision_test extends LinearOpMode {
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
//        //
//        //        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//        //        VuforiaTrackable relicTemplate = relicTrackables.get(0);
//        //        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
//        //
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
