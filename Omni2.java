package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.GyroSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Robot: Omni2", group="Robot")

public class Omni2 extends LinearOpMode {

    // Declare OpMode members for each of the motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontL = null;
    private DcMotor backL = null;
    private DcMotor frontR = null;
    private DcMotor backR = null;
    private DcMotorEx launch = null;
    private Servo DropIt = null;
    private Servo SendIt = null;
    private Servo PushIt = null; 
    
    private ElapsedTime launchTimer = new ElapsedTime();
    private boolean dropTriggered = false;
    private boolean sendTriggered = false;

    @Override
    public void runOpMode() {
        // Initialize hardware components
        frontL  = hardwareMap.get(DcMotor.class, "flm");
        backL  = hardwareMap.get(DcMotor.class, "blm");
        frontR = hardwareMap.get(DcMotor.class, "frm");
        backR = hardwareMap.get(DcMotor.class, "brm");
        launch = hardwareMap.get(DcMotorEx.class, "launch");
        DropIt = hardwareMap.get(Servo.class, "dropIt");
        SendIt = hardwareMap.get(Servo.class, "sendIt");
        PushIt = hardwareMap.get(Servo.class, "pushIt");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        
        // Initialize motors and servos
        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DropIt.setPosition(0.00);
        PushIt.setPosition(0.20);
        SendIt.setPosition(0.00);
                
        frontL.setDirection(DcMotor.Direction.REVERSE);
        backL.setDirection(DcMotor.Direction.REVERSE);
        frontR.setDirection(DcMotor.Direction.FORWARD);
        backR.setDirection(DcMotor.Direction.FORWARD);
        launch.setDirection(DcMotor.Direction.REVERSE);

        frontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize IMU
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // Wait for start signal
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        
        int speeddiv = 1;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            // Getting robot orientation and joystick values
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);            
            double axial    = -gamepad1.left_stick_y;
            double lateral  =  gamepad1.left_stick_x;
            double yaw      =  gamepad1.right_stick_x * 0.7;
            double lateralx = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
            double axialy   = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);
            double denominator = Math.max(Math.abs(axialy) + Math.abs(lateralx) + Math.abs(yaw), 1);
                            
            double leftFrontPower = (axialy + lateralx + yaw) / denominator;
            double leftBackPower = (axialy - lateralx + yaw) / denominator;
            double rightFrontPower = (axialy - lateralx - yaw) / denominator;
            double rightBackPower = (axialy + lateralx - yaw) / denominator;
                
            lateralx = lateralx * 1.1;

            if (gamepad1.options && opModeIsActive()) {
                imu.resetYaw();
            }
            
            // Shooter logic
            if (gamepad2.x && opModeIsActive()) {
                launch.setVelocity(1940);
            }
            
            if (gamepad2.b && opModeIsActive()) {
                launch.setVelocity(1600);
            }
            
            if (gamepad2.y && opModeIsActive()) {
                launch.setVelocity(0);
            }
            
            // Pushit logic
            if (gamepad2.a && opModeIsActive()) {
                PushIt.setPosition(0.00);
                sleep(300);
                PushIt.setPosition(0.20);
            }
            
            // DropIt logic with timing
            if (gamepad2.dpad_down && gamepad1.left_bumper && opModeIsActive()) {
                if (!dropTriggered) {
                    DropIt.setPosition(0.20);
                    launchTimer.reset();
                    dropTriggered = true;
                    sleep(500);
          //      }
        //        if (dropTriggered && launchTimer.milliseconds() > 600) {
                    DropIt.setPosition(0.05);
                    dropTriggered = false;
                }
            }
            
            // SendIt logic with timing
            if (gamepad2.dpad_up && gamepad1.left_bumper && opModeIsActive()) {
                if (!sendTriggered) {
                    SendIt.setPosition(0.20);
                    launchTimer.reset();
                    sendTriggered = true;
                    sleep(500);

            //    }
              //  if (sendTriggered && launchTimer.milliseconds() > 600) {
                    SendIt.setPosition(0.00);
                    sendTriggered = false;
                }
            }

            // Speed divider
            if (gamepad1.right_bumper && opModeIsActive()) {
                speeddiv = 4;
            }
            else {
                speeddiv = 1;
            }

            // Send calculated power to wheels
            frontL.setPower(leftFrontPower / speeddiv);
            frontR.setPower(rightFrontPower / speeddiv);
            backL.setPower(leftBackPower / speeddiv);
            backR.setPower(rightBackPower / speeddiv);
            
            // Telemetry update
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Axial", "%4.2f", axial);
            telemetry.addData("Lateral", "%4.2f", lateral);
            telemetry.addData("Yaw", "%4.2f", yaw);
            telemetry.addData("Launch speed", launch.getVelocity());
            telemetry.update();
        }
    }
}