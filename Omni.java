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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Robot: Omni", group="Robot")

public class Omni extends LinearOpMode {

    // Declare OpMode members for each of the motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontL = null;
    private DcMotor backL = null;
    private DcMotor frontR = null;
    private DcMotor backR = null;
    private DcMotor launch = null;
    private Servo DropIt = null;
    private Servo SendIt = null;
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontL  = hardwareMap.get(DcMotor.class, "flm");
        backL  = hardwareMap.get(DcMotor.class, "blm");
        frontR = hardwareMap.get(DcMotor.class, "frm");
        backR = hardwareMap.get(DcMotor.class, "brm");
        launch = hardwareMap.get(DcMotor.class, "launch");
        DropIt = hardwareMap.get(Servo.class, "dropIt");
        SendIt = hardwareMap.get(Servo.class, "sendIt");
        IMU imu = hardwareMap.get(IMU.class, "imu");
                
        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DropIt.setPosition(0);
        SendIt.setPosition(0);
                
        frontL.setDirection(DcMotor.Direction.REVERSE);
        backL.setDirection(DcMotor.Direction.REVERSE);
        frontR.setDirection(DcMotor.Direction.FORWARD);
        backR.setDirection(DcMotor.Direction.FORWARD);
        launch.setDirection(DcMotor.Direction.REVERSE);
        
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        
        int speeddiv = 1;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);            
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial    = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
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

            if (gamepad1.options) {
                imu.resetYaw();
            }
            
            if (gamepad2.x) {
                launch.setPower(0.73);
            }
            
            if (gamepad2.b) {
                launch.setPower(0.60);
            }
            
            if (gamepad2.y) {
                launch.setPower(0.00);
            }
             
            if (gamepad2.dpad_right && gamepad1.left_bumper) {
                launch.setPower(0.64);
                sleep(250);
                SendIt.setPosition(0.20);
                sleep(600);
                SendIt.setPosition(0.00);
                sleep(100);
                launch.setPower(0.60);
                
            }
                     
            if (gamepad2.dpad_down && gamepad1.left_bumper) {
                DropIt.setPosition(0.20);
                sleep(600);
                DropIt.setPosition(0.00);
            }

            if (gamepad2.dpad_up && gamepad1.left_bumper) {
                launch.setPower(0.77);
                sleep(250);
                SendIt.setPosition(0.20);
                sleep(600);
                SendIt.setPosition(0.00);
                sleep(100);
                launch.setPower(0.70);
                
            }

            if (gamepad1.right_bumper) {
                speeddiv = 4;
            }
            else {
                speeddiv = 1;
            }

            // Send calculated power to wheels
            //launch.setPower(launchspeed);
            frontL.setPower(leftFrontPower / speeddiv);
            frontR.setPower(rightFrontPower / speeddiv);
            backL.setPower(leftBackPower / speeddiv);
            backR.setPower(rightBackPower / speeddiv);
            
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Axial  ", "%4.2f", axial);
            telemetry.addData("lateral  ", "%4.2f", lateral);
            telemetry.addData("yaw  ", "%4.2f", yaw);
            telemetry.addData("launchspeed", launch.getPower());
            telemetry.addData("launch?", gamepad1.left_bumper);
            telemetry.update();
        }
    }}