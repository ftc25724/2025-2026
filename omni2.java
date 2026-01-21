package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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

import java.util.List;

@TeleOp(name="Robot: Omni2", group="Robot")

public class omni2 extends LinearOpMode {

    // Declare OpMode members for each of the motors.
    private ElapsedTime runtime = new ElapsedTime();

    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        //Motors
        DcMotor frontR = hardwareMap.get(DcMotor.class, "frm"); // ch 0
        DcMotor frontL = hardwareMap.get(DcMotor.class, "flm"); // ch 1
        DcMotor backR = hardwareMap.get(DcMotor.class, "brm"); // ch 2
        DcMotor backL = hardwareMap.get(DcMotor.class, "blm"); // ch 3
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake"); // eh 0
        DcMotorEx launch1 = hardwareMap.get(DcMotorEx.class, "launch1"); // eh 1
        DcMotorEx launch2 = hardwareMap.get(DcMotorEx.class, "launch2"); // eh 2
        DcMotor turret = hardwareMap.get(DcMotor.class, "turret"); // eh 3

        //Servos
        CRServo indexer = hardwareMap.get(CRServo.class, "indexer"); // ch 0
        Servo deindexer = hardwareMap.get(Servo.class, "deindexer"); // ch 1

        //Sensors
        limelight = hardwareMap.get(Limelight3A.class, "limelight"); // ch usb 3
        IMU imu = hardwareMap.get(IMU.class, "imu"); // ch built in
        DistanceSensor loaded = hardwareMap.get(DistanceSensor.class, "distance"); // ch 12C Bus 0
        DigitalChannel launchsens = hardwareMap.get(DigitalChannel.class, "launch sensor"); // ch Digital 0-1
        DigitalChannel intakesens = hardwareMap.get(DigitalChannel.class, "intake sensor"); // ch Digital 2-3
        ColorSensor colorSens = hardwareMap.get(ColorSensor.class, "color sensor"); // ch i2c Bus 2

        // Initialize motors and servos
        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launch1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launch2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontL.setDirection(DcMotor.Direction.REVERSE);
        backL.setDirection(DcMotor.Direction.REVERSE);
        frontR.setDirection(DcMotor.Direction.FORWARD);
        backR.setDirection(DcMotor.Direction.FORWARD);

        launch1.setDirection(DcMotor.Direction.FORWARD);
        launch2.setDirection(DcMotor.Direction.REVERSE);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        frontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launch1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launch2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        deindexer.setPosition(0.00);

        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        int inorout = 0;
        int speeddiv = 1;
        int intakespeed = 0;
        int launch1speed = 0;
        int launch2speed = 0;
        int combo = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            // Getting robot orientation and joystick values
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

            // reset IMU
            if (gamepad1.options && opModeIsActive()) {
                imu.resetYaw();
            }

            // Speed divider
            if (gamepad1.right_bumper && opModeIsActive()) {
                speeddiv = 4;
            }
            else {
                speeddiv = 1;
            }

            // inroout toggle
            if (gamepad1.a && inorout == 1 && opModeIsActive()) {
                inorout = 0; // in
            } else if (gamepad1.a && inorout == 0 && opModeIsActive()) {
                inorout = 1; // out
            }

            // intake or output
            switch (inorout) {

                case 0: // in

                    // intake
                    if (gamepad2.dpad_up && intakesens.getState() && intakespeed != 1 && opModeIsActive()) {
                        intakespeed = 1;
                        sleep(250);
                    } else if ((gamepad2.dpad_up || gamepad2.dpad_down)  && intakespeed != 0 && opModeIsActive()) {
                        intakespeed = 0;
                        sleep(250);
                    } else if (gamepad2.dpad_down && intakesens.getState() && opModeIsActive()) {
                        intakespeed = -1;
                        sleep(250);
                    }

                    // index in
                    if (gamepad2.y && opModeIsActive()) {
                        do {
                            indexer.setPower(0.09);
                        } while (!intakesens.getState());
                    }
                    break;

                case 1: // out
                    // index out
                    if (gamepad2.y && opModeIsActive()) {
                        do {
                            indexer.setPower(0.07);
                        } while (!launchsens.getState());
                    }

                    // deindex
                    if (gamepad2.a && !launchsens.getState() && opModeIsActive()) {
                        deindexer.setPosition(0.70);
                        sleep(300);
                        deindexer.setPosition(0.00);
                    }

                    // launch
                    if (gamepad2.right_bumper /*&& launchspeed != 1 */&& opModeIsActive()) {
                        launch1speed = 2000;
                        launch2speed = 1700;
                        sleep(250);
                    } else if (gamepad2.left_bumper /*&& intakespeed != 0*/ && opModeIsActive()) {
                        launch1speed = 0;
                        launch2speed = 0;
                        sleep(250);
                    }
                    break;

                default:
                    break;
            }

            // camera
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {

                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    double x = botpose.getPosition().x;
                    double y = botpose.getPosition().y;
                    telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
                }

                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                if (!tags.isEmpty()) {
                    for(LLResultTypes.FiducialResult tag : tags){
                        combo = tag.getFiducialId();
                        telemetry.addData("Target ID", combo);
                    }
                }
                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            // AprilTag detector
            switch (combo) {

                case 20:
                    telemetry.addData("alliance", "BLUE");
                    break;

                case 21:
                    telemetry.addData("combo", "G-P-P");
                    break;

                case 22:
                    telemetry.addData("combo", "P-G-P");
                    break;

                case 23:
                    telemetry.addData("combo", "P-P-G");
                    break;

                case 24:
                    telemetry.addData("alliance", "RED");
                    break;

                default:
                    break;
            }

            // Send calculated power to wheels
            frontL.setPower(leftFrontPower / speeddiv);
            frontR.setPower(rightFrontPower / speeddiv);
            backL.setPower(leftBackPower / speeddiv);
            backR.setPower(rightBackPower / speeddiv);
            intake.setPower(intakespeed);
            launch1.setVelocity(launch1speed);
            launch2.setVelocity(launch2speed);
            
            // Telemetry update
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Axial  ", "%4.2f", axial);
            telemetry.addData("lateral  ", "%4.2f", lateral);
            telemetry.addData("yaw  ", "%4.2f", yaw);
            telemetry.addData("loaded?", loaded.getDistance(DistanceUnit.MM));
            telemetry.addData("intake", intakesens.getState());
            telemetry.addData("output", launchsens.getState());
            telemetry.addData("inorout", inorout);
            telemetry.addData("launch1Velocity", launch1.getVelocity());
            telemetry.addData("launch2Velocity", launch2.getVelocity());
            telemetry.update();
        }
    }}
