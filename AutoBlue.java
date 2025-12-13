 package org.firstinspires.ftc.teamcode;
 
 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
 import com.qualcomm.robotcore.hardware.IMU;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.eventloop.opmode.Disabled;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.DcMotorEx;
 import com.qualcomm.robotcore.util.ElapsedTime;
 
 @Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")
 
 public class AutoBlue extends LinearOpMode {
 
     /* Declare OpMode members. */
    private DcMotor frontL = null;
    private DcMotor backL = null;
    private DcMotor frontR = null;
    private DcMotor backR = null;
    private DcMotorEx launch = null;
    private Servo DropIt = null;
    private Servo SendIt = null;
    private Servo PushIt = null; 
    private ElapsedTime     runtime = new ElapsedTime();
 
     static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
     static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
     static final double     WHEEL_DIAMETER_INCHES   = 4.09449 ;     // For figuring circumference
     static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                       (WHEEL_DIAMETER_INCHES * 3.1415);
     static final double     DRIVE_SPEED             = 0.6;
     static final double     TURN_SPEED              = 0.5;
 
     @Override
     public void runOpMode() {
 
         // Initialize the drive system variables.
        frontL  = hardwareMap.get(DcMotor.class, "flm");
        backL  = hardwareMap.get(DcMotor.class, "blm");
        frontR = hardwareMap.get(DcMotor.class, "frm");
        backR = hardwareMap.get(DcMotor.class, "brm");
        launch = hardwareMap.get(DcMotorEx.class, "launch");
        DropIt = hardwareMap.get(Servo.class, "dropIt");
        SendIt = hardwareMap.get(Servo.class, "sendIt");
        PushIt = hardwareMap.get(Servo.class, "pushIt");
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

         frontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         backL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         frontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         backR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         frontL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         backL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         frontR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         backR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


         //telemetry.addData("Starting at",  "%4.2f, %4.2f, %4.2f, %4.2f",
                           //frontL.getCurrentPosition(),
                           //backL.getCurrentPosition(),
                           //frontR.getCurrentPosition(),
                           //backR.getCurrentPosition();
        //  telemetry.addData("Starting at front left/Right", "%4.2f, %4.2f", frontL.getCurrentPosition(), frontR.getCurrent            telemetry.addData("slide", slider.getCurrentPosition());
        //  telemetry.addData("Starting at back  left/Right", "%4.2f, %4.2f", backL.getCurrentPosition(), backR.getCurrentPosition() );

        telemetry.update();
        waitForStart();

        launch.setVelocity(1940);

        while (launch.getVelocity() < 1940) {
            //wait
        }
        
        sleep(500);


        DropIt.setPosition(0.20);
        sleep(500);
        DropIt.setPosition(0.05);
        
        sleep(150);

        PushIt.setPosition(0.00);
        sleep(300);
        PushIt.setPosition(0.20);
        
        sleep(150);
        
        DropIt.setPosition(0.20);
        sleep(1000);
        DropIt.setPosition(0.05);

        sleep(1000);

        launch.setVelocity(2050);

        while (launch.getVelocity() < 2050) {
            //wait
        }
        
        sleep(500);


        SendIt.setPosition(0.20);
        sleep(600);
        SendIt.setPosition(0.00);
        sleep(500);
        
        launch.setVelocity(0);
            
        

            
         // Wait for the game to start (driver presses START)
         // Step through each leg of the path,
         // Note: Reverse movement is obtained by setting a negative distance (not speed)
         // encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
         // encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
         // encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
 
         //telemetry.addData("Path", "Complete");
         //telemetry.update();
         sleep(1000);  // pause to display final telemetry message.
     
         int newLeftTarget;
         int newRightTarget;
 
         // Ensure that the OpMode is still active
         if (opModeIsActive()) {
          
          
          
          
 
             // Determine new target position, and pass to motor controller
             //newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
             //newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
             //leftDrive.setTargetPosition(newLeftTarget);
             //rightDrive.setTargetPosition(newRightTarget);

             // Turn On RUN_TO_POSITION
             //leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             //rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             
             // reset the timeout time and start motion.
             //runtime.reset();
             //leftDrive.setPower(Math.abs(speed));
             //rightDrive.setPower(Math.abs(speed));
             
 
             // keep looping while we are still active, and there is time left, and both motors are running.
             // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
             // its target position, the motion will stop.  This is "safer" in the event that the robot will
             // always end the motion as soon as possible.
             // However, if you require that BOTH motors have finished their moves before the robot continues
             // onto the next step, use (isBusy() || isBusy()) in the loop test.
             //while (opModeIsActive() &&
                    //(runtime.seconds() < timeoutS) &&
                    //(leftDrive.isBusy() && rightDrive.isBusy())) {
 
                 // Display it for the driver.
                 //telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                 //telemetry.addData("front left/Right", "%4.2f, %4.2f", frontL.getCurrentPosition(), frontR.getCurrentPosition());
                 //telemetry.addData("back  left/Right", "%4.2f, %4.2f", backL.getCurrentPosition(), backR.getCurrentPosition());
                 //telemetry.addData("slide", slider.getCurrentPosition());
                 //telemetry.addData("arm", armm.getCurrentPosition());
                 //telemetry.update();
             
 
             // Stop all motion;
             //leftDrive.setPower(0);
             //rightDrive.setPower(0);
 
             // Turn off RUN_TO_POSITION
             //leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             //rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
 
             sleep(250);   // optional pause after each move.
         }
//     }
}}
 