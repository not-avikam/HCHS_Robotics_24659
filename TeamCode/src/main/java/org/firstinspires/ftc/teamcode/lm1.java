package org.firstinspires.ftc.teamcode;

//import statements
import android.app.Activity;
import android.view.View;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@TeleOp(name="who", group="opps")
public class lm1 extends LinearOpMode {

    View relativeLayout;

    IMU imu;

    // Declare OpMode members
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    private DcMotorEx hang1 = null;
    private DcMotorEx hang2 = null;
    private DcMotorEx intakeRotate = null;
    private CRServo intake = null;
    private Servo wrist = null;
    private SparkFunOTOS otos = null;
    private Follower follower;

    @Override
    public void runOpMode() {

        //hang1 = hardwareMap.get(DcMotorEx.class, "hang1");
        //hang2 = hardwareMap.get(DcMotorEx.class, "hang2");
        //hang1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //hang2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //hang1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //hang2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
/*
        if (gamepad1.dpad_up) {
            hang1.setPower(1);
            hang2.setPower(1);
        } else if (gamepad1.dpad_down) {
            hang1.setPower(-1);
            hang2.setPower(-1);
        } else {
            hang1.setPower(0);
            hang2.setPower(0);
        }

 */




        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // Initialize the hardware.
        //The strings used here (in green) must match the robot configuration on the Driver Hub.
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightRear");
        intake = hardwareMap.get(CRServo.class, "intake");
        intakeRotate = hardwareMap.get(DcMotorEx.class, "intakeRotate");
        imu = hardwareMap.get(IMU.class, "imu");
        wrist = hardwareMap.get(Servo.class, "wrist");
        otos = hardwareMap.get(SparkFunOTOS.class, "avikams_son");

        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeRotate.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //follower.startTeleopDrive();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            System.out.println("Credits: Design~Alexander Lorenzo; Code~Avikam Bali; Hardware: Dane Bluhm; and the beautiful members of 24659");

            //Initialize

            //DC motors

            /*
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            follower.update();
            */


            double max;

            /*
             * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
             * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
             * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
             */

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.

            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            double rotateIn = gamepad1.right_trigger;
            double rotateOut = -gamepad1.left_trigger;

            //Combine the joystick and trigger requests for each axis-motion to determine each power value.
            //Set up a variable for each motor to save the power level for telemetry.


            double leftFrontPower  = axial - lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial + lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            double rotatePower = rotateIn - rotateOut;

            /*Normalize the values so no wheel power exceeds 100%
            This ensures that the robot maintains the desired motion.*/

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            max = Math.max(max, Math.abs(rotatePower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
                rotatePower /= max;
            }

            // Send calculated power to wheels


            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            intakeRotate.setPower(rotatePower);

            //Servos
            if (gamepad1.dpad_right) {
                intake.setPower(1);
            } else if (gamepad1.dpad_left) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.a) {
                wrist.setPosition(.5);
            } else if (gamepad1.b) {
                wrist.setPosition(0);
            }

            //IMU orientation

            imu.resetYaw();
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            //Telemetry
            telemetry.addLine("Are you sure the robot was flat on the ground during initialization? If not, reinitialize");
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream\n");

            telemetry.update();
        }
    }}