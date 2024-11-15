package org.firstinspires.ftc.teamcode;

//import statements
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.app.Activity;
import android.util.Size;
import android.view.View;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/*
A note about PedroPathing's TeleOp enhancements

Written by Avikam (24659 lead software, co-captain Centerstage, Into The Deep)

My understanding of PedroPathing may be inaccurate, but I'll do my best to explain.

PedroPathing is a "Reactive Vector Follower" made by FTC team 10158 Scott's Bots for Centerstage.

Unlike Road Runner, which calculates the robot's trajectory beforehand, PedroPathing is able to
adjust the trajectory as the robot is moving; dynamically. So, if the robot gets pushed or bumped
into, PedroPathing can compensate for that and adjust the trajectory to get back onto the correct
path. Additionally, RoadRunner can have trouble keeping track during tight curves, which PedroPathing
is better able to handle

While PedroPathing is mainly for autonomous, it provides enhancements to
TeleOperated systems as well, which is what we have used in this file.

My understanding of the enhancements is this: PedroPathing compensates
for the centripetal force using the configured localizer
when turning the robot, resulting in tighter turns.
Without correcting for centripetal force, the robot will tend to swing out during cornering.\

PedroPathing website: https://pedropathing.com/Pedro+Pathing
PedroPathing video: https://www.youtube.com/watch?v=HI7eyLLpCgM
*/

@TeleOp(name="egt", group="opps")
public class egt extends LinearOpMode {

    View relativeLayout;

    IMU imu;

    // Declare OpMode members
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx submersibleSlide = null;
    private Servo intakeRotate = null;
    private DcMotorEx armY = null;
    private CRServo lServo = null;

    private CRServo rServo = null;
    private Servo lClaw = null;
    private Servo rClaw = null;


    private Follower follower;
    private SparkFunOTOS otos;

    @Override
    public void runOpMode() {

        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        //This is used by the PedroPathing TeleOp enhancements
        follower = new Follower(hardwareMap);

        // Initialize the hardware.
        //The strings used here (in green) must match the robot configuration on the Driver Hub.
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightRear");
        submersibleSlide = hardwareMap.get(DcMotorEx.class, "submersibleSlide");
        armY = hardwareMap.get(DcMotorEx.class, "armY");
        intakeRotate = hardwareMap.get(Servo.class, "intakeRotate");
        rServo = hardwareMap.get(CRServo.class, "rServo");
        lServo = hardwareMap.get(CRServo.class, "lServo");
        rClaw = hardwareMap.get(Servo.class, "rClaw");
        lClaw = hardwareMap.get(Servo.class, "lClaw");
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        imu = hardwareMap.get(IMU.class, "imu");

        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armY.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        submersibleSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armY.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        submersibleSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        rServo.setDirection(CRServo.Direction.REVERSE);
        lClaw.setDirection(Servo.Direction.REVERSE);

        follower.startTeleopDrive();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            System.out.println("I do reckon that you should wash your hands!-avikam bali");

            //Initialize
            lClaw.setPosition(0);
            rClaw.setPosition(0);


            //This is used by the PedroPathing TeleOp enhancements
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            follower.update();

            //DC motors

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

            double verticalUp = gamepad1.right_trigger;
            double verticalDown = gamepad1.left_trigger;
            double xPower = gamepad1.right_stick_y;

            //Combine the joystick and trigger requests for each axis-motion to determine each power value.
            //Set up a variable for each motor to save the power level for telemetry.


            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;


            double armYPower = verticalUp - verticalDown;

            /*Normalize the values so no wheel power exceeds 100%
            This ensures that the robot maintains the desired motion.*/

            /*
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            */

            max = Math.abs(armYPower);
            max = Math.abs(xPower);

            if (max > 1.0) {

                /*
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
                */

                armYPower /= max;
                xPower /= max;
            }

            // Send calculated power to wheels

            /*
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            */

            armY.setPower(armYPower);
            submersibleSlide.setPower(xPower);

            //Servos

            PredominantColorProcessor.Result result = colorSensor.getAnalysis();

            if (result.closestSwatch == PredominantColorProcessor.Swatch.RED && gamepad1.a) {
                lServo.setPower(1);
                rServo.setPower(1);
                gamepad1.stopRumble();
            } else if (result.closestSwatch == PredominantColorProcessor.Swatch.YELLOW && gamepad1.a) {
                lServo.setPower(1);
                rServo.setPower(1);
            } else if (gamepad1.a) {
                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            } else {
                lServo.setPower(0);
                rServo.setPower(0);
            }

            if (gamepad1.dpad_up) {
                lClaw.setPosition(1);
                rClaw.setPosition(1);
                intakeRotate.setPosition(1);
            } else if (gamepad1.dpad_down) {
                lClaw.setPosition(0);
                rClaw.setPosition(0);
                intakeRotate.setPosition(0);
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
            telemetry.addData("Vertical slide, %4.2f", armYPower);
            telemetry.addData("Horizontal slide, %4.2f", xPower);
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream\n");

            telemetry.update();
        }
    }}