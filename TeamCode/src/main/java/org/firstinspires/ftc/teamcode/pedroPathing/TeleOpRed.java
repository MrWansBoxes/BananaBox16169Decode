package org.firstinspires.ftc.teamcode.pedroPathing;


import static android.os.SystemClock.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.tuningAndConstants.Constants;

import java.util.function.Supplier;


@TeleOp
@Configurable
public class TeleOpRed extends OpMode {

    ColorSensor bench = new ColorSensor();   // gets the color sensor class
    ColorSensor.DetectedColor detectedColor;
    private Follower follower;
    public static Pose startingPose;    //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;  // we don't use this

    private Servo flip1, lift1, lift2, light;  // servos
    private DcMotorEx launcher1, launcher2, intake;  // DcMotors

    int intakeflag = 0;   // these are the flags
    int launchflag = 0;
    int parkflag = 0;
    int limeflag = 0;



public double intakeVelocity = 3000;
public double outtakeVelocity = -3000;
    public double highVelocity = 2400;
    public double lowVelocity = 1700;
    double curTargetVelocity = highVelocity;

    private double launcherPowerFar1 = 0.88;
    private double launcherPowerFar2 = -0.88;
    private int launcherOff = 0;
    private double launcherPowerClose1 = 0.68;
    private double launcherPowerClose2 = -0.68;
    private int intakeOn = 1;
    private int intakeOff = 0;
    private int intakeReverse = -1;
    private double flickUp = 0.86;
    private double flickDown = 0.5;
    private double lightGreen = 0.5;
    private double lightPurple = 0.722;
    private int lightOff = 0;
    private int liftUp = 1;
    private int liftDown = 0;

    double endGameStart;
    boolean isEndGame = false;
    double trackTimer;


    private DcMotorEx turret;    // turret
    private Limelight3A limelight;  // limelight

    // --- PID constants ---
    public static double P = 0.04;    // these are the PID controls for the turret and limelight
    public static double I = 0.000000001;
    public static double D = 0.05;

    static double F = 12.8;
    static double P2 = 30;

    private double integral = 0;
    private double lastError = 0;

    @Override
    public void init() {

        bench.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose(111.68583450210379, 135.51753155680223, 90) : startingPose);   // set where the robot starts in TeleOp
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(83.815, 83.209))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(65), 0.8))
                .build();

        flip1 = hardwareMap.get(Servo.class,"flip1");  // all the hardware maps for the servos, DcMotors, limelight, and turret
        lift1 = hardwareMap.get(Servo.class,"lift1");
        lift2 = hardwareMap.get(Servo.class,"lift2");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        light = hardwareMap.get(Servo.class,"light");


        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        launcher1.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher2.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P2,0,0,F);
        launcher1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        launcher2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("Initialized");



    }

    @Override
    public void start() {
        limelight.start();   // starts the limelight
        follower.startTeleopDrive();  // starts the driving
        limelight.pipelineSwitch(0);  // pipeline 1 is for blue tracking
        endGameStart = getRuntime() + 103;
        trackTimer = getRuntime() + 15;
    }

    @Override
    public void loop() {

        double curVelocity = launcher1.getVelocity();
        double error2 = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error2);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P2,0,0,F);
        launcher1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        launcher2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);



        telemetry.addData("Y", follower.getPose().getY());
        if (trackTimer <= getRuntime()) {
            gamepad2.rumbleBlips(1);
            trackTimer = getRuntime() + 15;
        }
        telemetry.addData("Runtime", getRuntime());
        if (endGameStart <= getRuntime() && !isEndGame) {
            gamepad1.rumble(5000);
            gamepad2.rumble(5000);
            isEndGame = true;
        }
        detectedColor = bench.getDetectedColor(telemetry);
        telemetry.addData("Detected Color", detectedColor);
        //Call this once per loop
        flip1.setPosition(flickDown);
        if (detectedColor == ColorSensor.DetectedColor.GREEN){
            light.setPosition(lightGreen);
        }
        else if (detectedColor == ColorSensor.DetectedColor.PURPLE){
            light.setPosition(lightPurple);
        }
        else if(detectedColor == ColorSensor.DetectedColor.UNKNOWN){
            light.setPosition(lightOff);
        }

        follower.update();
        telemetryM.update();
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }
        //Automated PathFollowing
          if (gamepad1.dpadLeftWasPressed()) {
               follower.followPath(pathChain.get());
              automatedDrive = true;
          }
        //Stop automated following if the follower is done
             if (automatedDrive && (gamepad1.dpadRightWasPressed() || !follower.isBusy())) {
               follower.startTeleopDrive();
               automatedDrive = false;
          }
        //Slow Mode
      //  if (gamepad1.rightBumperWasPressed()) {
      //      slowMode = !slowMode;
      //  }
        //Optional way to change slow mode strength
        //      if (gamepad1.xWasPressed()) {
        //         slowModeMultiplier += 0.25;
        //     }
        //Optional way to change slow mode strength
        //     if (gamepad2.yWasPressed()) {
        //         slowModeMultiplier -= 0.25;
        //     }


        if (gamepad1.yWasPressed()) {
                    flip1.setPosition(flickUp);
                    sleep(200);
                    flip1.setPosition(flickDown);
                    gamepad1.rumbleBlips(1);
        }

        if (gamepad1.aWasPressed()){
            if (intakeflag == 0){
                intake.setVelocity(intakeVelocity);
                intakeflag = 1;
            }
            else if (intakeflag == 1) {
                intake.setPower(intakeOff);
                intakeflag = 0;
            }
            else if (intakeflag == -1){
                intake.setPower(intakeOff);
                intakeflag = 0;
            }
        }

        if (gamepad1.bWasPressed()) {
            if (intakeflag == 0){
                intake.setVelocity(outtakeVelocity);
                intakeflag = -1;
            }
            else if(intakeflag == -1){
                intake.setPower(intakeOff);
                intakeflag = 0;
            }
            else if(intakeflag == 1){
                intake.setPower(intakeOff);
                intakeflag = 0;
            }
        }


        if (gamepad1.dpadUpWasPressed()) {
            if (launchflag == 0) {
                curTargetVelocity = highVelocity;
                launcher1.setVelocity(curTargetVelocity);
                launcher2.setVelocity(curTargetVelocity);
                launchflag = 1;
                limeflag = 1;
            }
            else if (launchflag == 1) {
                launcher1.setPower(launcherOff);
                launcher2.setPower(launcherOff);
                launchflag = 0;
                limeflag = 0;
            }
        }
        if (gamepad1.dpadDownWasPressed()) {
                if (launchflag == 0) {
                    curTargetVelocity = lowVelocity;
                        launcher1.setVelocity(curTargetVelocity);
                        launcher2.setVelocity(curTargetVelocity);
                        launchflag = 1;
                        limeflag = 1;
                } else if (launchflag == 1) {
                    launcher1.setPower(launcherOff);
                    launcher2.setPower(launcherOff);
                    launchflag = 0;
                    limeflag = 0;
                }
        }

        if (gamepad2.yWasPressed()) {
            if (parkflag == 0){
                lift2.setPosition(liftUp);
                lift1.setPosition(liftUp);
                parkflag = 1;
            }
            else if (parkflag == 1){
                lift1.setPosition(liftDown);
                lift2.setPosition(liftDown);
                parkflag = 0;
            }
        }



        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);


        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            if (limeflag == 1) {
                // Error is just tx straight from Limelight
                double error = result.getTx();

                // Basic PID
                integral += error;
                double derivative = error - lastError;

                double power = P * error + I * integral + D * derivative;


                turret.setPower(power);

                lastError = error;

                telemetry.addData("tx", error);
                telemetry.addData("power", power);
            }
        } else {
            // No target -> stop motor
            turret.setPower(0);
            telemetry.addLine("No Target");
        }

        telemetry.update();


    }
}
