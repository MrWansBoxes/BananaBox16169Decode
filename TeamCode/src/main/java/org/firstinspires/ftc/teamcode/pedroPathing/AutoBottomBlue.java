package org.firstinspires.ftc.teamcode.pedroPathing;

import static android.os.SystemClock.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.tuningAndConstants.Constants;
@Autonomous
@Configurable
public class AutoBottomBlue extends OpMode {

    private Follower follower;

    private Servo flip1;
    private Servo light;
    private DcMotorEx intake;
    private DcMotorEx launcher1;
    private DcMotorEx launcher2;
    private DcMotor turret;
    private Limelight3A limelight;

    private int pathState;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private Timer pathTimer, launchTimer;

    private Paths paths;
    int launchStep = -1;
    boolean isDone = false;
    boolean limeFlag = true;
    public double intakeVelocity = 3000;
    public double outtakeVelocity = -3000;
    public double highVelocity = 2400;
    public double lowVelocity = 1700;
    double curTargetVelocity = highVelocity;

    private double launcherPowerFar1 = 0.82;  // Variables for tuning
    private double launcherPowerFar2 = -0.82;
    private double launcherPowerClose1 = 0.65;
    private double launcherPowerClose2 = -0.65;
    private int launcherOff = 0;
    private int intakeOn = 1;
    private int intakeOff = 0;
    private double flickUp = 0.86;
    private double flickDown = 0.5;

    public static double P = 0.04;    // these are the PID controls for the turret and limelight
    public static double I = 0.000000001;
    public static double D = 0.05;

    static double F = 12.8;
    static double P2 = 30;

    private double integral = 0;
    private double lastError = 0;


@Override
public void init(){


    pathTimer = new Timer();
    launchTimer = new Timer();
    panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(new Pose(48.269284712482474, 8.078541374474053, Math.toRadians(90))); // starting spot
    paths = new Paths(follower);

    flip1 = hardwareMap.get(Servo.class, "flip1");     // Hardware map names
    intake = hardwareMap.get(DcMotorEx.class, "intake");
    launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
    launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
    limelight = hardwareMap.get(Limelight3A.class, "limelight");
    light = hardwareMap.get(Servo.class,"light");

    turret = hardwareMap.get(DcMotorEx.class, "turret");
    turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    launcher1.setDirection(DcMotorSimple.Direction.FORWARD);
    launcher2.setDirection(DcMotorSimple.Direction.REVERSE);
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P2,0,0,F);
    launcher1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    launcher2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

    panelsTelemetry.debug("Status", "Initialized");
    panelsTelemetry.update(telemetry);
}

    public void start() {
    limelight.start();
    limelight.pipelineSwitch(1);
    flip1.setPosition(flickDown);
        launchStep = 0;
        setPathState(0);
    }
@Override
public void loop(){
    follower.update(); // Update Pedro Pathing
    pathState = autonomousPathUpdate(); // Update autonomous state machine
    double curVelocity = launcher1.getVelocity();
    double error2 = curTargetVelocity - curVelocity;
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P2,0,0,F);
    launcher1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    launcher2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    LLResult result = limelight.getLatestResult();

    if (result != null && result.isValid()) {
        if (limeFlag) {
            // Error is just tx straight from Limelight
            double error = result.getTx();

            // Basic PID
            integral += error;
            double derivative = error - lastError;

            double power = P * error + I * integral + D * derivative;


            turret.setPower(power);

            lastError = error;
        }
    } else {
        // No target -> stop motor
        turret.setPower(0);
    }

    // Log values to Panels and Driver Station
    panelsTelemetry.debug("Path State", pathState);
    panelsTelemetry.debug("X", follower.getPose().getX());
    panelsTelemetry.debug("Y", follower.getPose().getY());
    panelsTelemetry.debug("Heading", follower.getPose().getHeading());
    panelsTelemetry.update(telemetry);
}

    private void launch3balls() {  // we call this function every time you want to launch 3 balls

        switch (launchStep) {
            case -1:
                limeFlag = true;
                intake.setPower(intakeOff);
                launchTimer.resetTimer();
                launchStep++;
                break;
            case 0:
                if (launchTimer.getElapsedTimeSeconds() > 0.5) {
                    flip1.setPosition(flickUp);
                    launchTimer.resetTimer();
                    launchStep++;
                }
            break;

            case 1:
                if (launchTimer.getElapsedTimeSeconds() > 0.45) {
                    flip1.setPosition(flickDown);
                    launchTimer.resetTimer();
                    launchStep++;
            }
                break;

            case 2:
                if (launchTimer.getElapsedTimeSeconds() > 0.8) {
                    flip1.setPosition(flickUp);
                    launchTimer.resetTimer();
                    launchStep++;
                }
                break;

            case 3:
                if (launchTimer.getElapsedTimeSeconds() > 0.45) {
                    flip1.setPosition(flickDown);
                    intake.setVelocity(intakeVelocity);
                    launchTimer.resetTimer();
                    launchStep++;
                }
                break;

            case 4:
                if (launchTimer.getElapsedTimeSeconds() > 0.8) {
                    flip1.setPosition(flickUp);
                    launchTimer.resetTimer();
                    launchStep++;
                }
                break;

            case 5:
                if (launchTimer.getElapsedTimeSeconds() > 0.45) {
                    flip1.setPosition(flickDown);
                    launcher1.setPower(launcherOff);
                    launcher2.setPower(launcherOff);
                    intake.setPower(intakeOff);
                    launchTimer.resetTimer();
                    launchStep = -1;
                    limeFlag = false;
                    isDone = true;
                }
                break;
        }
    }

    /* You could check for
           - Follower State: "if(!follower.isBusy()) {}"
           - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
           - Robot Position: "if(follower.getPose().getX() > 36) {}"
           */

    private int autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                curTargetVelocity = highVelocity;
                launcher1.setVelocity(curTargetVelocity);  // set power to launcher and moves to shoot position
                launcher2.setVelocity(curTargetVelocity);
                follower.followPath(paths.Shoot1, true);
                setPathState(1);
                break;

            case 1:

                if (!follower.isBusy()) {
                    launch3balls();  // when the robot finishes the path it will launch 3 balls
                }

                if (isDone) {
                    isDone = false;
                    intake.setVelocity(intakeVelocity);
                    follower.followPath(paths.GotoBallPile1, true);
                    setPathState(2);
                }

                break;
            case 2:
                if (!follower.isBusy()) {  // when it is finished with its path the robot will intake the balls then power up the motors and turn off the intake
                    follower.followPath(paths.IntakeBallPile1, 0.6, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {  // moves to shoot position

                    intake.setPower(intakeOff);
                    launcher1.setVelocity(curTargetVelocity);
                    launcher2.setVelocity(curTargetVelocity);
                    follower.followPath(paths.Shoot2, true);
                    setPathState(4);
                }
                break;

            case 4:

                if (!follower.isBusy()) {
                    launch3balls();// when the robot finishes the path it will launch 3 balls
                }

                if (isDone) {  // after 4 seconds it will move to next path and turn on the intake
                    isDone = false;
                    intake.setVelocity(intakeVelocity);
                    follower.followPath(paths.GotoBallPile2, true);
                    setPathState(5);
                }

                break;

            case 5:
                if (!follower.isBusy()) {  // when it is finished with its path the robot will intake the balls then power up the motors and turn off the intake
                    follower.followPath(paths.IntakeBallPile2, 0.6, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {  // moves to shoot position
                    curTargetVelocity = lowVelocity;
                    intake.setPower(intakeOff);
                    launcher1.setVelocity(curTargetVelocity);
                    launcher2.setVelocity(curTargetVelocity);
                    follower.followPath(paths.Shoot3, true);
                    setPathState(7);
                }
                break;

            case 7:

                if (!follower.isBusy()) {
                    launch3balls();// when the robot finishes the path it will launch 3 balls
                }
                if (isDone) {  // after 4 seconds it will move to next path and turn on the intake
                    isDone = false;
                    intake.setVelocity(intakeVelocity);
                    follower.followPath(paths.GotoBallPile3, true);
                    setPathState(8);
                }

                break;

            case 8:
                if (!follower.isBusy()) {  // when it is finished with its path the robot will intake the balls then power up the motors and turn off the intake
                    follower.followPath(paths.IntakeBallPile3, 0.6, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {  // moves to shoot position
                    intake.setPower(intakeOff);
                    launcher1.setVelocity(curTargetVelocity);
                    launcher2.setVelocity(curTargetVelocity);
                    follower.followPath(paths.Shoot4, true);
                    setPathState(10);
                }
                break;

            case 10:

                if (!follower.isBusy()) {
                    launch3balls();// when the robot finishes the path it will launch 3 balls
                }

                if (isDone) {  // after 4 seconds it will move to next path and turn on the intake
                    follower.followPath(paths.GoPark, true);
                    setPathState(-1);
                }

                break;
        }
        return pathState;
    }

    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    // Start of all the paths
    public static class Paths {
        public PathChain Shoot1, GotoBallPile1, IntakeBallPile1,
                Shoot2, GotoBallPile2, IntakeBallPile2,
                Shoot3, GotoBallPile3, IntakeBallPile3,
                Shoot4, GoPark;

        public Paths(Follower follower) {

            Shoot1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.269, 7.877),
                                    new Pose(56.318, 12.541),
                                    new Pose(60.387, 17.975)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(116))

                    .build();

            GotoBallPile1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(60.387, 17.975),
                                    new Pose(55.338, 36.151),
                                    new Pose(47.230, 34.940)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(116), Math.toRadians(180))

                    .build();

            IntakeBallPile1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47.230, 34.940),

                                    new Pose(12.118, 34.738)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Shoot2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(12.118, 34.738),
                                    new Pose(52.511, 40.999),
                                    new Pose(60.387, 17.975)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(117))

                    .build();

            GotoBallPile2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(60.387, 17.975),
                                    new Pose(62.205, 48.067),
                                    new Pose(47.624, 57.377)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(117), Math.toRadians(180))

                    .build();

            IntakeBallPile2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47.624, 57.377),

                                    new Pose(11.916, 57.175)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Shoot3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(11.916, 57.175),
                                    new Pose(57.560, 47.058),
                                    new Pose(56.550, 90.278)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(139))

                    .build();

            GotoBallPile3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.550, 90.278),
                                    new Pose(50.491, 82.199),
                                    new Pose(43.826, 83.815)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(139), Math.toRadians(180))

                    .build();

            IntakeBallPile3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43.826, 83.815),

                                    new Pose(18.369, 83.815)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Shoot4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(18.369, 83.815),
                                    new Pose(40.797, 77.352),
                                    new Pose(56.550, 90.076)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))

                    .build();

            GoPark = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.550, 90.076),
                                    new Pose(44.028, 80.785),
                                    new Pose(29.891, 88.460)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(90))

                    .build();
        }
    }
}