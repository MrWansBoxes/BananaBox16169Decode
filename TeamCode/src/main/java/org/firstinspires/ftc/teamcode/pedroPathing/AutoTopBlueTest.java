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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.tuningAndConstants.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class AutoTopBlueTest extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Servo flip1;
    private DcMotor intake;
    private DcMotor launcher1;
    private DcMotor launcher2;
    private double launcherPowerFar1 = 0.85;  // variables for tuning
    private double launcherPowerFar2 = -0.85;
    private int launcherOff = 0;
    private double launcherPowerClose1 = 0.68;
    private double launcherPowerClose2 = -0.68;
    private int intakeOn = 1;
    private int intakeOff = 0;
    private double flickUp = 0.86;
    private double flickDown = 0.5;

    @Override
    public void init() {

        flip1 = hardwareMap.get(Servo.class, "flip1");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher1 = hardwareMap.get(DcMotor.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotor.class, "launcher2");
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(32.31416549789621, 135.51753155680223, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    private void launch3balls() {
        sleep(200);
        flip1.setPosition(flickUp);
        sleep(200);
        flip1.setPosition(flickDown);
        sleep(700);
        intake.setPower(intakeOn);
        sleep(1000);
        flip1.setPosition(flickUp);
        sleep(200);
        flip1.setPosition(flickDown);
        sleep(800);
        flip1.setPosition(flickUp);
        sleep(200);
        flip1.setPosition(flickDown);
        sleep(600);
        launcher1.setPower(launcherOff);
        launcher2.setPower(launcherOff);
        intake.setPower(intakeOff);
    }

    public static class Paths {
        public PathChain Shoot1;
        public PathChain GotoBallPile1;
        public PathChain IntakeBallPile1;
        public PathChain Shoot2;
        public PathChain GotoBallPile2;
        public PathChain IntakeBallPile2;
        public PathChain Shoot3;
        public PathChain GotoBallPile3;
        public PathChain IntakeBallPile3;
        public PathChain Shoot4;
        public PathChain GoPark;

        public Paths(Follower follower) {
            Shoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(32.314, 135.518),
                                    new Pose(33.930, 101.588),
                                    new Pose(59.781, 83.613)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(132))
                    .build();

            GotoBallPile1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.781, 83.613), new Pose(39.987, 83.962))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(180))
                    .build();

            IntakeBallPile1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(39.987, 83.962), new Pose(14.339, 84.219))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(14.339, 84.219),
                                    new Pose(36.757, 73.919),
                                    new Pose(59.781, 83.613)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(132))
                    .build();

            GotoBallPile2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(59.781, 83.613),
                                    new Pose(60.387, 61.195),
                                    new Pose(39.787, 59.781)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(180))
                    .build();

            IntakeBallPile2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(39.787, 59.781), new Pose(8.281, 59.781))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(8.281, 59.781),
                                    new Pose(62.609, 52.309),
                                    new Pose(59.781, 83.613)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(132))
                    .build();

            GotoBallPile3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(59.781, 83.613),
                                    new Pose(60.589, 34.334),
                                    new Pose(39.383, 35.748)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(180))
                    .build();

            IntakeBallPile3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(39.383, 35.748), new Pose(8.281, 35.546))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(8.281, 35.546),
                                    new Pose(13.935, 21.206),
                                    new Pose(61.397, 21.408)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(117))
                    .build();

            GoPark = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(61.397, 21.408),
                                    new Pose(55.136, 6.665),
                                    new Pose(35.546, 9.492)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(117), Math.toRadians(180))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                launcher1.setPower(launcherPowerClose1);
                launcher2.setPower(launcherPowerClose2);      // start launcher motors
                follower.followPath(paths.Shoot1);
                setPathState(1);

                break;
            case 1:


                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    intake.setPower(intakeOn);
                    follower.followPath(paths.GotoBallPile1,true);
                    setPathState(2);
                }


                if(!follower.isBusy()) {
                    launch3balls();
                }
                break;
            case 2:
                if(!follower.isBusy()) {

                    follower.followPath(paths.IntakeBallPile1,true);
                    setPathState(3);
                    sleep(300);
                    intake.setPower(intakeOff);
                    launcher1.setPower(launcherPowerClose1);
                    launcher2.setPower(launcherPowerClose2);      // start launcher motors
                }
                break;
            case 3:
                if(!follower.isBusy()) {

                    follower.followPath(paths.Shoot2,true);
                    setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    intake.setPower(intakeOn);
                    follower.followPath(paths.GotoBallPile2,true);
                    setPathState(6);
                }
                if (!follower.isBusy()) {
                    launch3balls();
                }
                break;
            case 6:
                if(!follower.isBusy()) {

                    follower.followPath(paths.IntakeBallPile2,true);
                    setPathState(7);
                    sleep(300);
                    intake.setPower(intakeOff);
                    launcher1.setPower(launcherPowerClose1);
                    launcher2.setPower(launcherPowerClose2);      // start launcher motors for close side
                }
                break;
            case 7:
                if(!follower.isBusy()) {

                    follower.followPath(paths.Shoot2,true);
                    setPathState(8);
                }
                break;
            case 8:

                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    intake.setPower(intakeOn);
                    follower.followPath(paths.GotoBallPile3, true);
                    setPathState(9);
                }
                if(!follower.isBusy()) {
                    launch3balls();
                }
                break;

            case 9:
                if(!follower.isBusy()) {

                    follower.followPath(paths.IntakeBallPile3, true);
                    setPathState(10);
                    sleep(300);
                    intake.setPower(intakeOff);
                    launcher1.setPower(launcherPowerFar1);
                    launcher2.setPower(launcherPowerFar2);
                }
                break;
            case 10:
                if(!follower.isBusy()) {

                    follower.followPath(paths.Shoot4,true);
                    setPathState(4);
                }
                break;
            case 11:

                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    follower.followPath(paths.GoPark, true);
                    setPathState(-1);
                }

                if(!follower.isBusy()) {

                    launch3balls();

                }
                break;


        }
        return pathState;
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
