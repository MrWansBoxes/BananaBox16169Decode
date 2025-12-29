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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.TestAutoBottomRed;

@Autonomous(name = "Test Auto Bottom Red", group = "Autonomous")
@Configurable // Panels
public class TestAutoBottomRed extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Servo flip1;
    private DcMotor intake;
    private DcMotor launcher1;
    private DcMotor launcher2;
    private double launcherPowerFar1 = 0.85;
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
        flip1 = hardwareMap.get(Servo .class,"flip1");
        intake = hardwareMap.get(DcMotor .class,"intake");
        launcher1 = hardwareMap.get(DcMotor.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotor.class, "launcher2");
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new TestAutoBottomRed.Paths(follower); // Build paths

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
                                    new Pose(95.731, 8.079),
                                    new Pose(100.780, 16.561),
                                    new Pose(82.805, 15.349)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))
                    .build();

            GotoBallPile1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(82.805, 15.349),
                                    new Pose(97.346, 22.216),
                                    new Pose(103.405, 35.344)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))
                    .build();

            IntakeBallPile1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(103.405, 35.344), new Pose(136.325, 35.748))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(136.325, 35.748),
                                    new Pose(101.790, 47.461),
                                    new Pose(82.805, 15.349)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))
                    .build();

            GotoBallPile2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(82.805, 15.349),
                                    new Pose(92.701, 47.461),
                                    new Pose(103.001, 59.579)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))
                    .build();

            IntakeBallPile2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(103.001, 59.579), new Pose(135.921, 59.781))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(135.921, 59.781),
                                    new Pose(85.431, 53.318),
                                    new Pose(88.864, 88.460)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                    .build();

            GotoBallPile3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.864, 88.460),
                                    new Pose(93.307, 82.603),
                                    new Pose(104.415, 83.411)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                    .build();

            IntakeBallPile3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(104.415, 83.411), new Pose(129.055, 83.411))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(129.055, 83.411),
                                    new Pose(86.642, 81.593),
                                    new Pose(88.864, 88.460)
                            )
                    )

                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                    .build();

            GoPark = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.864, 88.460),
                                    new Pose(102.597, 97.952),
                                    new Pose(116.129, 88.662)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(270))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                launcher1.setPower(launcherPowerFar1);
                launcher2.setPower(launcherPowerFar2);      // start launcher motors
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
                    launcher1.setPower(launcherPowerFar1);
                    launcher2.setPower(launcherPowerFar2);      // start launcher motors
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
                    launcher1.setPower(launcherPowerClose1);
                    launcher2.setPower(launcherPowerClose2);
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
