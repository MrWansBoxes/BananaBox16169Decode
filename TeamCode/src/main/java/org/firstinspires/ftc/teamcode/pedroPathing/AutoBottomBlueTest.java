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

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class AutoBottomBlueTest extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer, actionTimer, opmodeTimer;
    private  Servo flip1;
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

        public PathChain Starttoshoot1;
        public PathChain Movetoballpile1;
        public PathChain Intakeballpile1;
        public PathChain Shootballpile1;
        public PathChain Gotoballpile2;
        public PathChain Grabballpile2;
        public PathChain Shootballpile2;
        public PathChain Gotoballpile3;
        public PathChain Pickupballpile3;
        public PathChain Shootballpile3;
        public PathChain Gopark;




        public Paths(Follower follower) {
            Starttoshoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.269, 7.877),
                                    new Pose(57.358, 9.290),
                                    new Pose(60.387, 17.975)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                    .build();

            Movetoballpile1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.387, 17.975),
                                    new Pose(55.338, 36.151),
                                    new Pose(40.191, 35.546)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                    .build();

            Intakeballpile1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.191, 35.546), new Pose(8.886, 35.546))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Shootballpile1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(8.886, 35.546),
                                    new Pose(52.511, 40.999),
                                    new Pose(60.387, 17.975)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                    .build();

            Gotoballpile2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.387, 17.975),
                                    new Pose(62.205, 48.067),
                                    new Pose(40.393, 59.781)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                    .build();

            Grabballpile2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.393, 59.781), new Pose(8.281, 59.579))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Shootballpile2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(8.281, 59.579),
                                    new Pose(57.560, 47.058),
                                    new Pose(56.550, 90.278)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                    .build();

            Gotoballpile3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.550, 90.278),
                                    new Pose(50.491, 82.199),
                                    new Pose(39.787, 84.219)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                    .build();

            Pickupballpile3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(39.787, 84.219), new Pose(15.147, 84.017))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Shootballpile3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(15.147, 84.017),
                                    new Pose(40.797, 77.352),
                                    new Pose(56.550, 90.076)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                    .build();

            Gopark = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.550, 90.076),
                                    new Pose(44.028, 80.785),
                                    new Pose(29.891, 88.460)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(90))
                    .build();
        }
    }


    /* You could check for
                - Follower State: "if(!follower.isBusy())"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1)"
                - Robot Position: "if(follower.getPose().getX() > 36)"
                */

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                launcher1.setPower(launcherPowerFar1);
                launcher2.setPower(launcherPowerFar2);      // start launcher motors

                follower.followPath(paths.Starttoshoot1);
                setPathState(1);

                break;
            case 1:

                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    intake.setPower(intakeOn);
                    follower.followPath(paths.Movetoballpile1,true);
                    setPathState(2);
                }


                if(!follower.isBusy()) {
                    launch3balls();
                }
                break;
            case 2:
                if(!follower.isBusy()) {

                    follower.followPath(paths.Intakeballpile1,true);
                    setPathState(3);
                    sleep(300);
                    intake.setPower(intakeOff);
                    launcher1.setPower(launcherPowerFar1);
                    launcher2.setPower(launcherPowerFar2);      // start launcher motors
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    intake.setPower(intakeOn);
                    follower.followPath(paths.Shootballpile1,true);
                    setPathState(4);
                }
                if (!follower.isBusy()) {
                    launch3balls();
                }
                break;
            case 4:

                if(!follower.isBusy()) {


                    follower.followPath(paths.Gotoballpile2,true);
                    setPathState(5);
                }
                break;
            case 5:

                if(!follower.isBusy()) {

                    follower.followPath(paths.Grabballpile2,true);
                    setPathState(6);
                    sleep(300);
                    intake.setPower(intakeOff);
                    launcher1.setPower(launcherPowerClose1);
                    launcher2.setPower(launcherPowerClose2);      // start launcher motors for close side
                }
                break;
            case 6:

                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    intake.setPower(intakeOn);
                    follower.followPath(paths.Shootballpile2, true);
                    setPathState(7);
                }
                if(!follower.isBusy()) {
                    launch3balls();
                }
                break;
            case 7:
                if (!follower.isBusy()) {

                    follower.followPath(paths.Gotoballpile3, true);
                    setPathState(8);
                }
                break;
            case 8:

                if(!follower.isBusy()) {

                    follower.followPath(paths.Pickupballpile3, true);
                    setPathState(9);
                    sleep(300);
                    intake.setPower(intakeOff);
                    launcher1.setPower(launcherPowerClose1);
                    launcher2.setPower(launcherPowerClose2);
                }
                break;
            case 9:

                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    intake.setPower(intakeOn);
                    follower.followPath(paths.Shootballpile3, true);
                    setPathState(10);
                }

                if(!follower.isBusy()) {

                    launch3balls();

                }
                break;
            case 10:

                if(!follower.isBusy()) {

                    follower.followPath(paths.Gopark, true);
                    setPathState(-1);
                }
                break;

        }
        return pathState;
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
