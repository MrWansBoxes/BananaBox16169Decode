package org.firstinspires.ftc.teamcode.pedroPathing;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoTopBlue {

    private final Follower follower;

    private final Servo flip1;
    private final DcMotor intake;
    private final DcMotor launcher1;
    private final DcMotor launcher2;

    private int pathState;
    private final Timer pathTimer;

    private Paths paths;


    private final double launcherPowerFar1 = 0.85;  // Variables for tuning
    private final double launcherPowerFar2 = -0.85;
    private final double launcherPowerClose1 = 0.68;
    private final double launcherPowerClose2 = -0.68;
    private final int launcherOff = 0;
    private final int intakeOn = 1;
    private final int intakeOff = 0;
    private final double flickUp = 0.86;
    private final double flickDown = 0.5;

    public AutoTopBlue(Follower follower, Servo flip1, DcMotor intake, DcMotor launcher1, DcMotor launcher2) {

        this.follower = follower;
        this.flip1 = flip1;
        this.intake = intake;
        this.launcher1 = launcher1;
        this.launcher2 = launcher2;

        pathTimer = new Timer();
    }

    public void start() {
        follower.setStartingPose(new Pose(32.31416549789621, 135.51753155680223, Math.toRadians(90))); // starting spot
        paths = new Paths(follower);
        setPathState(0);
    }

    public void update() {
        follower.update();
        autonomousPathUpdate();
    }

    private void launch3balls() {  // we call this function every time you want to launch 3 balls
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

        public PathChain Shoot1,GotoBallPile1, IntakeBallPile1,
                Shoot2, GotoBallPile2, IntakeBallPile2,
                Shoot3, GotoBallPile3, IntakeBallPile3,
                Shoot4, GoPark;

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