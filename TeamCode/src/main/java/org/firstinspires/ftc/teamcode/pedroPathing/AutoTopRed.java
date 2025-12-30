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

public class AutoTopRed {

    private final Follower follower;

    private final Servo flip1;
    private final DcMotor intake;
    private final DcMotor launcher1;
    private final DcMotor launcher2;

    private Paths paths;
    private int pathState;
    private final Timer pathTimer;

    private final double launcherPowerFar1 = 0.85;  // Variables for tuning
    private final double launcherPowerFar2 = -0.85;
    private final double launcherPowerClose1 = 0.68;
    private final double launcherPowerClose2 = -0.68;
    private final int launcherOff = 0;
    private final int intakeOn = 1;
    private final int intakeOff = 0;
    private final double flickUp = 0.86;
    private final double flickDown = 0.5;

    public AutoTopRed(Follower follower, Servo flip1, DcMotor intake, DcMotor launcher1, DcMotor launcher2) {

        this.follower = follower;
        this.flip1 = flip1;
        this.intake = intake;
        this.launcher1 = launcher1;
        this.launcher2 = launcher2;

        pathTimer = new Timer();
    }

    public void start() {
        follower.setStartingPose(new Pose(111.68583450210379, 135.51753155680223, Math.toRadians(90)));  // starting spot
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

            public PathChain Shoot1, GotoBallPile1, IntakeBallPile1,
                    Shoot2, GotoBallPile2, IntakeBallPile2,
                    Shoot3, GotoBallPile3, IntakeBallPile3,
                    Shoot4, GoPark;

            public Paths(Follower follower){


        Shoot1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(111.686, 135.518), new Pose(83.815, 83.209))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(52))
                .build();

        GotoBallPile1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(83.815, 83.209), new Pose(104.819, 83.411))
                )
                .setLinearHeadingInterpolation(Math.toRadians(52), Math.toRadians(0))
                .build();

        IntakeBallPile1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(104.819, 83.411), new Pose(129.257, 83.411))
                )
                .setTangentHeadingInterpolation()
                .build();

        Shoot2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(129.257, 83.411),
                                new Pose(104.617, 69.273),
                                new Pose(83.815, 83.209)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(52))
                .build();

        GotoBallPile2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(83.815, 83.209),
                                new Pose(81.795, 58.771),
                                new Pose(104.617, 59.579)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(52), Math.toRadians(0))
                .build();

        IntakeBallPile2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(104.617, 59.579), new Pose(135.719, 59.579))
                )
                .setTangentHeadingInterpolation()
                .build();

        Shoot3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(135.719, 59.579),
                                new Pose(77.554, 50.087),
                                new Pose(83.815, 83.209)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(52))
                .build();

        GotoBallPile3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(83.815, 83.209),
                                new Pose(75.736, 33.122),
                                new Pose(104.415, 35.142)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(52), Math.toRadians(0))
                .build();

        IntakeBallPile3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(104.415, 35.142), new Pose(135.719, 35.142))
                )
                .setTangentHeadingInterpolation()
                .build();

        Shoot4 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(135.719, 35.142),
                                new Pose(80.381, 38.777),
                                new Pose(83.209, 19.792)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))
                .build();

        GoPark = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(83.209, 19.792),
                                new Pose(107.243, 22.418),
                                new Pose(108.050, 10.300)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(180))
                .build();
    }
}
    /* You could check for
           - Follower State: "if(!follower.isBusy()) {}"
           - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
           - Robot Position: "if(follower.getPose().getX() > 36) {}"
           */

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

                    follower.followPath(paths.Shoot3,true);
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