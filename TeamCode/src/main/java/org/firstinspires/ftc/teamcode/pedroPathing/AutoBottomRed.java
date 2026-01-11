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

public class AutoBottomRed {

    private final Follower follower;

    private final Servo flip1;
    private final DcMotor intake;
    private final DcMotor launcher1;
    private final DcMotor launcher2;

    private Paths paths;
    private int pathState;
    private final Timer pathTimer, shootTimer, intakeTimer;


    private final double launcherPowerFar1 = 0.8;  // Variables for tuning
    private final double launcherPowerFar2 = -0.8;
    private final double launcherPowerClose1 = 0.68;
    private final double launcherPowerClose2 = -0.68;
    private final int launcherOff = 0;
    private final int intakeOn = 1;
    private final int intakeOff = 0;
    private final double flickUp = 0.86;
    private final double flickDown = 0.5;

    public AutoBottomRed(Follower follower, Servo flip1, DcMotor intake, DcMotor launcher1, DcMotor launcher2) {

        this.follower = follower;
        this.flip1 = flip1;
        this.intake = intake;
        this.launcher1 = launcher1;
        this.launcher2 = launcher2;

        pathTimer = new Timer();
        shootTimer = new Timer();
        intakeTimer = new Timer();
    }

    public void start() {
        follower.setStartingPose(new Pose(95.73071528751754, 8.078541374474053, Math.toRadians(90)));  // starting spot
        paths = new Paths(follower);
        setPathState(0);
    }

    public void update() {
        follower.update();
        autonomousPathUpdate();
    }

    private void launch3balls() {  // we call this function every time you want to launch 3 balls

        flip1.setPosition(flickUp);
        sleep(500);
        flip1.setPosition(flickDown);
        sleep(300);
        intake.setPower(intakeOn);
        sleep(500);
        flip1.setPosition(flickUp);
        sleep(500);
        flip1.setPosition(flickDown);
        sleep(450);
        flip1.setPosition(flickUp);
        sleep(500);
        flip1.setPosition(flickDown);
        launcher1.setPower(launcherOff);
        launcher2.setPower(launcherOff);
        intake.setPower(intakeOff);
    }

    /* You could check for
           - Follower State: "if(!follower.isBusy()) {}"
           - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
           - Robot Position: "if(follower.getPose().getX() > 36) {}"
           */

    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                launcher1.setPower(launcherPowerFar1);  // set power to launcher and moves to shoot position
                launcher2.setPower(launcherPowerFar2);
                sleep(300);
                follower.followPath(paths.Shoot1, true);
                setPathState(1);
                break;

            case 1:

                if (!follower.isBusy()) {
                    flip1.setPosition(flickUp);
                    sleep(200);
                    flip1.setPosition(flickDown);
                    launch3balls();// when the robot finishes the path it will launch 3 balls
                }
                if (pathTimer.getElapsedTimeSeconds() > 2) {  // after 4 seconds it will move to next path and turn on the intake
                    intake.setPower(intakeOn);
                    follower.followPath(paths.GotoBallPile1, true);
                    setPathState(2);
                }

                break;
            case 2:
                if (!follower.isBusy()) {  // when it is finished with its path the robot will intake the balls then power up the motors and turn off the intake
                    follower.followPath(paths.IntakeBallPile1, 0.5, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {// moves to shoot position

                    intake.setPower(intakeOff);
                    launcher1.setPower(launcherPowerFar1);
                    launcher2.setPower(launcherPowerFar2);
                    follower.followPath(paths.Shoot2, true);
                    setPathState(4);
                }
                break;

            case 4:

                if (!follower.isBusy()) launch3balls();  // when the robot finishes the path it will launch 3 balls

                if (pathTimer.getElapsedTimeSeconds() > 2) {  // after 4 seconds it will move to next path and turn on the intake
                    intake.setPower(intakeOn);
                    follower.followPath(paths.GotoBallPile2, true);
                    setPathState(5);
                }

                break;

            case 5:
                if (!follower.isBusy()) {  // when it is finished with its path the robot will intake the balls then power up the motors and turn off the intake
                    follower.followPath(paths.IntakeBallPile2, 0.5,true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {  // moves to shoot position
                    intake.setPower(intakeOff);
                    launcher1.setPower(launcherPowerClose1);
                    launcher2.setPower(launcherPowerClose2);
                    follower.followPath(paths.Shoot3, true);
                    setPathState(7);
                }
                break;

            case 7:

                if (!follower.isBusy()) {
                    launch3balls();// when the robot finishes the path it will launch 3 balls
                }
                if (pathTimer.getElapsedTimeSeconds() > 2) {  // after 4 seconds it will move to next path and turn on the intake
                    intake.setPower(intakeOn);
                    follower.followPath(paths.GotoBallPile3, true);
                    setPathState(8);
                }

                break;

            case 8:
                if (!follower.isBusy()) {  // when it is finished with its path the robot will intake the balls then power up the motors and turn off the intake
                    follower.followPath(paths.IntakeBallPile3, 0.5, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {  // moves to shoot position
                    intake.setPower(intakeOff);
                    launcher1.setPower(launcherPowerClose1);
                    launcher2.setPower(launcherPowerClose2);
                    follower.followPath(paths.Shoot4, true);
                }
                break;

            case 10:

                if (!follower.isBusy()) {
                    launch3balls();// when the robot finishes the path it will launch 3 balls
                }

                if (pathTimer.getElapsedTimeSeconds() > 4) {  // after 4 seconds it will move to next path and turn on the intake
                    follower.followPath(paths.GoPark, true);
                    setPathState(-1);
                }

                break;
        }
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

            Shoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(95.731, 8.684),
                                    new Pose(100.174, 21.408),
                                    new Pose(86.440, 20.802)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(71))
                    .build();

            GotoBallPile1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(86.440, 20.802),
                                    new Pose(97.346, 22.216),
                                    new Pose(103.203, 33.122)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(71), Math.toRadians(0))
                    .build();

            IntakeBallPile1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(103.203, 33.122), new Pose(134.912, 32.516))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(134.912, 32.516),
                                    new Pose(101.790, 47.461),
                                    new Pose(86.440, 20.802)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
                    .build();

            GotoBallPile2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(86.440, 20.802),
                                    new Pose(92.701, 47.461),
                                    new Pose(102.799, 57.358)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
                    .build();

            IntakeBallPile2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(102.799, 57.358), new Pose(134.710, 56.954))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(134.710, 56.954),
                                    new Pose(85.431, 53.318),
                                    new Pose(88.864, 88.460)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            GotoBallPile3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.864, 88.460),
                                    new Pose(93.307, 82.603),
                                    new Pose(104.213, 81.593)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            IntakeBallPile3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(104.213, 81.593), new Pose(128.853, 81.391))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(128.853, 81.391),
                                    new Pose(86.642, 81.593),
                                    new Pose(88.864, 88.460)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
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
}