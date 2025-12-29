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

    public AutoBottomRed(Follower follower, Servo flip1, DcMotor intake, DcMotor launcher1, DcMotor launcher2) {

        this.follower = follower;
        this.flip1 = flip1;
        this.intake = intake;
        this.launcher1 = launcher1;
        this.launcher2 = launcher2;

        pathTimer = new Timer();
    }

    public void start() {
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));  // starting spot
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
                follower.followPath(paths.Shoot1);
                setPathState(1);
                break;

            case 1:

                if (!follower.isBusy()) launch3balls();   // when the robot finishes the path it will launch 3 balls

                if (pathTimer.getElapsedTimeSeconds() > 4) {   // after 4 seconds it will move to next path and turn on the intake
                    intake.setPower(intakeOn);
                    follower.followPath(paths.GotoBallPile1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {  // when it is finished with its path the robot will intake the balls then power up the motors and turn off the intake
                    follower.followPath(paths.IntakeBallPile1, true);
                    sleep(300);
                    intake.setPower(intakeOff);
                    launcher1.setPower(launcherPowerFar1);
                    launcher2.setPower(launcherPowerFar2);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {  // moves to shoot position
                    follower.followPath(paths.Shoot2, true);
                    setPathState(4);
                }
                break;

            case 4:

                if (!follower.isBusy()) launch3balls(); // when the robot finishes the path it will launch 3 balls

                if (pathTimer.getElapsedTimeSeconds() > 4) {  // after 4 seconds it will move to next path and turn on the intake
                    intake.setPower(intakeOn);
                    follower.followPath(paths.GotoBallPile2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) { // when it is finished with its path the robot will intake the balls then power up the motors and turn off the intake
                    follower.followPath(paths.IntakeBallPile2, true);
                    sleep(300);
                    intake.setPower(intakeOff);
                    launcher1.setPower(launcherPowerClose1);
                    launcher2.setPower(launcherPowerClose2);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {  // moves to shoot position
                    follower.followPath(paths.Shoot2, true);
                    setPathState(8);
                }
                break;
            case 8:

                if (!follower.isBusy()) launch3balls();  // when the robot finishes the path it will launch 3 balls

                if (pathTimer.getElapsedTimeSeconds() > 4) {  // after 4 seconds it will move to next path and turn on the intake
                    intake.setPower(intakeOn);
                    follower.followPath(paths.GotoBallPile3, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {  // when it is finished with its path the robot will intake the balls then power up the motors and turn off the intake
                    follower.followPath(paths.IntakeBallPile3, true);
                    sleep(300);
                    intake.setPower(intakeOff);
                    launcher1.setPower(launcherPowerClose1);
                    launcher2.setPower(launcherPowerClose2);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) { // moves to shoot position
                    follower.followPath(paths.Shoot4, true);
                    setPathState(11);
                }
                break;

            case 11:

                if (!follower.isBusy()) launch3balls(); // when the robot finishes the path it will launch 3 balls

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

            Shoot1 = follower.pathBuilder()  // this sets up all the paths with the control points and heading
                    .addPath(new BezierCurve(
                            new Pose(95.731, 8.079),
                            new Pose(100.780, 16.561),
                            new Pose(82.805, 15.349)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))
                    .build();

            GotoBallPile1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(82.805, 15.349),
                            new Pose(97.346, 22.216),
                            new Pose(103.405, 35.344)))
                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))
                    .build();

            IntakeBallPile1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(103.405, 35.344),
                            new Pose(136.325, 35.748)))
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(136.325, 35.748),
                            new Pose(101.790, 47.461),
                            new Pose(82.805, 15.349)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))
                    .build();

            GotoBallPile2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(82.805, 15.349),
                            new Pose(92.701, 47.461),
                            new Pose(103.001, 59.579)))
                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))
                    .build();

            IntakeBallPile2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(103.001, 59.579),
                            new Pose(135.921, 59.781)))
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(135.921, 59.781),
                            new Pose(85.431, 53.318),
                            new Pose(88.864, 88.460)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                    .build();

            GotoBallPile3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(88.864, 88.460),
                            new Pose(93.307, 82.603),
                            new Pose(104.415, 83.411)))
                    .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                    .build();

            IntakeBallPile3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(104.415, 83.411),
                            new Pose(129.055, 83.411)))
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot4 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(129.055, 83.411),
                            new Pose(86.642, 81.593),
                            new Pose(88.864, 88.460)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                    .build();

            GoPark = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(88.864, 88.460),
                            new Pose(102.597, 97.952),
                            new Pose(116.129, 88.662)))
                    .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(270))
                    .build();
        }
    }
}
