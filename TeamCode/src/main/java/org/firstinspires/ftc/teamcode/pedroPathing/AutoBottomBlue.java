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

public class AutoBottomBlue {

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

    public AutoBottomBlue(Follower follower, Servo flip1, DcMotor intake, DcMotor launcher1, DcMotor launcher2) {

        this.follower = follower;
        this.flip1 = flip1;
        this.intake = intake;
        this.launcher1 = launcher1;
        this.launcher2 = launcher2;

        pathTimer = new Timer();
    }

    public void start() {
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90))); // starting spot
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
                follower.followPath(paths.Starttoshoot1);
                setPathState(1);
                break;

            case 1:

                if (!follower.isBusy()) launch3balls();  // when the robot finishes the path it will launch 3 balls

                if (pathTimer.getElapsedTimeSeconds() > 4) {  // after 4 seconds it will move to next path and turn on the intake
                    intake.setPower(intakeOn);
                    follower.followPath(paths.Movetoballpile1, true);
                    setPathState(2);
                }

                break;

            case 2:
                if (!follower.isBusy()) {  // when it is finished with its path the robot will intake the balls then power up the motors and turn off the intake
                    follower.followPath(paths.Intakeballpile1, true);
                    sleep(300);
                    intake.setPower(intakeOff);
                    launcher1.setPower(launcherPowerFar1);
                    launcher2.setPower(launcherPowerFar2);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {  // moves to shoot position
                    follower.followPath(paths.Shootballpile1, true);
                    setPathState(4);
                }
                break;

            case 4:

                if (!follower.isBusy()) launch3balls();  // when the robot finishes the path it will launch 3 balls

                if (pathTimer.getElapsedTimeSeconds() > 4) {  // after 4 seconds it will move to next path and turn on the intake
                    intake.setPower(intakeOn);
                    follower.followPath(paths.Gotoballpile2, true);
                    setPathState(5);
                }

                break;

            case 5:
                if (!follower.isBusy()) {  // when it is finished with its path the robot will intake the balls then power up the motors and turn off the intake
                    follower.followPath(paths.Grabballpile2, true);
                    sleep(300);
                    intake.setPower(intakeOff);
                    launcher1.setPower(launcherPowerClose1);
                    launcher2.setPower(launcherPowerClose2);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {  // moves to shoot position
                    follower.followPath(paths.Shootballpile2, true);
                    setPathState(7);
                }
                break;

            case 7:

                if (!follower.isBusy()) launch3balls();  // when the robot finishes the path it will launch 3 balls

                if (pathTimer.getElapsedTimeSeconds() > 4) {  // after 4 seconds it will move to next path and turn on the intake
                    intake.setPower(intakeOn);
                    follower.followPath(paths.Gotoballpile3, true);
                    setPathState(8);
                }

                break;

            case 8:
                if (!follower.isBusy()) {  // when it is finished with its path the robot will intake the balls then power up the motors and turn off the intake
                    follower.followPath(paths.Pickupballpile3, true);
                    sleep(300);
                    intake.setPower(intakeOff);
                    launcher1.setPower(launcherPowerClose1);
                    launcher2.setPower(launcherPowerClose2);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {  // moves to shoot position
                    follower.followPath(paths.Shootballpile3, true);
                    setPathState(10);
                }
                break;

            case 10:

                if (!follower.isBusy()) launch3balls();  // when the robot finishes the path it will launch 3 balls

                if (pathTimer.getElapsedTimeSeconds() > 4) {  // after 4 seconds it will move to next path and turn on the intake
                    follower.followPath(paths.Gopark, true);
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
        public PathChain Starttoshoot1, Movetoballpile1, Intakeballpile1,
                Shootballpile1, Gotoballpile2, Grabballpile2,
                Shootballpile2, Gotoballpile3, Pickupballpile3,
                Shootballpile3, Gopark;

        public Paths(Follower follower) {

            Starttoshoot1 = follower.pathBuilder()  // this sets up all the paths with the control points and heading
                    .addPath(new BezierCurve(
                            new Pose(48.269, 7.877),
                            new Pose(57.358, 9.290),
                            new Pose(60.387, 17.975)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                    .build();

            Movetoballpile1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(60.387, 17.975),
                            new Pose(55.338, 36.151),
                            new Pose(40.191, 35.546)))
                    .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                    .build();

            Intakeballpile1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(40.191, 35.546),
                            new Pose(8.886, 35.546)))
                    .setTangentHeadingInterpolation()
                    .build();

            Shootballpile1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(8.886, 35.546),
                            new Pose(52.511, 40.999),
                            new Pose(60.387, 17.975)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                    .build();

            Gotoballpile2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(60.387, 17.975),
                            new Pose(62.205, 48.067),
                            new Pose(40.393, 59.781)))
                    .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                    .build();

            Grabballpile2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(40.393, 59.781),
                            new Pose(8.281, 59.579)))
                    .setTangentHeadingInterpolation()
                    .build();

            Shootballpile2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(8.281, 59.579),
                            new Pose(57.560, 47.058),
                            new Pose(56.550, 90.278)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                    .build();

            Gotoballpile3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(56.550, 90.278),
                            new Pose(50.491, 82.199),
                            new Pose(39.787, 84.219)))
                    .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                    .build();

            Pickupballpile3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(39.787, 84.219),
                            new Pose(15.147, 84.017)))
                    .setTangentHeadingInterpolation()
                    .build();

            Shootballpile3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(15.147, 84.017),
                            new Pose(40.797, 77.352),
                            new Pose(56.550, 90.076)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                    .build();

            Gopark = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(56.550, 90.076),
                            new Pose(44.028, 80.785),
                            new Pose(29.891, 88.460)))
                    .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(90))
                    .build();
        }
    }
}
