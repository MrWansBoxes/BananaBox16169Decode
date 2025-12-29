package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutoBottomBlue extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private DcMotor launcher1;
    private DcMotor launcher2;
    private Servo flip1;
    private DcMotor intake;
    private DcMotorEx turret;

    private final Pose startPose = new Pose(48.2243, 8.29906542056074, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(34.766356, 11.887850467289713, Math.toRadians(90)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierCurve(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                if (pathTimer.getElapsedTimeSeconds() > 27){
                    sleep(200);
                    flip1.setPosition(0.86);
                    sleep(200);
                    flip1.setPosition(0.5);
                    sleep(700);
                    intake.setPower(1);
                    sleep(700);
                    flip1.setPosition(0.86);
                    sleep(200);
                    flip1.setPosition(0.5);
                    sleep(700);
                    flip1.setPosition(0.86);
                    sleep(200);
                    flip1.setPosition(0.5);
                    sleep(500);
                    launcher1.setPower(0.0);
                    launcher2.setPower(0.0);
                }

                if (pathTimer.getElapsedTimeSeconds() > 27) {
                    follower.followPath(scorePreload);
                    setPathState(-1);
                }
                    break;

        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
        launcher1 = hardwareMap.get(DcMotor.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotor.class, "launcher2");
        flip1 = hardwareMap.get(Servo.class,"flip1");
        intake = hardwareMap.get(DcMotor.class,"intake");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}
