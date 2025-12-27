package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutoTopRed extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private DcMotor launcher1;
    private DcMotor launcher2;
    private Servo flip1;
    private DcMotor intake;
    private DcMotorEx turret;
    private Limelight3A limelight;
    public static double P = 0.02;
    public static double I = 0.0;
    public static double D = 0.0;

    private double integral = 0;
    private double lastError = 0;

    private final Pose startPose = new Pose(113.7, 135.4696, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose path1 = new Pose(89.1622, 88.5528, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose path2 = new Pose(128.29906542056074, 108.11214953271028, Math.toRadians(90)); // private final Pose path2 = new Pose(102.9732, 83.6784, Math.toRadians(0));
    private final Pose path3 = new Pose(128.1579, 83.4753, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose path4 = new Pose(89.1622, 88.5528, Math.toRadians(45)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose path5 = new Pose(128.5641, 75.7574, Math.toRadians(90));// open gate
    private final Pose path20 = new Pose(89.3653, 70.2736, Math.toRadians(0));
    private final Pose path6 = new Pose(103.5825, 59.9153, Math.toRadians(0));
    private final Pose path7 = new Pose(136, 59.5091, Math.toRadians(0));
    private final Pose path8 = new Pose(89.1622, 88.5528, Math.toRadians(45));
    private final Pose path9 = new Pose(103.5825, 35.3399, Math.toRadians(0));
    private final Pose path10 = new Pose(136, 35.5430, Math.toRadians(0));
    private final Pose path11 = new Pose(89.1622, 88.5528, Math.toRadians(45));
    private final Pose path14 = new Pose(89.1622, 88.5528, Math.toRadians(45));
    private final Pose path15 = new Pose(102.1607, 57.4781, Math.toRadians(0));


    private Path scorePreload;
    private PathChain paths1, paths2, paths3, paths4, paths5, paths6, paths7, paths8, paths9, paths10, paths11, paths15;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierCurve(startPose, path1));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), path1.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        paths1 = follower.pathBuilder()
                .addPath(new BezierCurve(path1, path2))
                .setLinearHeadingInterpolation(path1.getHeading(), path2.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        paths2 = follower.pathBuilder()
                .addPath(new BezierCurve(path2, path3))
                .setLinearHeadingInterpolation(path2.getHeading(), path3.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        paths3 = follower.pathBuilder()
                .addPath(new BezierCurve(path3, path4))
                .setLinearHeadingInterpolation(path3.getHeading(), path4.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        paths4 = follower.pathBuilder()
                .addPath(new BezierCurve(path4, path6))
                .setLinearHeadingInterpolation(path4.getHeading(), path6.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
     //   paths5 = follower.pathBuilder()
       //         .addPath(new BezierCurve(path5, path20))
        //        .setLinearHeadingInterpolation(path5.getHeading(), path20.getHeading())
         //       .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
       // paths6 = follower.pathBuilder()
       //         .addPath(new BezierCurve(path, path6))
       //         .setLinearHeadingInterpolation(path20.getHeading(), path6.getHeading())
        //        .build();

        paths7 = follower.pathBuilder()
                .addPath(new BezierCurve(path6, path7))
                .setLinearHeadingInterpolation(path6.getHeading(), path7.getHeading())
                .build();

        paths8 = follower.pathBuilder()
                .addPath(new BezierCurve(path7, path8))
                .setLinearHeadingInterpolation(path7.getHeading(), path8.getHeading())
                .build();

        paths9 = follower.pathBuilder()
                .addPath(new BezierCurve(path8, path9))
                .setLinearHeadingInterpolation(path8.getHeading(), path9.getHeading())
                .build();

        paths10 = follower.pathBuilder()
                .addPath(new BezierCurve(path9, path10))
                .setLinearHeadingInterpolation(path9.getHeading(), path10.getHeading())
                .build();

        paths11 = follower.pathBuilder()
                .addPath(new BezierCurve(path10, path11))
                .setLinearHeadingInterpolation(path10.getHeading(), path11.getHeading())
                .build();


        paths15 = follower.pathBuilder()
                .addPath(new BezierCurve(path14, path15))
                .setLinearHeadingInterpolation(path14.getHeading(), path15.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (pathTimer.getElapsedTimeSeconds() > 23) {
                    follower.followPath(scorePreload);
                    setPathState(1);
                    launcher1.setPower(0.68);
                    launcher2.setPower(-0.68);
                    flip1.setPosition(0.5);
                }
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                if(!follower.isBusy()) {
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

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(paths1);
                    setPathState(2);

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
    /*
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Error is just tx straight from Limelight
            double error = result.getTx();

            // Basic PID
            integral += error;
            double derivative = error - lastError;

            double power = P * error + I * integral + D * derivative;

            // Apply power directly (no deadzone, no clamping)
            turret.setPower(power);

            lastError = error;

            telemetry.addData("tx", error);
            telemetry.addData("power", power);

        } else {
            // No target -> stop motor
            turret.setPower(0);
            telemetry.addLine("No Target");
        }

        telemetry.update();

*/
    }


    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        launcher1 = hardwareMap.get(DcMotor.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotor.class, "launcher2");
        flip1 = hardwareMap.get(Servo.class,"flip1");
        intake = hardwareMap.get(DcMotor.class,"intake");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        limelight.start();
        limelight.pipelineSwitch(0);
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}

