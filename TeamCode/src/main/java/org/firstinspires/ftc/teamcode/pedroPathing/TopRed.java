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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "TopRed", group = "Examples")
public class TopRed extends OpMode {


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    DcMotor IntakeMotor;
    DcMotor LauncherMotor;
    Servo Gate_Servo;



    private final Pose startPose = new Pose(105.00423131170663, 134.86036671368123, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose shootPose1 = new Pose(75.35119887165021, 65.39915373765866, Math.toRadians(51)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(102.56699576868829, 83.88152327221438, Math.toRadians(360)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(121.4555712270804, 83.88152327221438, Math.toRadians(360)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose shootPose2 = new Pose(75.148095909732, 65.39915373765866, Math.toRadians(51)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose lineupPose = new Pose(102.56699576868829, 83.88152327221438, Math.toRadians(360)); // Lowest (Third Set) of Artifacts from the Spike Mark.


    private Path StarttoShoot;
    private PathChain Shoot1, Pickup1, Pickup2, Shoot2;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        StarttoShoot = new Path(new BezierCurve(startPose, shootPose1));
        StarttoShoot.setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Shoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, pickup1Pose))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickup2Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, shootPose2))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), shootPose2.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Shoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, lineupPose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), lineupPose.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                LauncherMotor.setPower(-0.8);
                follower.followPath(StarttoShoot);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    sleep(2000);
                    Gate_Servo.setPosition(0.35);
                    IntakeMotor.setPower(-1);
                    sleep(200);
                    IntakeMotor.setPower(0);
                    sleep(2500);
                    IntakeMotor.setPower(0.5);
                    sleep(250);
                    IntakeMotor.setPower(-1);
                    sleep(1000);
                    Gate_Servo.setPosition(0.5);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Shoot1,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(Pickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    sleep(750);
                    IntakeMotor.setPower(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Pickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    sleep(2000);
                    Gate_Servo.setPosition(0.35);
                    IntakeMotor.setPower(-1);
                    sleep(200);
                    IntakeMotor.setPower(0);
                    sleep(2500);
                    IntakeMotor.setPower(0.5);
                    sleep(250);
                    IntakeMotor.setPower(-1);
                    sleep(1000);
                    Gate_Servo.setPosition(0.5);
                    sleep(1000);
                    LauncherMotor.setPower(0);
                    IntakeMotor.setPower(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(Shoot2,true);
                    setPathState(5);
                }
                break;
            case 5:

        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
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

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Gate_Servo = hardwareMap.get(Servo.class, "Gate_SERVO");
        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        LauncherMotor = hardwareMap.get(DcMotor.class, "LauncherMotor");

        Gate_Servo.setPosition(0.35);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}
