package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class TeleOp1 extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    DcMotor IntakeMotor;
    DcMotor LauncherMotor;
    Servo Gate_Servo;
    int IntakeFlag = 0;
    int LauncherFlag = 0;
    int GateFlag = 0;
    private Limelight3A limelight;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose(41.02679830747532, 59.712270803949224, 180) : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(70.07052186177715, 66.00846262341325))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(130), 0.8))
                .build();
        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        limelight = hardwareMap.get(Limelight3A.class,"LimeLight");
        LauncherMotor = hardwareMap.get(DcMotor.class, "LauncherMotor");
        Gate_Servo = hardwareMap.get(Servo.class, "Gate_SERVO");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        Gate_Servo.setPosition(0.5);
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        limelight.start();
    }

    @Override
    public void loop() {


        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }

       // Automated PathFollowing
          if (gamepad1.aWasPressed()) {
               follower.followPath(pathChain.get());
               automatedDrive = true;
           }

        //Stop automated following if the follower is done
            if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
               follower.startTeleopDrive();
              automatedDrive = false;
           }


        if (gamepad1.xWasPressed()) {
            if (IntakeFlag == 0) {
                IntakeMotor.setPower(1);
                IntakeFlag = 1;
            }
            else if (IntakeFlag == 1) {
                IntakeMotor.setPower(0);
                IntakeFlag = 0;
            } else if (IntakeFlag == -1) {
                IntakeMotor.setPower(0);
                IntakeFlag = 0;
            }
        }

        //Intake reverse

        if (gamepad1.yWasPressed()) {
            if (IntakeFlag == 0) {
                IntakeMotor.setPower(-1);
                IntakeFlag = -1;
            } else if (IntakeFlag == -1) {
                IntakeMotor.setPower(0);
                IntakeFlag = 0;
            } else if (IntakeFlag == 1) {
                IntakeMotor.setPower(0);
                IntakeFlag = 0;
            }
        }

        if (gamepad2.dpadUpWasPressed()) {
            if (LauncherFlag == 0) {
                LauncherMotor.setPower(-1);
                LauncherFlag = 1;
            }
            else if (LauncherFlag == 1) {
                LauncherMotor.setPower(0);
                LauncherFlag = 0;
            }
        }
        if (gamepad2.dpadDownWasPressed()) {
            if (LauncherFlag == 0) {
                LauncherMotor.setPower(-0.75);
                LauncherFlag = 1;
            }
            else if (LauncherFlag == 1) {
                LauncherMotor.setPower(0);
                LauncherFlag = 0;
            }
        }
        if (gamepad2.aWasPressed()) {
            if (GateFlag == 0) {
                Gate_Servo.setPosition(0.35);
                GateFlag = 1;
            } else if (GateFlag == 1) {
                Gate_Servo.setPosition(0.5);
                GateFlag = 0;
            }
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

    }
}