package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedroPathing.tuningAndConstants.Constants;

@Autonomous(name = "Auto Selector")
public class AutoSelector extends OpMode {

    enum Alliance { BLUE, RED }
    enum StartPos { TOP, BOTTOM }

    Alliance alliance = Alliance.BLUE;
    StartPos startPos = StartPos.BOTTOM;

    Follower follower;

    Servo flip1;
    DcMotor intake, launcher1, launcher2;

    AutoBottomBlue bottomBlueAuto;
    AutoBottomRed bottomRedAuto;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        flip1 = hardwareMap.get(Servo.class, "flip1");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher1 = hardwareMap.get(DcMotor.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotor.class, "launcher2");

        bottomBlueAuto = new AutoBottomBlue(follower, flip1, intake, launcher1, launcher2);
        bottomRedAuto = new AutoBottomRed(follower, flip1, intake, launcher1, launcher2);

    }

    @Override
    public void init_loop() {
        if (gamepad1.x) alliance = Alliance.BLUE;
        if (gamepad1.b) alliance = Alliance.RED;

        if (gamepad1.a) startPos = StartPos.BOTTOM;
        if (gamepad1.y) startPos = StartPos.TOP;

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Start Position", startPos);
        telemetry.update();
    }

    @Override
    public void start() {
        if (alliance == Alliance.BLUE && startPos == StartPos.BOTTOM) {
            bottomBlueAuto.start();
        }
        if (alliance == Alliance.RED && startPos == StartPos.BOTTOM) {
            bottomRedAuto.start();
        }
    }

    @Override
    public void loop() {
        if (alliance == Alliance.BLUE && startPos == StartPos.BOTTOM) {
            bottomBlueAuto.update();
        }
        if (alliance == Alliance.RED && startPos == StartPos.BOTTOM) {
            bottomRedAuto.update();
        }
    }
}
