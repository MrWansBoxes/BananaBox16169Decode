package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//@TeleOp
//@Configurable
public class LauncherTest extends OpMode {


    public DcMotorEx launcher1, launcher2;

    public double highVelocity = 2400;
    public double lowVelocity = 1700;

double curTargetVelocity = highVelocity;

static double F = 12.8;
static double P = 30;
    @Override
    public void init() {
launcher1 = hardwareMap.get(DcMotorEx.class,"launcher1");
launcher2 = hardwareMap.get(DcMotorEx.class,"launcher2");
launcher1.setDirection(DcMotorSimple.Direction.FORWARD);
launcher2.setDirection(DcMotorSimple.Direction.REVERSE);
PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
launcher1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
launcher2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {


if (gamepad1.yWasPressed()){
    if (curTargetVelocity == highVelocity){
        curTargetVelocity = lowVelocity;
    }
    else if (curTargetVelocity == lowVelocity){
        curTargetVelocity = highVelocity;
    }
}

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        launcher1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        launcher2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        launcher1.setVelocity(curTargetVelocity);
        launcher2.setVelocity(curTargetVelocity);

        double curVelocity = launcher1.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.update();

    }
}
