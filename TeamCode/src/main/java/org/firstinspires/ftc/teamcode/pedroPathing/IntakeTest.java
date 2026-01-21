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
public class IntakeTest extends OpMode {


    public DcMotorEx intake;

    public static double intakeVelocity = 1000;


    double curTargetVelocity = intakeVelocity;

    static double F = 12.8;
    static double P = 30;
    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {



        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        intake.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        intake.setVelocity(curTargetVelocity);

        double curVelocity = intake.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.update();

    }
}