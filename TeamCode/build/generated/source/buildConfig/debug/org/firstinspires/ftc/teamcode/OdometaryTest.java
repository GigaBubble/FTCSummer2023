package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "OdometryTest", group = "TeleOp")
@Disabled
public class OdometaryTest extends LinearOpMode {
    private Odomatry robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Odomatry(hardwareMap);
        ElapsedTime timer = new ElapsedTime();
        robot.pathTimer = new ElapsedTime();

        waitForStart();
        timer.reset();
    }
    while (opModeIsActive()) {
        telemetry.addLine("Version 2.10");
        telemetry.addLine("x: reset, a: Target pos, B: goto zero");
        telemetry.update();

    }
}
