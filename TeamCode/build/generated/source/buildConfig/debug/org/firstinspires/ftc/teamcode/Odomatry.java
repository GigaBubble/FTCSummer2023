package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Odomatry {

    public static final double TRIGGER_THREASHOLD = 0.5;
    public ElapsedTime pathTimer;
    // DriveTrain Motors
    private ColorSensor colorDistance;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DistanceSensor colorDistanceDistanceSensor;

    // Odomatry Encoders
    public DcMotor encoderLeft;
    public DcMotor encoderRight;
    public DcMotor encoderAux;

    private HardwareMap hardwareMap;

    public Odomatry(HardwareMap hardwareMap) {
    }

    public void odometry(HardwareMap hardwareMap) {

        HardwareMap aHardwareMap = null;
        hardwareMap = aHardwareMap;

        // Configure the Drive Motors
        frontLeft = hardwareMap.dcMotor.get("front_left");
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft = hardwareMap.dcMotor.get("back_left");
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight = hardwareMap.dcMotor.get("front_right");
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRight = hardwareMap.dcMotor.get("back_right");
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Odometry Encoders
        encoderRight = backRight;
        encoderLeft = backLeft;
        encoderAux = frontRight;

        stop();
        resetDriveEncoders();
    }

    public void resetDriveEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stop() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    final static double L = 12; // Distance between encoder 1 and 2 in cm
    final static double B = 11.5; // Distance between the midpoint of encoder 1 and 2 and 3
    final static double R = 3; // Wheel Radius in CM
    final static double N = 8192; // Encoder tick per revolution
    final static double cm_per_tick = 2.0 * Math.PI * R / N;

    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentAuxPosition = 0;

    private int oldRightPosition = 0;
    private int oldLeftPosition = 0;
    private int oldAuxPosition = 0;
    
    public XyhVector START_POS = new XyhVector(213, 102, Math.toRadians(-174));
    public XyhVector pos = new XyhVector(START_POS);

    public void odometry() {
        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldAuxPosition = currentAuxPosition;

        currentRightPosition = -encoderRight.getCurrentPosition();
        currentLeftPosition = encoderLeft.getCurrentPosition();
        currentAuxPosition = encoderAux.getCurrentPosition();

        int dn1 = currentLeftPosition - oldLeftPosition;
        int dn2 = currentRightPosition - oldRightPosition;
        int dn3 = currentAuxPosition - oldAuxPosition;

        // The robot has moved and turned a tiny bit between two measurements:
        double dtheta = cm_per_tick * ((dn2 - dn1) / L);
        double dx = cm_per_tick * ((dn1 + dn2) / 2.0);
        double dy = cm_per_tick * (dn3 + ((dn2 - dn1) / 2.0));

        double telemetrydx = dx;
        double telemetrydy = dy;
        double telemetrydh = dtheta;

        // Small movement of the robot gets added to the field coordinate system:
        pos.h += dtheta / 2;
        pos.x += dx * Math.cos(pos.h) - dy * Math.sin(pos.h);
        pos.y += dx * Math.sin(pos.h) + dy * Math.cos(pos.h);
        pos.h += dtheta / 2;
        pos.h = normDiff(pos.h);
    }

    private double normDiff(double h) {
        return h;
    }
}
