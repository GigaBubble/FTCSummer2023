package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Color_Test (Blocks to Java)")
public class Color_Test extends LinearOpMode {

    private ColorSensor Color_Distance;
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;
    private DistanceSensor Color_Distance_DistanceSensor;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double Distance = 0;
        int Sensor = 0;
        int Finished = 0;

        Color_Distance = hardwareMap.get(ColorSensor.class, "Color_Distance");
        DcMotor front_left = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor front_right = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor back_left = hardwareMap.get(DcMotor.class, "back_left");
        DcMotor back_right = hardwareMap.get(DcMotor.class, "back_right");
        Color_Distance_DistanceSensor = hardwareMap.get(DistanceSensor.class, "Color_Distance");

        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left.setTargetPosition(0);
        back_left.setTargetPosition(0);
        front_right.setTargetPosition(0);
        back_right.setTargetPosition(0);

        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setPower(0.3);
        back_left.setPower(0.3);
        front_right.setPower(0.3);
        back_right.setPower(0.3);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        // Put initialization blocks here.
        waitForStart();

        back_left.setTargetPosition(504);
        back_right.setTargetPosition(504);
        front_left.setTargetPosition(-504);
        front_right.setTargetPosition(-504);

        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (front_left.isBusy()) ;

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        back_left.setTargetPosition(1035);
        back_right.setTargetPosition(1035);
        front_left.setTargetPosition(1035);
        front_right.setTargetPosition(1035);

        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (front_left.isBusy()) ;

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        back_left.setTargetPosition(-95);
        back_right.setTargetPosition(-95);
        front_left.setTargetPosition(95);
        front_right.setTargetPosition(95);

        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (front_left.isBusy()) ;

        Color_Distance.enableLed(true); // Enable LED before reading distance

        Distance = Color_Distance_DistanceSensor.getDistance(DistanceUnit.INCH);

        sleep(500);

        //if 1st block is there
        double Distance2 = -1;
        if (Distance < 3 && Sensor == 0) {
            Sensor = 10;
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            back_left.setTargetPosition(-659);
            back_right.setTargetPosition(-659);
            front_left.setTargetPosition(659);
            front_right.setTargetPosition(659);

            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (front_left.isBusy()) ;

            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            back_left.setTargetPosition(3856);
            back_right.setTargetPosition(3856);
            front_left.setTargetPosition(3856);
            front_right.setTargetPosition(3856);

            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (front_left.isBusy()) ;

            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            back_left.setTargetPosition(1296);
            back_right.setTargetPosition(1296);
            front_left.setTargetPosition(-1296);
            front_right.setTargetPosition(-1296);

            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (front_left.isBusy()) ;

            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            back_left.setTargetPosition(-1008);
            back_right.setTargetPosition(-1008);
            front_left.setTargetPosition(-1008);
            front_right.setTargetPosition(-1008);

            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (front_left.isBusy()) ;

            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            back_left.setTargetPosition(-1296);
            back_right.setTargetPosition(-1296);
            front_left.setTargetPosition(1296);
            front_right.setTargetPosition(1296);

            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (front_left.isBusy()) ;

        }
        //go from 1st block to 2nd block
        else if (Distance > 3 && Sensor == 0) {
            Sensor = 1;
            back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            back_left.setTargetPosition(-510);
            back_right.setTargetPosition(-510);
            front_left.setTargetPosition(510);
            front_right.setTargetPosition(510);

            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (front_left.isBusy()) ;

            sleep(500);
            Distance2 = -1;
            while (Distance2 == -1) {
                Color_Distance_DistanceSensor.getDistance(DistanceUnit.INCH);
                Distance2 = Color_Distance_DistanceSensor.getDistance(DistanceUnit.INCH);
            }

            //if 2nd block is there
            if (Distance2 < 3 && Sensor == 1 && Distance2 > 0) {
                Sensor = 20;
                front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                back_left.setTargetPosition(-162);
                back_right.setTargetPosition(-162);
                front_left.setTargetPosition(162);
                front_right.setTargetPosition(162);

                front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (front_left.isBusy()) ;

                front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                back_left.setTargetPosition(3856);
                back_right.setTargetPosition(3856);
                front_left.setTargetPosition(3856);
                front_right.setTargetPosition(3856);

                front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (front_left.isBusy()) ;

                front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                back_left.setTargetPosition(-1008);
                back_right.setTargetPosition(-1008);
                front_left.setTargetPosition(-1008);
                front_right.setTargetPosition(-1008);

                front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (front_left.isBusy()) ;

            }
            //2nd block to 3rd block and 3rd block
            else if (Distance2 > 3 && Sensor == 1) {
                Sensor = 30;
                front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                back_left.setTargetPosition(-162);
                back_right.setTargetPosition(-162);
                front_left.setTargetPosition(162);
                front_right.setTargetPosition(162);

                front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (front_left.isBusy()) ;

                front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                back_left.setTargetPosition(3856);
                back_right.setTargetPosition(3856);
                front_left.setTargetPosition(3856);
                front_right.setTargetPosition(3856);

                front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (front_left.isBusy()) ;

                front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                back_left.setTargetPosition(-1296);
                back_right.setTargetPosition(-1296);
                front_left.setTargetPosition(1296);
                front_right.setTargetPosition(1296);

                front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (front_left.isBusy()) ;

                front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                back_left.setTargetPosition(-1008);
                back_right.setTargetPosition(-1008);
                front_left.setTargetPosition(-1008);
                front_right.setTargetPosition(-1008);

                front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (front_left.isBusy()) ;

                front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                back_left.setTargetPosition(1296);
                back_right.setTargetPosition(1296);
                front_left.setTargetPosition(-1296);
                front_right.setTargetPosition(-1296);

                front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (front_left.isBusy()) ;

            }
        }

        telemetry.addData("Distance", Distance);
        telemetry.addData("Distance2", Distance2);
        telemetry.addData("Sensor", Sensor);
        telemetry.addData("Front Right Encoder Position", front_right.getCurrentPosition());
        telemetry.update();
        sleep(10000);
    }
}
