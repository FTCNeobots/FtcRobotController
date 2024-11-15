package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name = "auto")
public class Auto extends LinearOpMode {

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    double countsPerRevolution = 28 * 15;
    double cmPerRevolution = 31.4;
    double ticksPerCm = countsPerRevolution/cmPerRevolution;
    double power = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        leftBackDrive = hardwareMap.dcMotor.get("LBD");
        leftFrontDrive = hardwareMap.dcMotor.get("LFD");
        rightBackDrive = hardwareMap.dcMotor.get("RBD");
        rightFrontDrive = hardwareMap.dcMotor.get("RFD");

        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //according to REV 28 per revolution, *15 because of gearbox



        waitForStart();
        Strafe(-10);
        Strafe(20);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(5000);
    }


    public void Strafe(double _cm){
        int _target = (int) (_cm * cmPerRevolution);
        telemetry.addData("target: ", _target);
        telemetry.update();
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackDrive.setTargetPosition(_target);
        leftFrontDrive.setTargetPosition(- _target);
        rightBackDrive.setTargetPosition(- _target);
        rightFrontDrive.setTargetPosition(_target);

        leftBackDrive.setPower(power);
        leftFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(power);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);



    }
}
