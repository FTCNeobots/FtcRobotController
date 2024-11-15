package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "FieldCentricDrive")
public class FieldCentricDrive extends LinearOpMode {

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    private double maxSpeed = 1;
    private double botHeading;



    @Override
    public void runOpMode() throws InterruptedException {
        leftBackDrive = hardwareMap.dcMotor.get("LBD");
        leftFrontDrive = hardwareMap.dcMotor.get("LFD");
        rightBackDrive = hardwareMap.dcMotor.get("RBD");
        rightFrontDrive = hardwareMap.dcMotor.get("RFD");

        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //dit hoort niet te hoeven, maar zo werkt het?
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();
        while (opModeIsActive()){
            NormalDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            SpeedControl();

            // This button choice was made so that it is hard to hit on accident,
            if (gamepad1.back) {
                imu.resetYaw();
                telemetry.addData("Yaw ", "reset!");
                telemetry.update();

            }
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        }


    }

    private void NormalDrive(double _Xget, double _Yget, double _Turnget){

        double _X = _Xget * Math.cos(botHeading) - _Yget * Math.sin(botHeading);
        double _Y = _Xget * Math.sin(botHeading) + _Yget * Math.cos(botHeading);
        double _Turn = 0.5 * _Turnget;
        _X = _X *1.1;

        double denominator = Math.max(Math.abs(_Y) + Math.abs(_X) + Math.abs(_Turn), maxSpeed);

        leftFrontDrive.setPower((_Y - _X + _Turn)*denominator);
        leftBackDrive.setPower((_Y + _X + _Turn)*denominator);
        rightBackDrive.setPower((_Y - _X - _Turn)*denominator);
        rightFrontDrive.setPower((_Y + _X - _Turn)*denominator);

    }
    private void SpeedControl(){
        if(gamepad1.left_bumper){
            maxSpeed = 0.5;
        }else if(gamepad1.right_bumper){
            maxSpeed = 1;
        }

    }


}
