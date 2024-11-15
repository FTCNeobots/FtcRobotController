package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Bakje")
public class Bakje extends LinearOpMode {


    private DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.dcMotor.get("test");

        waitForStart();

        while (opModeIsActive()){
            /*if(gamepad1.a){
                motor.setPower(1);
            }else if(gamepad1.b){
                motor.setPower(-1);
            }else{
                motor.setPower(0);
            }*/
            motor.setPower(gamepad1.left_stick_x + 0.5 * gamepad1.right_stick_x);

        }

    }
}
