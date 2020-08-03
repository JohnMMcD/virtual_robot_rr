package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import virtual_robot.controller.VirtualRobotController;

/**
 * Example OpMode. Controls robot using left joystick, with arcade drive.
 */

@TeleOp(name = "RangerTankDrive", group = "TwoWheel")
public class RangerTankDrive extends LinearOpMode {
    private VirtualRobotController.ColorSensorImpl colorSensor = null;

    public void runOpMode(){

        DcMotor left = hardwareMap.dcMotor.get("left_motor");
        DcMotor right = hardwareMap.dcMotor.get("right_motor");
        colorSensor = (VirtualRobotController.ColorSensorImpl)hardwareMap.colorSensor.get("color_sensor");
        left.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Press Start When Ready","");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            double leftPower;
            double rightPower;
            if (gamepad1.y)
            {
                leftPower = 1.0;
                rightPower = 1.0;
            }
            else if (gamepad1.a)
            {
                leftPower = -1.0;
                rightPower = -1.0;
            }
            else if (Math.abs(gamepad1.left_stick_y) < 0.05 || Math.abs(gamepad1.left_stick_x) < 0.05)
            {
                leftPower = 0.0;
                rightPower = 0.0;
            }
            else
            {
                leftPower = -gamepad1.left_stick_y;
                rightPower = gamepad1.left_stick_x;
            }
            left.setPower(leftPower);
            right.setPower(rightPower);
            telemetry.addData("Power", String.format("L: %.2f R: %.2f", leftPower, rightPower));
            telemetry.addData("Color", String.format("R:%2d G: %2d B: %2d", colorSensor.red(), colorSensor.green(), colorSensor.blue()));
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
    }
}

