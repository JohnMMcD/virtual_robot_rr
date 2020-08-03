package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import virtual_robot.controller.VirtualRobotController;

/**
 * Example OpMode. Controls robot using left joystick, with arcade drive.
 */

@TeleOp(name = "_RangerTankDrive", group = "")
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
            else if (gamepad1.x)
            {
                leftPower = 0.5;
                rightPower = -0.5;
            }
            else if (gamepad1.b)
            {
                leftPower = -0.5;
                rightPower = 0.5;
            }
            else
            {
                leftPower = 0.0;
                rightPower = 0.0;
            }
            left.setPower(leftPower);
            right.setPower(rightPower);
            //TODO: telemetry.addData("X and Y", String.format("X:%.2f Y:%.2f", 0.0, 0.0 ));
            telemetry.addData("Power", String.format("L: %.2f R: %.2f", leftPower, rightPower));
            telemetry.addData("Color", String.format("R:%2d G: %2d B: %2d", colorSensor.red(), colorSensor.green(), colorSensor.blue()));
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
    }
}

