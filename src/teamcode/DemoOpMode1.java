package teamcode;

import hardware.ColorSensor;
import hardware.DCMotor;
import hardware.GyroSensor;
import opmode.LinearOpMode;
import time.ElapsedTime;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
public class DemoOpMode1 extends LinearOpMode {

    public void runOpMode(){
        DCMotor left = hardwareMap.dcMotor.get("left_motor");
        DCMotor right = hardwareMap.dcMotor.get("right_motor");
        left.setDirection(DCMotor.Direction.REVERSE);
        GyroSensor gyro = hardwareMap.gyroSensor.get("gyro_sensor");
        gyro.init();
        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
        telemetry.addData("Press Start When Ready","");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            if (gamePad1.a){
                telemetry.addData("a pressed","");
                left.setPower(.5);
                right.setPower(.5);
            } else if (gamePad1.b){
                telemetry.addData("b pressed", "");
                left.setPower(-.5);
                right.setPower(.5);
            } else {
                left.setPower(0);
                right.setPower(0);
            }
            telemetry.addData("Press A to drive forward, \n B to rotate","");
            telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
            telemetry.addData("Heading"," %.1f", gyro.getHeading());
            telemetry.addData("Encoders","Left %d  Right %d", left.getCurrentPosition(), right.getCurrentPosition());
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
    }
}