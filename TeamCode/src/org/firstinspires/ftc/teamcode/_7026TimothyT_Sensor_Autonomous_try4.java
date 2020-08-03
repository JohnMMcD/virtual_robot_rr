package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.TouchSensor;
// import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Autonomous(name = "_7026TimothyT_Sensor_Autonomous_try4 (Blocks to Java)", group = "")
public class _7026TimothyT_Sensor_Autonomous_try4 extends LinearOpMode {
  private final int BLUE_LINE_DETECTION = 200;
  private final int RED_LINE_DETECTION = 100; // TODO: Fix

  private DcMotor left;
  private TouchSensor touch1;
  private DcMotor right;
  private ColorSensor color1;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
//    int CurrentColor;

    left = hardwareMap.dcMotor.get("left_motor");
//    touch1 = hardwareMap.touchSensor.get("touch1");
    touch1 = new TouchSensor();
    right = hardwareMap.dcMotor.get("right_motor");
    color1 = hardwareMap.colorSensor.get("color_sensor");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      left.setDirection(DcMotorSimple.Direction.REVERSE);
      // Put run blocks here.
    }
    while (!touch1.isPressed(gamepad1) && opModeIsActive()) {
      left.setPower(0.4);
      right.setPower(0.4);
    }
    // Back up after hitting wall
    left.setPower(-0.5);
    right.setPower(-0.5);
    sleep(333);

    // JMM added stop and check for button press
    while (!gamepad1.b && opModeIsActive()) {
      left.setPower(0);
      right.setPower(0);
    }

    // Turn left appx 90 degrees- OK
    left.setPower(-0.5);
    right.setPower(0.5);
    sleep(turn90Sleep()); // JMM decreased from 1000 due to slight overshoot

    // JMM added stop and check for button press
    while (!gamepad1.b && opModeIsActive()) {
        left.setPower(0);
        right.setPower(0);
    }

    // Go forward until the red line
    left.setPower(0.3);
    right.setPower(0.3);
//    CurrentColor = Color.argb(color1.alpha(), color1.red(), color1.green(), color1.blue());
//    while (!(JavaUtil.colorToSaturation(CurrentColor) > 0.4 && (JavaUtil.colorToHue(CurrentColor) >= 330 || JavaUtil.colorToHue(CurrentColor) <= 30))) {
    while (!isRed(color1)) {
      left.setPower(0.3);
      right.setPower(0.3);
//      CurrentColor = Color.argb(color1.alpha(), color1.red(), color1.green(), color1.blue());
    }

    // JMM added stop and check for button press
    while (!gamepad1.b && opModeIsActive()) {
        left.setPower(0);
        right.setPower(0);
    }

    // Red has been reached, start turning
    left.setPower(-0.5);
    right.setPower(0.5);
    sleep(turn90Sleep());

  // JMM added stop and check for button press
  while (!gamepad1.b && opModeIsActive()) {
      left.setPower(0);
      right.setPower(0);
  }

    // advance forward and hit the block
    while (!touch1.isPressed(gamepad1)&& opModeIsActive()) {
      left.setPower(0.4);
      right.setPower(0.4);
    }

    // JMM added stop and check for button press
    while (!gamepad1.b && opModeIsActive()) {
        left.setPower(0);
        right.setPower(0);
    }
    // back up after hitting the block
    left.setPower(-.4);
    right.setPower(-4);
    sleep(500);

    // JMM added stop and check for button press
    while (!gamepad1.b && opModeIsActive()) {
        left.setPower(0);
        right.setPower(0);
    }

    left.setPower(-0.5);
    right.setPower(0.5);
    sleep(turn90Sleep() + 100);

    // JMM added stop and check for button press
    while (!gamepad1.b && opModeIsActive()) {
        left.setPower(0);
        right.setPower(0);
    }

    while (!touch1.isPressed(gamepad1) && opModeIsActive()) {
        left.setPower(0.1);
        right.setPower(0.1);
    }

      // JMM added stop and check for button press
      while (!gamepad1.b && opModeIsActive()) {
          left.setPower(0);
          right.setPower(0);
      }
    // CurrentColor = Color.argb(color1.alpha(), color1.red(), color1.green(), color1.blue());
    // JMM: Back up towards red
    while (!isRed(color1)&& opModeIsActive()) {
        left.setPower(-0.2);
        right.setPower(-0.5);
//      CurrentColor = Color.argb(color1.alpha(), color1.red(), color1.green(), color1.blue());
    }
    left.setPower(0);
    right.setPower(0);
  }

  /**
   * Returns true if the color sensor is over the red line.
   * */
  private boolean isRed(ColorSensor colorSensor)
  {
      return colorSensor.red() > 135 && colorSensor.green() < 95;
  }

  /**
   * Returns true if the color sensor is over the blue line.
   * */
  private boolean isBlue(ColorSensor colorSensor)
  {
      return colorSensor.red() < 70 && colorSensor.blue() > 160;
  }

  private int turn90Sleep()
  {
      return 875;
  }
}

