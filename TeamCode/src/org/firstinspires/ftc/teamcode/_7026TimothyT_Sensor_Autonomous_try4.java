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

    left = hardwareMap.dcMotor.get("left");
//    touch1 = hardwareMap.touchSensor.get("touch1");
    touch1 = new TouchSensor();
    right = hardwareMap.dcMotor.get("right");
    color1 = hardwareMap.colorSensor.get("color1");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      left.setDirection(DcMotorSimple.Direction.REVERSE);
      // Put run blocks here.
    }
    while (!touch1.isPressed(gamepad1)) {
      left.setPower(0.4);
      right.setPower(0.4);
    }
    // Back up after hitting wal
    left.setPower(-0.5);
    right.setPower(-0.5);
    sleep(333);
    // Turn left  appx 90 degrees- OK
    left.setPower(-0.5);
    right.setPower(0.5);
    sleep(1000);
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
    // Red has been reached, start turning
    left.setPower(0.5);
    right.setPower(-0.5);
    sleep(1600);
    while (!touch1.isPressed(gamepad1)) {
      left.setPower(0.4);
      right.setPower(0.4);
    }
    left.setPower(-1);
    right.setPower(-1);
    sleep(500);
    left.setPower(-0.5);
    right.setPower(0.5);
    sleep(2000);
    while (!touch1.isPressed(gamepad1)) {
      left.setPower(0.1);
      right.setPower(0.1);
    }
    // CurrentColor = Color.argb(color1.alpha(), color1.red(), color1.green(), color1.blue());
    // JMM: Back up towards red
    while (!isRed(color1)) {
      left.setPower(-0.2);
      right.setPower(-0.2);
//      CurrentColor = Color.argb(color1.alpha(), color1.red(), color1.green(), color1.blue());
    }
    left.setPower(0);
    right.setPower(0);
  }

  private boolean isRed(ColorSensor colorSensor)
  {
    return colorSensor.red() > RED_LINE_DETECTION;
  }

  private boolean isBlue(ColorSensor colorSensor)
  {
    return colorSensor.blue() > BLUE_LINE_DETECTION;
  }
}
