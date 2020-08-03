package virtual_robot.controller.robots.classes;

import com.qualcomm.robotcore.hardware.*;
import javafx.fxml.FXML;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.util.AngleUtils;

/**
 * For internal use only. Represents a robot with two standard wheels in the back, color sensor, four distance sensors,
 * a Gyro Sensor, and a Servo-controlled arm on the back. The physical Ranger Bot has two unpowered omni wheels in the front 
 * and a platform, all of which are not modeled. See https://youtu.be/VPO0XH40pzk or
 * https://www.tetrixrobotics.com/rcbuildersguide/files/resources/Print_All_TETRIX_BG.pdf
 * RangerBot is the controller class for the "ranger_bot.fxml" markup file. 
 * The RangerBot class is based on the Two Wheel Bot class. It has not been fully updated.
 */
@BotConfig(name = "_Ranger Bot", filename = "ranger_bot")
public class RangerBot extends VirtualBot {

    private MotorType motorType;
    private DcMotorImpl leftMotor;
    private DcMotorImpl rightMotor;
    private GyroSensorImpl gyro;
    private VirtualRobotController.ColorSensorImpl colorSensor;
    private ServoImpl servo;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors;

    //The backServoArm object is instantiated during loading via a fx:id property.
    @FXML Rectangle backServoArm;

    private double wheelCircumference;
    private double interWheelDistance;
    private double robotLength;
    private double halfRobotLength;
    private double tau;
    private double wheelDiameter;
    private double wheelRadius;

    public RangerBot(){
        super();
        leftMotor = (DcMotorImpl)hardwareMap.dcMotor.get("left_motor");
        rightMotor = (DcMotorImpl)hardwareMap.dcMotor.get("right_motor");
        distanceSensors = new VirtualRobotController.DistanceSensorImpl[]{
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "front_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "left_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "back_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "right_distance")
        };
        gyro = (GyroSensorImpl)hardwareMap.gyroSensor.get("gyro_sensor");
        colorSensor = (VirtualRobotController.ColorSensorImpl)hardwareMap.colorSensor.get("color_sensor");
        servo = (ServoImpl)hardwareMap.servo.get("back_servo");
// Original values
//        wheelCircumference = Math.PI * botWidth / 4.5;
//        interWheelDistance = botWidth * 8.0 / 9.0;

        /* Updated with values from RangerBot */
        tau = Math.PI * 2.0;
        wheelDiameter = 3.0;
        wheelRadius = wheelDiameter / 2;
        wheelCircumference = tau * wheelRadius;
        interWheelDistance = 8.0;

        robotLength = 18.0; // Actual is 10 or so, but we're going to use 18 to get the color sensor in the apparent front of the bot
        halfRobotLength = robotLength / 2.0;
    }

    public void initialize(){
        backServoArm.getTransforms().add(new Rotate(0, 37.5, 67.5));
    }

    protected void createHardwareMap(){
        motorType = MotorType.Neverest40;
        hardwareMap = new HardwareMap();
        hardwareMap.put("left_motor", new DcMotorImpl(motorType));
        hardwareMap.put("right_motor", new DcMotorImpl(motorType));
        String[] distNames = new String[]{"front_distance", "left_distance", "back_distance", "right_distance"};
        for (String name: distNames) hardwareMap.put(name, controller.new DistanceSensorImpl());
        hardwareMap.put("gyro_sensor", new GyroSensorImpl(this, 10));
        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());
        hardwareMap.put("back_servo", new ServoImpl());
    }

    /** TODO: Update with values from RangerBot */
    public synchronized void updateStateAndSensors(double millis){
        double deltaLeftPos = leftMotor.update(millis);
        double deltaRightPos = rightMotor.update(millis);
        double leftWheelDist = -deltaLeftPos * wheelCircumference / motorType.TICKS_PER_ROTATION;
        double rightWheelDist = deltaRightPos * wheelCircumference / motorType.TICKS_PER_ROTATION;
        double distTraveled = (leftWheelDist + rightWheelDist) / 2.0;
        
        double headingChange = (rightWheelDist - leftWheelDist) / interWheelDistance; // Adjust for wheels in back of robot
        double deltaRobotX = -distTraveled * Math.sin(headingRadians + headingChange / 2.0);
        double deltaRobotY = distTraveled * Math.cos(headingRadians + headingChange / 2.0);
        x += deltaRobotX;
        y += deltaRobotY;
        if (x >  (halfFieldWidth - halfBotWidth)) x = halfFieldWidth - halfBotWidth;
        else if (x < (halfBotWidth - halfFieldWidth)) x = halfBotWidth - halfFieldWidth;
        if (y > (halfFieldWidth - halfBotWidth)) y = halfFieldWidth - halfBotWidth;
        else if (y < (halfBotWidth - halfFieldWidth)) y = halfBotWidth - halfFieldWidth;
        headingRadians += headingChange;
        if (headingRadians > Math.PI) headingRadians -= tau;
        else if (headingRadians < -Math.PI) headingRadians += tau;
        gyro.updateHeading(headingRadians * 180.0 / Math.PI);

        // The X,Y origin is in the center of the field with positive X in the direction of the red alliance
        // and positive Y in the direction of the scoring table.
        // The heading is 0 degrees in the default orientation. As the robot rotates a half turn, the heading
        // increases to up to pi (tau/2) radians, then when it crosses a half-turn it flips to negative pi (tau/2) radians.
        // The color sensor is mounted at the edge of the robot. In the default orientation, the
        // color sensor is mounted on the edge closest to the scoring table.
        // TODO: Determine why the color sensor seems to always report the value under the center of the robot
        double colorX = x - (Math.sin(headingRadians) * halfRobotLength);
        double colorY = y + (Math.cos(headingRadians) * halfRobotLength);
        System.out.println(String.format("Debug: x: %.2f y:%.2f colorX:%.2f colorY:%.2f Heading:%.2f", x, y, colorX, colorY, headingRadians));
        colorSensor.updateColor(colorX, colorY);
        // When you zero out the location of the color sensor, the color value never changes
        //colorSensor.updateColor(0,0);

        final double tauOver4 = tau / 4.0;
        for (int i = 0; i<4; i++){
            double sensorHeading = AngleUtils.normalizeRadians(headingRadians + i * tauOver4);
            distanceSensors[i].updateDistance( x - halfBotWidth * Math.sin(sensorHeading),
                    y + halfBotWidth * Math.cos(sensorHeading), sensorHeading);
        }
    }

    public synchronized void updateDisplay(){
        super.updateDisplay();
        ((Rotate)backServoArm.getTransforms().get(0)).setAngle(-180.0 * servo.getInternalPosition());
    }

    public void powerDownAndReset(){
        leftMotor.stopAndReset();
        rightMotor.stopAndReset();
        gyro.deinit();
    }


}
