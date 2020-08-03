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
    private DcMotorImpl leftMotor = null;
    private DcMotorImpl rightMotor = null;
    private GyroSensorImpl gyro = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    private ServoImpl servo = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;

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

        robotLength = 10.0;
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
        double deltaLeftPos = -leftMotor.update(millis); // negative because left motor is reversed due to it being on the opposite side of the robot. I think.
        double deltaRightPos = rightMotor.update(millis);
        double leftWheelDist = deltaLeftPos * wheelCircumference / motorType.TICKS_PER_ROTATION;
        double rightWheelDist = deltaRightPos * wheelCircumference / motorType.TICKS_PER_ROTATION;
        double distTraveled = (leftWheelDist + rightWheelDist) / 2.0;
        
        double headingChange = (rightWheelDist - leftWheelDist) / interWheelDistance;
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

        // TODO: verify the color sensor reports the color underneath accurately
        double colorX = x + (Math.cos(headingRadians) * halfRobotLength);
        double colorY = y + (Math.sin(headingRadians) * halfRobotLength);
        System.out.println(String.format("Color sensor checks: colorX:%.2f colorY:%.2f",colorX, colorY ));
        colorSensor.updateColor(colorX, colorY);
        // FIXME: Let's see what happens when you zero out the location of the color sensor
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
