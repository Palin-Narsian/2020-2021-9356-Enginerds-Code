package org.firstinspires.ftc.teamcode.onewheel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.onewheel.Angle;
import org.firstinspires.ftc.teamcode.onewheel.DriveModule;
import org.firstinspires.ftc.teamcode.onewheel.Robot;
import org.firstinspires.ftc.teamcode.onewheel.RobotUtil;
import org.firstinspires.ftc.teamcode.onewheel.Vector2d;

enum ModuleSide {LEFT}

public class DriveController {

    Robot robot;
    DriveModule moduleLeft;
    DriveModule moduleRight;

    //used for straight line distance tracking
    double robotDistanceTraveled = 0;
    double previousRobotDistanceTraveled = 0;
    double moduleLeftLastDistance;
    double moduleRightLastDistance;

    //tolerance for module rotation (in degrees)
    public final double ALLOWED_MODULE_ROT_ERROR = 5;

    //distance from target when power scaling will begin
    public final double START_DRIVE_SLOWDOWN_AT_CM = 15;

    //maximum number of times the robot will try to correct its heading when rotating
    public final int MAX_ITERATIONS_ROBOT_ROTATE = 2;

    //will multiply the input from the rotation joystick (max value of 1) by this factor
    public final double ROBOT_ROTATION_SCALE_FACTOR = 0.7;

    public DriveController(Robot robot) {
        this.robot = robot;
        moduleLeft = new DriveModule(robot, ModuleSide.LEFT);

        moduleLeftLastDistance = moduleLeft.getDistanceTraveled();

    }

    //converts joystick vectors to parameters for update() method
    //called every loop cycle in TeleOp
    public void updateUsingJoysticks(Vector2d joystick1, Vector2d joystick2) {
        update(joystick1, -joystick2.getX() * ROBOT_ROTATION_SCALE_FACTOR);
    }

    //should be called every loop cycle when driving (auto or TeleOp)
    //note: positive rotationMagnitude is CCW rotation
    public void update(Vector2d translationVector, double rotationMagnitude) {
        moduleLeft.updateTarget(translationVector, rotationMagnitude);
    }

    //AUTONOMOUS METHODS
    //do NOT call in a loop

    //speed should be scalar from 0 to 1
    public void drive(Vector2d direction, double cmDistance, double speed, LinearOpMode linearOpMode) {
        //turns modules to correct positions for straight driving
        //rotateModules()
        resetDistanceTraveled();
        while (getDistanceTraveled() < cmDistance && linearOpMode.opModeIsActive()) {
            //slows down drive power in certain range
            if (cmDistance - getDistanceTraveled() < START_DRIVE_SLOWDOWN_AT_CM) {
                //speed = RobotUtil.scaleVal(cmDistance - getDistanceTraveled(), 0, START_DRIVE_SLOWDOWN_AT_CM, 0.1, 1);
            }
            updateTracking();
            update(direction.normalize(speed), 0);

            linearOpMode.telemetry.addData("Driving robot", "");
            linearOpMode.telemetry.update();
        }
        update(Vector2d.ZERO, 0);
    }




    //TRACKING METHODS
    //methods for path length tracking in autonomous (only useful for driving in straight lines)

    public void resetDistanceTraveled() {
        previousRobotDistanceTraveled = robotDistanceTraveled;
        robotDistanceTraveled = 0;
        moduleLeft.resetDistanceTraveled();
    }

    public void updateTracking() {
        moduleLeft.updateTracking();

        double moduleLeftChange = moduleLeft.getDistanceTraveled() - moduleLeftLastDistance;
        robotDistanceTraveled += (moduleLeftChange) / 2;

        moduleLeftLastDistance = moduleLeft.getDistanceTraveled();
    }

    //note: returns ABSOLUTE VALUE
    public double getDistanceTraveled() {
        return Math.abs(robotDistanceTraveled);
    }

    public void resetEncoders() {
        moduleLeft.resetEncoders();
    }
}