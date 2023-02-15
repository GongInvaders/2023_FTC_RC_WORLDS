package org.firstinspires.ftc.teamcode.teamUtil;

public class RobotConstants {

    public enum moduleSides {
        LEFT, RIGHT
    }

    public enum enabledModules{
        LEFT, RIGHT, BOTH
    }

    public enum configuredSystems {
        MECANUM,
        BOTH_MODULES,
        LEFT_MODULE,
        RIGHT_MODULE,
        LIFT,
        WRIST,
        INTAKE,
        ARM,
        ENCODER_READ,
        LIMIT_SWITCH,
        GAMEPADS;
    }

    public enum poleHeights {
        HIGH(-2750),
        MEDIUM(-1950),
        LOW(-1250),
        GROUND(0),
        HIGH_DROP(-2350),
        MEDIUM_DROP(-1750),
        LOW_DROP(-1000),
        STACK4(-350),
        STACK3(-260),
        STACK2(-170),
        STACK1(-110),
        STACK0(0),
        IDLE(0);

        poleHeights(int encoderValue){
            this.encoderValue = encoderValue;
        }

        private final int encoderValue;

        public int getEncoderValue() {
            return encoderValue;
        }
    }

    public static final double armBack = 0.06;
    public static final double armFront = 0.87;

    public static final double wristFront = 0;
    public static final double wristBack = 0.66;

    public static final double intakeOpen = 0.3;
    public static final double intakeClosed = 0.49;

    public static final double motorResolution = 134.4; //per revolution
    public static final double colsonCircumference = Math.PI*63.50000006477; //mm

    public static final double moduleMaxAngularVelocity = 1;
    public static final double moduleMaxAngularAcceleration = 0;

    public static final double robotMaxAngularVelocity = 1;
    public static final double robotMaxAngularAcceleration = 0;

    public static final double maxSwerveRPS = (114.0/60.0);
    public static final double maxSwerveVelocity = maxSwerveRPS * colsonCircumference; // (mm/s)
    public static final double maxSwerveAcceleration = 900;// (mm/s/s)

    public static double ticksPerDegreeLeftSwerve = 31921.0/3600.0;
    public static double ticksPerDegreeRightSwerve = 31921.0/3600.0;

    public static double ticksPerMMLeftSwerve = 0.4498;
    public static double ticksPerMMRightSwerve = 0.4498;

    public static final double moduleAngle_kP = 0.1;
    public static final double robotHeading_kP = 0.3;

    public static final double Kv = 0.1;
    public static final double Ka = 0.1;

    /**
     * true value = this/(this+1)
     */
    public static final double minimumHeadingEmphasis = 0.25;
    public static final double minimumHeadingApproachPower = 0.25;
    public static final double minimumModuleAngleApproachPower = 0.01;

    public static final double swerveDistance = 10;

    public static final boolean robotFlipping = false;
}