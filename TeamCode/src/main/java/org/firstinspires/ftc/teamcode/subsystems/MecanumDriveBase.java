package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teamUtil.Angle;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.ConfigNames;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;

public class MecanumDriveBase extends Subsystem {
    RobotConfig r;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;
    private DcMotorEx frontLeft;

    private boolean fieldCentric;
    
    private double powerX;
    private double powerY;
    private double headingX;
    private double headingY;
    private double headingDelta;
    private double robotHeadingRadians;
    private Angle targetHeading;
    private Angle robotHeading;
    
    private boolean flipped;
    
    public enum DriveModes{
        POV,
        CENTRIC,
        TANK
    }
    
    private DriveModes driveMode;
    
    public MecanumDriveBase(RobotConfig r, boolean fieldCentric, DriveModes driveMode){
        this.r = r;
        this.fieldCentric = fieldCentric;
        frontRight = r.opMode.hardwareMap.get(DcMotorEx.class, ConfigNames.frontRight);
        backRight = r.opMode.hardwareMap.get(DcMotorEx.class, ConfigNames.backRight);
        backLeft = r.opMode.hardwareMap.get(DcMotorEx.class, ConfigNames.backLeft);
        frontLeft = r.opMode.hardwareMap.get(DcMotorEx.class, ConfigNames.frontLeft);
    
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.driveMode = driveMode;
    }
    public MecanumDriveBase(boolean fieldCentric, DriveModes driveMode){
        this(RobotConfig.getInstance(), fieldCentric, driveMode);
    }

    public void trueHolonomicDrive(double planarX, double planarY, double headingX, double headingY, double throttle){
        
        if(fieldCentric){
            this.powerX = planarY * Math.sin(robotHeadingRadians) + planarX * Math.cos(robotHeadingRadians);
            this.powerY = planarY * Math.cos(robotHeadingRadians) - planarX * Math.sin(robotHeadingRadians);
        }
        else{
            this.powerX = planarX;
            this.powerY = planarY;
        }
        this.headingX = headingX;
        this.headingY = headingY;
        
        this.frontRightPower = (powerX + powerY + headingDelta) * throttle * 0.6;
        this.backRightPower = (powerX - powerY + headingDelta) * throttle * 0.6;
        this.backLeftPower = (powerX + powerY - headingDelta) * throttle * 0.6;
        this.frontLeftPower = (powerX - powerY - headingDelta) * throttle * 0.6;
        
    }
    
    public void tankDrive(double leftTank, double rightTank, double leftStrafe, double rightStrafe, boolean throttleInput) {
        double throttle;
        if(throttleInput) {
            throttle = 0.4;
        }
        else {
            throttle = 0.7;
        }
        
        if(flipped) {
            this.frontRightPower = -(leftTank + leftStrafe - rightStrafe) * throttle;
            this.backRightPower = -(leftTank - leftStrafe + rightStrafe) * throttle;
            this.backLeftPower = -(rightTank - rightStrafe + leftStrafe) * throttle;
            this.frontLeftPower = -(rightTank + rightStrafe - leftStrafe) * throttle;
        }
        else {
            this.frontRightPower = (rightTank - rightStrafe + leftStrafe) * throttle;
            this.backRightPower = (rightTank + rightStrafe - leftStrafe) * throttle;
            this.backLeftPower = (leftTank + leftStrafe - rightStrafe) * throttle;
            this.frontLeftPower = (leftTank - leftStrafe + rightStrafe) * throttle;
        }
    }

    public void setFieldCentric(boolean fieldCentric){
        this.fieldCentric = fieldCentric;
    }

    @Override
    public void init() {
        setFieldCentric(fieldCentric);
        r.resetIMU();
    }

    @Override
    public void read() {
        if(fieldCentric){
            robotHeading = new Angle(r.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)-r.getImuOffset()+90);
            robotHeadingRadians = Math.toRadians(robotHeading.getValue());
            switch(driveMode) {
    
                case POV:
                    headingDelta = -headingX;
                    break;
                case CENTRIC:
                    if (flipped){
                        targetHeading = new Angle (Angle.atanHandler(headingX, headingY).getValue()+180);
                    }
                    else{
                        targetHeading = new Angle (Angle.atanHandler(headingX, headingY).getValue());
                    }
                    headingDelta = -(robotHeading.angleShortDifference(targetHeading)/45.0)/Math.max(1, Math.abs(headingDelta));
                    if (Double.isNaN(headingDelta)){
                        headingDelta = 0;
                    }
                    break;
                case TANK:
                    
                    break;
            }
        }
        else{
            headingDelta = -headingX;
        }
    }
    
    private double frontRightPower;
    private double backRightPower;
    private double backLeftPower;
    private double frontLeftPower;
    private double cachedFrontRightPower;
    private double cachedBackRightPower;
    private double cachedBackLeftPower;
    private double cachedFrontLeftPower;

    @Override
    public void update() {
        if(Math.abs(Math.abs(cachedFrontRightPower) - Math.abs(frontRightPower)) >= 0.01) {
            frontRight.setPower(frontRightPower);
            cachedFrontRightPower = frontRightPower;
        }
        if(Math.abs(Math.abs(cachedBackRightPower) - Math.abs(backRightPower)) >= 0.01) {
            backRight.setPower(backRightPower);
            cachedBackRightPower = backRightPower;
        }
        if(Math.abs(Math.abs(cachedBackLeftPower) - Math.abs(backLeftPower)) >= 0.01) {
            backLeft.setPower(backLeftPower);
            cachedBackLeftPower = backLeftPower;
        }
        if(Math.abs(Math.abs(cachedFrontLeftPower) - Math.abs(frontLeftPower)) >= 0.01) {
            frontLeft.setPower(frontLeftPower);
            cachedFrontLeftPower = frontLeftPower;
        }
    }

    @Override
    public void close() {
    
    }
    
    public void flipDriveBase(){
        flipped = !flipped;
    }
    
    public void setDriveBaseFlip(boolean flipped){
        this.flipped = flipped;
    }
    
    public void setDriveMode(DriveModes driveMode){
        this.driveMode = driveMode;
    }
}