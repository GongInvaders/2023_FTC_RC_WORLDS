package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.ConfigNames;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;

public class Intake extends Subsystem {
    RobotConfig r;
    private IntakePos initPos;

    public Intake(RobotConfig r, IntakePos initPos) {
        this.initPos = initPos;
        this.r = r;
    }
    public Intake(IntakePos initPos){
        this (RobotConfig.getInstance(), initPos);
    }

    public enum IntakePos {
        OPEN(RobotConstants.intakeOpen),
        CLOSED(RobotConstants.intakeClosed),
        INIT(0);

        IntakePos(double position){
            this.position = position;
        }

        private final double position;

        public double getPosition() {
            return position;
        }
    }

    private Servo intake;
    private double intakePosition;

    public void freeTargetPosition(double targetPos){
        this.intakePosition = targetPos;
    }

    public void presetTargetPosition(IntakePos intakePos){
        this.intakePosition = intakePos.getPosition();
    }

    @Override
    public void init() {
        intake = r.opMode.hardwareMap.get(Servo.class, ConfigNames.intake);
        presetTargetPosition(initPos);
        update();
    }

    @Override
    public void read() {

    }

    public void update(){
        intake.setPosition(intakePosition);
    }

    @Override
    public void close() {}
}
