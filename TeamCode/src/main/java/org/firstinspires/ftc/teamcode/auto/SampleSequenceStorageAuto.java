package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LimitSwitch;
import org.firstinspires.ftc.teamcode.subsystems.Tensioner;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.webcam.Webcam;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.OpModeWrapper;
import org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.drive.SampleMecanumDrive;

@Disabled //remove this to make the code appear on the REV
@Autonomous(name="Sample Sequence Storage Auto", group="")
public class SampleSequenceStorageAuto extends OpModeWrapper {
	private SampleMecanumDrive mecanum;
	private Arm arm;
	private Wrist wrist;
	private Intake intake;
	private LimitSwitch limitSwitch;
	private Lift lift;
	private Webcam webcam;
	private Tensioner tensioner;
	TrajectorySequenceStorage sequence;
	
	@Override
	public void superInit() {
		mecanum = new SampleMecanumDrive(hardwareMap);
		arm = new Arm();
		wrist = new Wrist();
		intake = new Intake(Intake.IntakePos.INIT);
		limitSwitch = new LimitSwitch();
		lift = new Lift();
		webcam = new Webcam();
		tensioner = new Tensioner(Tensioner.RunMode.DEPLOY);
		
		sequence = new TrajectorySequenceStorage()
				.sampleSequenceStorage ( //change to correct sequence storage name.
				r,
				mecanum,
				lift,
				intake,
				arm,
				wrist,
				webcam
		);
	}
	
	@Override
	public void registerTriggers() {
	
	}
	
	@Override
	public void superInit_Loop() {
		webcam.readStream();
	}
	
	@Override
	public void superStart() {
		sequence.addRightParkSequence(webcam.getTagId()); //if on the left side, change this to sequence.addLeftParkSequence(webcam.getTagId());
		webcam.closeStream();
		sequence.startFollowSetSequenceAsync();
	}
	
	@Override
	public void superLoop() {
		sequence.followSetSequenceAsync();
		lift.limitSwitchInput(limitSwitch.limitSwitchEX.buttonState());
	}
	
	@Override
	public void superStop() {
	
	}
}
