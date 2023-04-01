package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.webcam.Webcam;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

public class TrajectorySequenceStorage {
	
	private TrajectorySequence[] trajectorySequences;
	private ArrayList<TrajectorySequence> trajectorySequenceArrayList;
	private int sequenceIndex;
	private RobotConfig r;
	private SampleMecanumDrive drive;
	private Lift lift;
	private Intake intake;
	private Arm arm;
	private Wrist wrist;
	private Webcam webcam;
	
	public static final Pose2d startPoseRight = new Pose2d (34, -65, Math.toRadians(90));

	public static final Pose2d startPoseLeft = new Pose2d (-34, -65, Math.toRadians(90));
	
	private TrajectorySequence rightStartMediumPole(){
		drive.setPoseEstimate(startPoseRight);
		return drive.trajectorySequenceBuilder(startPoseRight)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
					r.setPickup(true);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HOVER);
				})
				.splineTo(new Vector2d(32.5, -17.7),  Math.toRadians(90))//drive forward //was 33.5 for x
				.turn(Math.toRadians(-78.5))//turn //TODO change
				.UNSTABLE_addTemporalMarkerOffset(-1.3, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.BACK_DELIVERY);
				})
				.waitSeconds(0)
				.UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM_DROP);
				})
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.build();
	}
	
	private TrajectorySequence rightMediumPoleToStack(Lift.PoleHeights poleHeight, Vector2d offset){
		return drive.trajectorySequenceBuilder(trajectorySequenceArrayList.get(trajectorySequenceArrayList.size()-1).end())
				.UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
					lift.presetLiftPosition(poleHeight);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					arm.presetTargetPosition(Arm.ArmPos.HALF);
				})
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.lineTo(new Vector2d(55.5, -10.6).plus(offset))
				.waitSeconds(0.0)
				.UNSTABLE_addTemporalMarkerOffset(-0.8, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
				})
				.UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.build();
	}
	
	private TrajectorySequence rightStackToMediumPole(Vector2d offset){
		return drive.trajectorySequenceBuilder(trajectorySequenceArrayList.get(trajectorySequenceArrayList.size()-1).end())
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HOVER);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
				})
				.lineTo(new Vector2d(33.5, -17.0).plus(offset))
				.UNSTABLE_addTemporalMarkerOffset(-1.2, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.BACK_DELIVERY);
				})
				.waitSeconds(0)
				.UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM_DROP);
				})
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.build();
	}

	private TrajectorySequence rightStartMidlineHighPole(){
		drive.setPoseEstimate(startPoseRight);
		return drive.trajectorySequenceBuilder(startPoseRight)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
					r.setPickup(true);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HOVER);
					arm.presetTargetPosition(Arm.ArmPos.HALF);
				})
				.splineTo(new Vector2d(31, -14),  Math.toRadians(90))//drive forward
				.turn(Math.toRadians(72))//turn //TODO change
				.UNSTABLE_addTemporalMarkerOffset(-1.3, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HIGH);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
				})

				.waitSeconds(0)
				.UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HIGH_DROP);
				})
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.build();
	}

	private TrajectorySequence rightMidlineHighPoleToStack(Lift.PoleHeights poleHeight, Vector2d offset){
		return drive.trajectorySequenceBuilder(trajectorySequenceArrayList.get(trajectorySequenceArrayList.size()-1).end())
				.UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
					lift.presetLiftPosition(poleHeight);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.HALF);
				})
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.lineTo(new Vector2d(54.4, -18).plus(offset))
				.waitSeconds(0.0)
				.UNSTABLE_addTemporalMarkerOffset(-0.8, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.BACK_DELIVERY);
				})
				.UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.build();
	}

	private TrajectorySequence rightStackToMidlineHighPole(Vector2d offset){
		return drive.trajectorySequenceBuilder(trajectorySequenceArrayList.get(trajectorySequenceArrayList.size()-1).end())
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HOVER);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
				})
				.lineTo(new Vector2d(33.5, -6.3).plus(offset))
				.UNSTABLE_addTemporalMarkerOffset(-1.2, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HIGH);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
				})
				.waitSeconds(0)
				.UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HIGH_DROP);
				})
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.build();
	}
	
	private TrajectorySequence leftStartMediumPole(){
		drive.setPoseEstimate(startPoseLeft);
		return drive.trajectorySequenceBuilder(startPoseLeft)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
					r.setPickup(true);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HOVER);
				})
				.splineTo(new Vector2d(-33.5, -17.7),  Math.toRadians(90))//drive forward
				.turn(Math.toRadians(-103))//turn //TODO change
				.UNSTABLE_addTemporalMarkerOffset(-1.3, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
				})
				.waitSeconds(0)
				.UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM_DROP);
				})
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.build();
	}
	
	private TrajectorySequence leftMediumPoleToStack(Lift.PoleHeights poleHeight, Vector2d offset){
		return drive.trajectorySequenceBuilder(trajectorySequenceArrayList.get(trajectorySequenceArrayList.size()-1).end())
				.UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
					lift.presetLiftPosition(poleHeight);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.HALF);
				})
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.lineTo(new Vector2d(-54.4, -10.6).plus(offset))
				.waitSeconds(0.0)
				.UNSTABLE_addTemporalMarkerOffset(-0.8, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.BACK_DELIVERY);
				})
				.UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.build();
	}
	
	private TrajectorySequence leftStackToMediumPole(Vector2d offset){
		return drive.trajectorySequenceBuilder(trajectorySequenceArrayList.get(trajectorySequenceArrayList.size()-1).end())
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HOVER);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
				})
				.lineTo(new Vector2d(-33.5, -16.0).plus(offset))
				.UNSTABLE_addTemporalMarkerOffset(-1.2, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM);
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
				})
				.waitSeconds(0)
				.UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM_DROP);
				})
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.build();
	}

	
	public TrajectorySequenceStorage sampleSequenceStorage(
			RobotConfig r,
			SampleMecanumDrive drive,
			Lift lift,
			Intake intake,
			Arm arm,
			Wrist wrist,
			Webcam webcam
		){
		this.r = r;
		this.drive = drive;
		this.lift = lift;
		this.intake = intake;
		this.arm = arm;
		this.wrist = wrist;
		this.webcam = webcam;
		this.sequenceIndex = 0;
		
		trajectorySequenceArrayList = new ArrayList<>();
		
		//use trajectorySequenceArrayList.add(<sequence>); here to sequence them up
		
		trajectorySequenceArrayList.add(parkingPlaceholder());
		
		trajectorySequences = new TrajectorySequence[trajectorySequenceArrayList.size()];
		for (int i = 0; i < trajectorySequenceArrayList.size(); i++) {
			trajectorySequences[i] = trajectorySequenceArrayList.get(i);
		}
		
		return this;
	}
	
	public TrajectorySequenceStorage leftMedium5(
			RobotConfig r,
			SampleMecanumDrive drive,
			Lift lift,
			Intake intake,
			Arm arm,
			Wrist wrist,
			Webcam webcam
	){
		this.r = r;
		this.drive = drive;
		this.lift = lift;
		this.intake = intake;
		this.arm = arm;
		this.wrist = wrist;
		this.webcam = webcam;
		this.sequenceIndex = 0;
		
		trajectorySequenceArrayList = new ArrayList<>();
		trajectorySequenceArrayList.add(leftStartMediumPole());
		
		trajectorySequenceArrayList.add(leftMediumPoleToStack(Lift.PoleHeights.STACK4, new Vector2d(0, 0)));
		trajectorySequenceArrayList.add(leftStackToMediumPole(new Vector2d(-0.7, 0)));
		
		trajectorySequenceArrayList.add(leftMediumPoleToStack(Lift.PoleHeights.STACK3, new Vector2d(0, 0.5)));
		trajectorySequenceArrayList.add(leftStackToMediumPole(new Vector2d(-0.6, 0.5)));
		
		trajectorySequenceArrayList.add(leftMediumPoleToStack(Lift.PoleHeights.STACK2, new Vector2d(0, 0.9)));
		trajectorySequenceArrayList.add(leftStackToMediumPole(new Vector2d(-0.5, 1.0)));
		
		trajectorySequenceArrayList.add(leftMediumPoleToStack(Lift.PoleHeights.STACK1, new Vector2d(0, 0.9)));
		trajectorySequenceArrayList.add(leftStackToMediumPole(new Vector2d(-0.4, 1.5)));
		
		trajectorySequenceArrayList.add(leftMediumPoleToStack(Lift.PoleHeights.STACK0, new Vector2d(0, 1.9)));
		trajectorySequenceArrayList.add(leftStackToMediumPole(new Vector2d(-0.3, 2.0)));
		
		trajectorySequenceArrayList.add(parkingPlaceholder());
		
		trajectorySequences = new TrajectorySequence[trajectorySequenceArrayList.size()];
		for (int i = 0; i < trajectorySequenceArrayList.size(); i++) {
			trajectorySequences[i] = trajectorySequenceArrayList.get(i);
		}
		
		return this;
	}

	public TrajectorySequenceStorage rightMedium5(
			RobotConfig r,
			SampleMecanumDrive drive,
			Lift lift,
			Intake intake,
			Arm arm,
			Wrist wrist,
			Webcam webcam
	){
		this.r = r;
		this.drive = drive;
		this.lift = lift;
		this.intake = intake;
		this.arm = arm;
		this.wrist = wrist;
		this.webcam = webcam;
		this.sequenceIndex = 0;
		
		trajectorySequenceArrayList = new ArrayList<>();
		trajectorySequenceArrayList.add(rightStartMediumPole());
		
		trajectorySequenceArrayList.add(rightMediumPoleToStack(Lift.PoleHeights.STACK4, new Vector2d(0, 0))); //X offset was 0
		trajectorySequenceArrayList.add(rightStackToMediumPole(new Vector2d(0.7, 0)));
		
		trajectorySequenceArrayList.add(rightMediumPoleToStack(Lift.PoleHeights.STACK3, new Vector2d(0, 0.6))); //X offset was 0
		trajectorySequenceArrayList.add(rightStackToMediumPole(new Vector2d(0.6, 0.5)));
		
		trajectorySequenceArrayList.add(rightMediumPoleToStack(Lift.PoleHeights.STACK2, new Vector2d(0, 1.1)));
		trajectorySequenceArrayList.add(rightStackToMediumPole(new Vector2d(0.5, 1.0)));
		
		trajectorySequenceArrayList.add(rightMediumPoleToStack(Lift.PoleHeights.STACK1, new Vector2d(0, 1.2)));
		trajectorySequenceArrayList.add(rightStackToMediumPole(new Vector2d(0.4, 1.4)));
		
		trajectorySequenceArrayList.add(rightMediumPoleToStack(Lift.PoleHeights.STACK0, new Vector2d(0, 2.3)));
		trajectorySequenceArrayList.add(rightStackToMediumPole(new Vector2d(0.3, 1.8)));
		
		trajectorySequenceArrayList.add(parkingPlaceholder());
		
		trajectorySequences = new TrajectorySequence[trajectorySequenceArrayList.size()];
		for (int i = 0; i < trajectorySequenceArrayList.size(); i++) {
			trajectorySequences[i] = trajectorySequenceArrayList.get(i);
		}

		return this;
	}

	public TrajectorySequenceStorage rightMidlineHigh5(
			RobotConfig r,
			SampleMecanumDrive drive,
			Lift lift,
			Intake intake,
			Arm arm,
			Wrist wrist,
			Webcam webcam
	){
		this.r = r;
		this.drive = drive;
		this.lift = lift;
		this.intake = intake;
		this.arm = arm;
		this.wrist = wrist;
		this.webcam = webcam;
		this.sequenceIndex = 0;

		trajectorySequenceArrayList = new ArrayList<>();
		trajectorySequenceArrayList.add(rightStartMidlineHighPole());

		trajectorySequenceArrayList.add(rightMidlineHighPoleToStack(Lift.PoleHeights.STACK4, new Vector2d(0, 1.0)));
		trajectorySequenceArrayList.add(rightStackToMidlineHighPole(new Vector2d(-0.2, -0.3)));

		trajectorySequenceArrayList.add(rightMidlineHighPoleToStack(Lift.PoleHeights.STACK3, new Vector2d(0, 1.0)));
		trajectorySequenceArrayList.add(rightStackToMidlineHighPole(new Vector2d(-0.4, -0.6)));

		trajectorySequenceArrayList.add(rightMidlineHighPoleToStack(Lift.PoleHeights.STACK2, new Vector2d(0, 1.0)));
		trajectorySequenceArrayList.add(rightStackToMidlineHighPole(new Vector2d(-0.6, -0.9)));

		trajectorySequenceArrayList.add(rightMidlineHighPoleToStack(Lift.PoleHeights.STACK1, new Vector2d(0, 0)));
		trajectorySequenceArrayList.add(rightStackToMidlineHighPole(new Vector2d(-0.8, -1.2)));

		trajectorySequenceArrayList.add(rightMidlineHighPoleToStack(Lift.PoleHeights.STACK0, new Vector2d(0, 0)));
		trajectorySequenceArrayList.add(rightStackToMidlineHighPole(new Vector2d(-1, -1.5)));

		trajectorySequenceArrayList.add(parkingPlaceholder());

		trajectorySequences = new TrajectorySequence[trajectorySequenceArrayList.size()];
		for (int i = 0; i < trajectorySequenceArrayList.size(); i++) {
			trajectorySequences[i] = trajectorySequenceArrayList.get(i);
		}

		return this;
	}


	public void followSetSequenceAsync(){
		if(!drive.isBusy()){
			if(sequenceIndex >= trajectorySequences.length - 1) {
				return;
			}
			drive.followTrajectorySequenceAsync(trajectorySequences[++sequenceIndex]);
			
		}
		drive.update();
	}
	
	public void startFollowSetSequenceAsync(){
		if(sequenceIndex >= trajectorySequences.length){
			return;
		}
		drive.followTrajectorySequenceAsync(trajectorySequences[sequenceIndex]);
	}
	
	public void addRightParkSequence(int tagid){
		switch (tagid){
			case 1:
				trajectorySequences[trajectorySequences.length-1] = rightPark1(trajectorySequences[trajectorySequences.length-2].end());
				break;
			case 3:
				trajectorySequences[trajectorySequences.length-1] = rightPark3(trajectorySequences[trajectorySequences.length-2].end());
				break;
			default:
				trajectorySequences[trajectorySequences.length-1] = rightPark2(trajectorySequences[trajectorySequences.length-2].end());
				break;
		}
	}
	
	public void addLeftParkSequence(int tagid){
		switch (tagid){
			case 1:
				trajectorySequences[trajectorySequences.length-1] = leftPark1(trajectorySequences[trajectorySequences.length-2].end());
				break;
			case 3:
				trajectorySequences[trajectorySequences.length-1] = leftPark3(trajectorySequences[trajectorySequences.length-2].end());
				break;
			default:
				trajectorySequences[trajectorySequences.length-1] = leftPark2(trajectorySequences[trajectorySequences.length-2].end());
				break;
		}
	}
	
	private TrajectorySequence parkingPlaceholder(){
		return drive.trajectorySequenceBuilder(trajectorySequenceArrayList.get(trajectorySequenceArrayList.size()-1).end())
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.back(1)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					arm.presetTargetPosition(Arm.ArmPos.HALF);
				})
				.build();
	}
	private TrajectorySequence rightPark1(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.lineToLinearHeading(new Pose2d(7, -12, Math.toRadians(90)))
				.build();
	}
	
	private TrajectorySequence rightPark2(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.lineToLinearHeading(new Pose2d(36, -16, Math.toRadians(90)))
				.build();
	}
	
	private TrajectorySequence rightPark3(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(90)))
				.build();
	}
	
	private TrajectorySequence leftPark1(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.lineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(90)))
				.build();
	}
	
	private TrajectorySequence leftPark2(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.lineToLinearHeading(new Pose2d(-30, -12, Math.toRadians(90)))
				.build();
	}
	
	private TrajectorySequence leftPark3(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.lineToLinearHeading(new Pose2d(-16, -12, Math.toRadians(90)))
				.build();
	}
}
