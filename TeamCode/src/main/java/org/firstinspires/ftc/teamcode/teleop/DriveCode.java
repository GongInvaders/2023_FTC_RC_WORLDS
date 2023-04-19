package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LimitSwitch;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.OpModeWrapper;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;

// brodie mode biggest bird edition
@TeleOp(name = "brodie mode \nbiggest bird edition \nft. CRACKER BARREL \nft. blue lobster \n\n\nBIIIIIIIG", group = "!up to the ftrony!")
public class DriveCode extends OpModeWrapper {
	private MecanumDriveBase mecanum;
	private Arm arm;
	private Wrist wrist;
	private Intake intake;
	private LimitSwitch limitSwitch;
	private Lift lift;
	
	double checkTime;
	boolean throttle;
	
	
	@Override
	public void superInit() {
		mecanum = new MecanumDriveBase(true, MecanumDriveBase.DriveModes.TANK);
		arm = new Arm();
		wrist = new Wrist();
		intake = new Intake(Intake.IntakePos.CLOSED);
		limitSwitch = new LimitSwitch();
		lift = new Lift();
	}

	@Override
	public void registerTriggers() {
		gamepadEX2.rightY.applyDeadZone(0.1);
		
		gamepadEX2.right_trigger
				.thresholdTrigger(0.9)
				.onTrue(() -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
					checkTime = RobotConfig.elapsedTime.time();
				})
				.onFalse(() -> {
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				});
		gamepadEX2.right_trigger
				.thresholdTrigger(0.1)
				.onTrue(() -> {
					r.setPickup(true);
				})
				.onFalse(() -> {
					if (RobotConfig.elapsedTime.time() - checkTime > 0.15) {
						r.setPickup(false);
					}
				});
		gamepadEX2.left_trigger
				.thresholdTrigger(0.2)
				.onTrue(() -> {
					r.setDelivery(false);
					r.setPickup(true);
				});
		
		gamepadEX2.back
				.onPress()
				.onTrue(() -> {
					lift.resetEncoders();
				});
		
		gamepadEX1.back
				.onPress()
				.onTrue(() -> {
					mecanum.flipDriveBase();
				});
		
		gamepadEX1.left_bumper
				.isPressed()
				.onTrue(() -> {
					arm.presetTargetPosition(Arm.ArmPos.BACK_DELIVERY);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
				});

		gamepadEX1.right_bumper
				.isPressed()
				.onTrue(() -> {
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
				});
		
		gamepadEX1.right_stick_button
				.onPress()
				.toggleOnTrue(() -> throttle = true)
				.toggleOnFalse(() -> throttle = false);
	}

	@Override
	public void superInit_Loop() {
	}

	@Override
	public void superStart() {
	
	}

	@Override
	public void superLoop() {
		mecanum.tankDrive(
				gamepadEX1.leftY.getValue(),
				gamepadEX1.rightY.getValue(),
				gamepadEX1.left_trigger.getValue(),
				gamepadEX1.right_trigger.getValue(),
				throttle
		);
		
		lift.liftInputs(
				gamepadEX2.rightY.getValue(),
				limitSwitch.limitSwitchEX.buttonState(),
				gamepadEX2.y.buttonState(),
				gamepadEX2.b.buttonState(),
				gamepadEX2.x.buttonState(),
				gamepadEX2.a.buttonState()
		);
	}

	@Override
	public void superStop() {

	}
}
