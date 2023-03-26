package org.firstinspires.ftc.teamcode.subsystems.webcam;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class Webcam extends Subsystem {
	private final RobotConfig r;
	
	private OpenCvCamera camera;
	private AprilTagDetectionPipeline aprilTagDetectionPipeline;
	
	public int getTagId() {
		if (!(tag == null)){
			return tag.id;
		}
		else {
			return 0;
		}
	}
	
	private AprilTagDetection tag;
	
	public boolean isTagFound() {
		return tagFound;
	}
	
	private boolean tagFound;
	
	private Telemetry.Item dataOutput;
	
	
	public Webcam (RobotConfig r){
		this.r = r;
	}
	
	public Webcam (){
		this(RobotConfig.getInstance());
	}
	
	@Override
	public void init() {
		int cameraMonitorViewId = r.opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", r.opMode.hardwareMap.appContext.getPackageName());
		camera = OpenCvCameraFactory.getInstance().createWebcam(r.opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
		aprilTagDetectionPipeline = new AprilTagDetectionPipeline(0.166, 578.272, 578.272, 402.145, 221.506);
		
		dataOutput = r.opMode.telemetry.addData("", "");
		
		camera.setPipeline(aprilTagDetectionPipeline);
		camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened()
			{
				camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
			}
			
			@Override
			public void onError(int errorCode)
			{
			
			}
		});
		FtcDashboard.getInstance().startCameraStream(camera, 10); //TODO: remove this for the competition
		tag = null;
	}
	
	@Override
	public void read() {
	
	}
	
	@Override
	public void update() {
	
	}
	
	@Override
	public void close() {
	
	}
	
	public void readStream(){
		ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
		
		if(currentDetections.size() != 0) {
			tagFound = false;
			
			for(AprilTagDetection tag : currentDetections)
			{
				if(tag.id == 1 || tag.id == 2 || tag.id == 3) {
					this.tag = tag;
					tagFound = true;
					break;
				}
			}
			
			if(tagFound)
			{
				dataOutput.setCaption("tag found");
				dataOutput.setValue(tag.id);
			}
			else
			{
				if(tag == null) {
					dataOutput.setCaption("No tag found as of yet, other tag sighted");
				}
				else {
					dataOutput.setCaption("tag previously seen, plus other tag sighted");
					dataOutput.setValue(tag.id);
				}
			}
			
		}
		else {
			if(tag == null) {
				dataOutput.setCaption("No tag found as of yet");
			}
			else {
				dataOutput.setCaption("tag previously seen");
				dataOutput.setValue(tag.id);
			}
		}
	}
	
	public void closeStream(){
		camera.stopStreaming();
	}
}
