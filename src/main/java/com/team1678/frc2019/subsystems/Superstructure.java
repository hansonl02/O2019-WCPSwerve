package com.team1678.frc2019.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import com.team1678.frc2019.Constants;
import com.team1678.frc2019.RobotState;
import com.team1678.frc2019.loops.ILooper;
import com.team1678.frc2019.loops.LimelightProcessor;
import com.team1678.frc2019.loops.Loop;
import com.team1678.frc2019.subsystems.Swerve.VisionState;
import com.team1678.frc2019.subsystems.requests.Request;
import com.team1678.frc2019.subsystems.requests.RequestList;
import com.team1323.lib.util.InterpolatingDouble;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;

public class Superstructure extends Subsystem {

	public Wrist wrist;
	public BallIntake ballIntake;
	public BallCarriage ballCarriage;
	public DiskIntake diskIntake;

	private Compressor compressor;
	
	private Swerve swerve;

	private LimelightProcessor limelight;

	private RobotState robotState;

	private boolean isClimbing = false;
	public boolean isClimbing(){ return isClimbing; }
	public void enableInterpolator(boolean enable){ 
		isClimbing = enable;
		swerve.setLowPowerScalar(0.25);
	}
	public void stopClimbing(){ 
		isClimbing = false; 
		swerve.setLowPowerScalar(0.6);
	}
	
	public Superstructure(){
		ballIntake = BallIntake.getInstance();
		ballCarriage = BallCarriage.getInstance();
		diskIntake = DiskIntake.getInstance();
		
		compressor = new Compressor(20);
		
		swerve = Swerve.getInstance();

		limelight = LimelightProcessor.getInstance();

		robotState = RobotState.getInstance();
		
		queuedRequests = new ArrayList<>(0);
	}
	private static Superstructure instance = null;
	public static Superstructure getInstance(){
		if(instance == null)
			instance = new Superstructure();
		return instance;
	}
	
	private RequestList activeRequests;
	private ArrayList<RequestList> queuedRequests;
	private Request currentRequest;
	
	private boolean newRequests = false;
	private boolean activeRequestsCompleted = false;
	private boolean allRequestsCompleted = false;
	public boolean requestsCompleted(){ return allRequestsCompleted; }
	
	private void setActiveRequests(RequestList requests){
		activeRequests = requests;
		newRequests = true;
		activeRequestsCompleted = false;
		allRequestsCompleted = false;
	}
	
	private void setQueuedRequests(RequestList requests){
		queuedRequests.clear();
		queuedRequests.add(requests);
	}
	
	private void setQueuedRequests(List<RequestList> requests){
		queuedRequests.clear();
		queuedRequests = new ArrayList<>(requests.size());
		for(RequestList list : requests){
			queuedRequests.add(list);
		}
	}
	
	public void request(Request r){
		setActiveRequests(new RequestList(Arrays.asList(r), false));
		setQueuedRequests(new RequestList());
	}
	
	public void request(Request active, Request queue){
		setActiveRequests(new RequestList(Arrays.asList(active), false));
		setQueuedRequests(new RequestList(Arrays.asList(queue), false));
	}
	
	public void request(RequestList list){
		setActiveRequests(list);
		setQueuedRequests(new RequestList());
	}
	
	public void request(RequestList activeList, RequestList queuedList){
		setActiveRequests(activeList);
		setQueuedRequests(queuedList);
	}
	
	public void addActiveRequest(Request request){
		activeRequests.add(request);
		newRequests = true;
		activeRequestsCompleted = false;
		allRequestsCompleted = false;
	}
	
	/** Ill-advised */
	public void addForemostActiveRequest(Request request){
		activeRequests.addToForefront(request);
		newRequests = true;
		activeRequestsCompleted = false;
		allRequestsCompleted = false;
	}
	
	public void queue(Request request){
		queuedRequests.add(new RequestList(Arrays.asList(request), false));
	}
	
	public void queue(RequestList list){
		queuedRequests.add(list);
	}
	
	public void replaceQueue(Request request){
		setQueuedRequests(new RequestList(Arrays.asList(request), false));
	}
	
	public void replaceQueue(RequestList list){
		setQueuedRequests(list);
	}
	
	public void replaceQueue(List<RequestList> lists){
		setQueuedRequests(lists);
	}
	
	private final Loop loop = new Loop(){

		@Override
		public void onStart(double timestamp) {
			stop();
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized(Superstructure.this){
				final boolean inRange = diskIntake.getState() == DiskIntake.State.DEPLOYED;
				limelight.enableUpdates(inRange);
				if(!inRange)
					robotState.clearVisionTargets();
			
				swerve.setMaxSpeed(1.0);

				if(!activeRequestsCompleted){
					if(newRequests){
						if(activeRequests.isParallel()){
							boolean allActivated = true;
							for(Iterator<Request> iterator = activeRequests.getRequests().iterator(); iterator.hasNext();){
								Request request = iterator.next();
								boolean allowed = request.allowed();
								allActivated &= allowed;
								if(allowed) request.act();
							}
							newRequests = !allActivated;
						}else{
							if(activeRequests.isEmpty()){
								activeRequestsCompleted = true;
								return;
							}
							currentRequest = activeRequests.remove();
							currentRequest.act();
							newRequests = false;
						}
					}
					if(activeRequests.isParallel()){
						boolean done = true;
						for(Request request : activeRequests.getRequests()){
							done &= request.isFinished();
						}
						activeRequestsCompleted = done;
					}else if(currentRequest.isFinished()){
							if(activeRequests.isEmpty()){
								activeRequestsCompleted = true;
							}else if(activeRequests.getRequests().get(0).allowed()){
								newRequests = true;
								activeRequestsCompleted = false;
							}
					}
				}else{
					if(!queuedRequests.isEmpty()){
						setActiveRequests(queuedRequests.remove(0));
					}else{
						allRequestsCompleted = true;
					}
				}
			
			}
		}

		@Override
		public void onStop(double timestamp) {
			disabledState();
		}
		
	};
	
	public synchronized void sendManualInput(double wristOutput, double elevatorOutput, double jackOutput){
		RequestList list = RequestList.emptyList();
		if(wristOutput != 0){
			list.add(wrist.openLoopRequest(wristOutput));
		}else if(wrist.isOpenLoop()){
			list.add(wrist.lockAngleRequest());
		}
		if(elevatorOutput != 0){
			list.add(elevator.openLoopRequest(elevatorOutput));
		}else if(elevator.isOpenLoop()){
			list.add(elevator.lockHeightRequest());
		}
		/*if(jackOutput != 0){
			list.add(jacks.openLoopRequest(jackOutput));
		}else if(jacks.isOpenLoop()){
			list.add(jacks.lockHeightRequest());
		}*/
		
		if(!list.isEmpty()){
			request(list);
		}
	}

	public void sendJackInput(double input){
		RequestList list = RequestList.emptyList();
		/*if(input != 0){
			list.add(jacks.openLoopRequest(input));
		}else if(jacks.isOpenLoop()){
			list.add(jacks.lockHeightRequest());
		}*/
		//list.add(jacks.openLoopRequest(input));

		if(!list.isEmpty()){
			request(list);
		}
	}
	
	public void enableCompressor(boolean enable){
		compressor.setClosedLoopControl(enable);
	}
	
	public RequestList elevatorWristConfig(double elevatorHeight, double wristAngle){
		return new RequestList(Arrays.asList(elevator.heightRequest(elevatorHeight),
				wrist.angleRequest(wristAngle)), true);
	}
	
	public RequestList idleRequest(){
		return new RequestList(Arrays.asList(wrist.openLoopRequest(0.0),
				elevator.openLoopRequest(0.0)), true);
	}

	@Override
	public void stop() {
		setActiveRequests(idleRequest());
	}

	@Override
	public void zeroSensors() {
		
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(loop);
	}

	@Override
	public void outputTelemetry() {
	}

	public Request waitRequest(double seconds){
		return new Request(){
			double startTime = 0.0;
			double waitTime = 1.0;
		
			@Override
			public void act() {
				startTime = Timer.getFPGATimestamp();
				waitTime = seconds;
			}

			@Override
			public boolean isFinished(){
				return (Timer.getFPGATimestamp() - startTime) > waitTime;
			}
		};
	}

	public Request waitForVisionRequest(){
		return new Request(){

			@Override
			public void act() {

			}

			@Override
			public boolean isFinished(){
				return robotState.seesTarget();
			}

		};
	}

	public Request interpolatorRequest(boolean on){
		return new Request(){
		
			@Override
			public void act() {
				enableInterpolator(on);
			}
		};
	}

	/////States/////

	public void disabledState(){
		RequestList state = new RequestList(Arrays.asList(
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.OFF),
			wrist.angleRequest(Constants.kWristPrimaryStowAngle),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.STOWED)), true);
		request(state);
	}

	public void neutralState(){
		RequestList state = new RequestList(Arrays.asList(
			//ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.OFF),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			diskScorer.stateRequest(diskScorer.isExtended() ? DiskScorer.State.NEUTRAL_EXTENDED : DiskScorer.State.STOWED)), true);
		request(state);
	}

	public void ballIntakingState(){
		RequestList state = new RequestList(Arrays.asList(
			elevator.heightRequest(Constants.kElevatorBallIntakeHeight), 
			wrist.angleRequest(Constants.kWristIntakingAngle),
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.INTAKING),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.STOWED),
			ballIntake.waitForBallRequest()), true);
		/*RequestList queue = new RequestList(Arrays.asList(
			elevator.heightRequest(Constants.kElevatorBallIntakeHeight), 
			wrist.angleRequest(Constants.kWristBallHoldingAngle),
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.HOLDING)), true);*/
		request(state);
	}

	public void ballHoldingState(){
		RequestList state = new RequestList(Arrays.asList(
			elevator.heightRequest(Constants.kElevatorBallIntakeHeight), 
			wrist.angleRequest(Constants.kWristBallHoldingAngle),
			ballCarriage.stateRequest(BallCarriage.State.RECEIVING), 
			ballIntake.stateRequest(BallIntake.State.HOLDING),
			diskScorer.stateRequest(DiskScorer.State.STOWED),
			diskIntake.stateRequest(DiskIntake.State.OFF)), true);
		request(state); 
	}

	public void ballFeedingState(){
		RequestList state = new RequestList(Arrays.asList(
			elevator.heightRequest(Constants.kElevatorBallIntakeHeight), 
			wrist.angleRequest(Constants.kWristBallFeedingAngle, 0.1),
			ballCarriage.stateRequest(BallCarriage.State.RECEIVING), 
			ballIntake.stateRequest(BallIntake.State.FEEDING),
			diskScorer.stateRequest(DiskScorer.State.STOWED),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			ballCarriage.waitForBallRequest()), true);
		RequestList queue = new RequestList(Arrays.asList(
			ballIntake.stateRequest(BallIntake.State.POST_FEEDING),
			ballCarriage.stateRequest(BallCarriage.State.OFF)), true);
		request(state, queue);
	}

	public void fullBallCycleState(){
		RequestList state = new RequestList(Arrays.asList(
			elevator.heightRequest(Constants.kElevatorBallIntakeHeight), 
			wrist.angleRequest(Constants.kWristBallHoldingAngle),
			ballCarriage.stateRequest(BallCarriage.State.RECEIVING),
			ballIntake.stateRequest(BallIntake.State.HOLDING),
			diskScorer.stateRequest(DiskScorer.State.STOWED),
			diskIntake.stateRequest(DiskIntake.State.OFF)), true);
		List<RequestList> queue = Arrays.asList(
			new RequestList(Arrays.asList(
				elevator.heightRequest(Constants.kElevatorBallIntakeHeight), 
				wrist.angleRequest(Constants.kWristBallFeedingAngle, 0.5, false), 
				ballIntake.stateRequest(BallIntake.State.FEEDING),
				diskIntake.stateRequest(DiskIntake.State.OFF),
				ballCarriage.waitForBallRequest()), true),
			new RequestList(Arrays.asList(
				ballIntake.stateRequest(BallIntake.State.POST_FEEDING),
				elevator.heightRequest(Constants.kElevatorLowBallHeight)), true)
		);
		request(state);
		replaceQueue(queue);
	}

	public void ballScoringState(double elevatorHeight){
		RequestList state = new RequestList(Arrays.asList(
			elevator.heightRequest(elevatorHeight), 
			wrist.angleRequest(Constants.kWristBallFeedingAngle),
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.STOWED),
			diskIntake.stateRequest(DiskIntake.State.OFF)), true);
		request(state); 
	}

	public void diskScoringState(double elevatorHeight, boolean resuck){
		RequestList state = new RequestList(Arrays.asList(
			elevator.heightRequest(elevatorHeight),
			wrist.angleRequest(Constants.kWristPrimaryStowAngle),
			diskScorer.stateRequest(resuck ? DiskScorer.State.DETECTED : DiskScorer.State.HOLDING),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			ballIntake.stateRequest(BallIntake.State.OFF),
			ballCarriage.stateRequest(BallCarriage.State.OFF)), true);
		request(state); 
	}

	/** Used for driver tracking */
	public void diskTrackingState(){
		RequestList state = new RequestList(Arrays.asList(
			waitForVisionRequest(),
			swerve.startTrackRequest(Constants.kDiskTargetHeight, new Translation2d(-1.0, 0.0), true, VisionState.LINEAR),
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.OFF),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			swerve.waitForTrackRequest()), false);
		RequestList queue = new RequestList(Arrays.asList(
			swerve.strictWaitForTrackRequest()), false);
		request(state, queue); 
	}

	public void humanLoaderTrackingState(){
		RequestList state = new RequestList(Arrays.asList(
			diskIntake.stateRequest(DiskIntake.State.OFF),
			ballIntake.stateRequest(BallIntake.State.OFF),
			ballCarriage.stateRequest(BallCarriage.State.OFF),
			waitForVisionRequest(),
			swerve.trackRequest(Constants.kDiskTargetHeight, new Translation2d(/*7.0*/4.0, 0.0), false, Rotation2d.fromDegrees(180.0), 48.0, 54.0)), false);

		List<RequestList> queue = Arrays.asList(
			new RequestList(Arrays.asList(
				diskIntake.stateRequest(DiskIntake.State.INTAKING),
				diskIntake.waitForDiskRequest()), true),
			new RequestList(Arrays.asList(
				//swerve.waitForTrackRequest(),
				diskIntake.stateRequest(DiskIntake.State.HOLDING)), false)
		);
		request(state); 
		replaceQueue(queue);
	}
}
