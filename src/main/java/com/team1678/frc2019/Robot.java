/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1678.frc2019;

import java.util.Arrays;

import com.team1678.frc2019.loops.LimelightProcessor;
import com.team1678.frc2019.loops.LimelightProcessor.Pipeline;
import com.team1678.frc2019.loops.Looper;
import com.team1678.frc2019.loops.QuinticPathTransmitter;
import com.team1678.frc2019.loops.RobotStateEstimator;
import com.team1678.frc2019.subsystems.BallIntake;
import com.team1678.frc2019.subsystems.DiskIntake;
import com.team1678.frc2019.subsystems.SubsystemManager;
import com.team1678.frc2019.subsystems.Superstructure;
import com.team1678.frc2019.subsystems.Swerve;
import com.team1323.io.SwitchController;
import com.team1323.io.Xbox;
import com.team1323.lib.util.CrashTracker;
import com.team1323.lib.util.InterpolatingDouble;
import com.team1323.lib.util.Logger;
import com.team1323.lib.util.Util;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	private Swerve swerve;
	private BallIntake ballIntake;
	private DiskIntake diskIntake;
	private Superstructure s;
	private SubsystemManager subsystems;

	private LimelightProcessor limelight;

	private TrajectoryGenerator generator = TrajectoryGenerator.getInstance();

	private Looper enabledLooper = new Looper();
	private Looper disabledLooper = new Looper();

	private RobotState robotState = RobotState.getInstance();

	private DriverStation ds = DriverStation.getInstance();

	private Xbox driver;
	private Xbox operator;
	private SwitchController switchController;
	private final boolean useSwitchController = false;
	private final boolean oneControllerMode = false;
	private boolean flickRotation = false;
	private boolean robotCentric = false;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		s = Superstructure.getInstance();
		swerve = Swerve.getInstance();
		ballIntake = BallIntake.getInstance();
		diskIntake = DiskIntake.getInstance();
		subsystems = new SubsystemManager(
				Arrays.asList(swerve, ballIntake, diskIntake, diskIntake, s));

		limelight = LimelightProcessor.getInstance();
		
		if (useSwitchController) {
			switchController = new SwitchController(2);
		}
		driver = new Xbox(0);
		operator = new Xbox(1);
		driver.setDeadband(0.0);
		operator.setDeadband(0.4);
		operator.rightBumper.setLongPressDuration(1.0);

		Logger.clearLog();

		enabledLooper.register(RobotStateEstimator.getInstance());
		enabledLooper.register(QuinticPathTransmitter.getInstance());
		enabledLooper.register(LimelightProcessor.getInstance());
		disabledLooper.register(RobotStateEstimator.getInstance());
		disabledLooper.register(QuinticPathTransmitter.getInstance());
		disabledLooper.register(LimelightProcessor.getInstance());
		subsystems.registerEnabledLoops(enabledLooper);
		subsystems.registerDisabledLoops(disabledLooper);

		swerve.zeroSensors();
		// swerve.zeroSensors(new Pose2d());

		robotState.feignVisionTargets();
		swerve.startTracking(Constants.kDiskTargetHeight, new Translation2d(-6.0, 0.0), true, new Rotation2d());
		swerve.stop();

		generator.generateTrajectories();
	}

	public void allPeriodic() {
		subsystems.outputToSmartDashboard();
		robotState.outputToSmartDashboard();
		enabledLooper.outputToSmartDashboard();
		//Pigeon.getInstance().outputToSmartDashboard();
		SmartDashboard.putBoolean("Enabled", ds.isEnabled());
		SmartDashboard.putNumber("Match time", ds.getMatchTime());
	}

	public void autoConfig() {
		swerve.zeroSensors();
		swerve.setNominalDriveOutput(0.0);
		swerve.requireModuleConfiguration();
		//swerve.set10VoltRotationMode(true);

		s.enableCompressor(false);
	}

	public void teleopConfig() {
		s.enableCompressor(true);
		swerve.setNominalDriveOutput(0.0);
		swerve.set10VoltRotationMode(false);

		if(diskIntake.hasDisk())
			diskIntake.conformToState(DiskIntake.State.HOLDING);
	}

	@Override
	public void autonomousInit() {
		try {
			disabledLooper.stop();
			enabledLooper.start();
			teleopConfig();
			SmartDashboard.putBoolean("Auto", false);
			robotState.enableXTarget(false);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		try {
			driver.update();
			operator.update();

			if (useSwitchController) {
				switchController.update();
			}

			if (oneControllerMode)
				oneControllerMode();
			else
				twoControllerMode();

			allPeriodic();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopInit() {
		try {
			disabledLooper.stop();
			enabledLooper.start();
			teleopConfig();
			SmartDashboard.putBoolean("Auto", false);
			robotState.enableXTarget(false);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		try {
			driver.update();
			operator.update();

			if (useSwitchController) {
				switchController.update();
			}

			if (oneControllerMode)
				oneControllerMode();
			else
				twoControllerMode();

			allPeriodic();
			
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledInit() {
		try {
			enabledLooper.stop();
			subsystems.stop();
			disabledLooper.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledPeriodic() {
		try {
			allPeriodic();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {

	}

	@Override
	public void testPeriodic() {
		allPeriodic();
	}

	private void twoControllerMode() {
		if (operator.backButton.isBeingPressed()) {
			s.neutralState();
		}

		if (operator.leftTrigger.isBeingPressed()) {
			ballIntake.setState(BallIntake.WantedAction.INTAKE);
		}

		if (operator.rightTrigger.isBeingPressed()) {
			ballIntake.setState(BallIntake.WantedAction.OUTTAKE);
		}

		if (operator.leftBumper.isBeingPressed()) {
			diskIntake.stateRequest(DiskIntake.State.INTAKING);
		}

		if (operator.rightBumper.isBeingPressed()) {
			diskIntake.stateRequest(DiskIntake.State.SCORING);
		}


		double swerveYInput = driver.getX(Hand.kLeft);
		double swerveXInput = -driver.getY(Hand.kLeft);
		double swerveRotationInput = (flickRotation ? 0.0 : driver.getX(Hand.kRight));

		if (useSwitchController) {
			swerveYInput = switchController.getX(Hand.kLeft);
			swerveXInput = -switchController.getY(Hand.kLeft);
			swerveRotationInput = (flickRotation ? 0.0 : switchController.getX(Hand.kRight));

			if (switchController.plusButton.wasPressed()) {

			} else if (switchController.minusButton.wasPressed()) {
				swerve.temporarilyDisableHeadingController();
				swerve.zeroSensors(Constants.kRobotLeftStartingPose);
				swerve.resetAveragedDirection();
			}

			if (switchController.xButton.isBeingPressed())
				swerve.rotate(0);
			else if (switchController.aButton.isBeingPressed())
				swerve.rotate(90);
			else if (switchController.bButton.isBeingPressed())
				swerve.rotate(180);
			else if (switchController.yButton.isBeingPressed())
				swerve.rotate(270);
		}

		swerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, robotCentric, driver.leftTrigger.isBeingPressed());

		if (driver.rightCenterClick.shortReleased()) {
			/*if (flickRotation) {
				driver.rumble(3, 1);
			} else {
				driver.rumble(1, 1);
			}
			flickRotation = !flickRotation;*/
		}

		if (flickRotation) {
			swerve.updateControllerDirection(new Translation2d(-driver.getY(Hand.kRight), driver.getX(Hand.kRight)));
			if (!Util.epsilonEquals(
					Util.placeInAppropriate0To360Scope(swerve.getTargetHeading(),
							swerve.averagedDirection.getDegrees()),
					swerve.getTargetHeading(), swerve.rotationDivision / 2.0)) {
				swerve.rotate(swerve.averagedDirection.getDegrees());
			}
		}
		
		if (driver.bButton.isBeingPressed())
			swerve.rotate(90);
		else if (driver.aButton.isBeingPressed())
			swerve.rotate(180);
		else if (driver.xButton.isBeingPressed())
			swerve.rotate(270);
		else if (driver.leftBumper.shortReleased())
			swerve.rotate(-24);
		else if(driver.leftBumper.longPressed())
			swerve.rotate(-151.0);
		else if (driver.rightBumper.shortReleased())
			swerve.rotate(24);
		else if(driver.rightBumper.longPressed())
			swerve.rotate(151.0);
		else if(driver.POV0.isBeingPressed())
			swerve.rotate(0.0);

		if (driver.backButton.shortReleased() || driver.backButton.longPressed()) {
			swerve.temporarilyDisableHeadingController();
			swerve.zeroSensors(Constants.kRobotLeftStartingPose);
			swerve.resetAveragedDirection();
		} else if (driver.startButton.shortReleased()) {
			if(!swerve.isTracking()){
				//diskIntake.loseDisk();
				limelight.setPipeline(Pipeline.CLOSEST);
				s.humanLoaderTrackingState();
			}
		} /*else if (driver.leftBumper.isBeingPressed()) {
			swerve.setVelocity(new Rotation2d(), 24.0);
		} else if (swerve.getState() == Swerve.ControlState.VELOCITY) {
			swerve.setState(Swerve.ControlState.MANUAL);
		}*/

		//s.sendManualInput(-operator.getY(Hand.kLeft), -operator.getY(Hand.kRight), /*-operator.getY(Hand.kLeft)*/0.0);

		/* ////// Official Controls //////

		if(!s.isClimbing()){
			if (operator.startButton.shortReleased()) {
				if(operator.leftTrigger.isBeingPressed()){
					if(!swerve.isTracking()){
						//diskIntake.loseDisk();
						limelight.setPipeline(Pipeline.CLOSEST);
						s.humanLoaderTrackingState();
					}
				}else{
					s.diskIntakingState();
				}
			} else if(operator.rightTrigger.shortReleased()){
				s.ballScoringState();
			} else if (operator.rightBumper.shortReleased()) {
				s.diskIntakingState();
			} else if (driver.rightTrigger.shortReleased()) {
				diskIntake.conformToState(DiskIntake.State.HOLDING);
				ballCarriage.conformToState(BallCarriage.State.EJECTING);
			} else if(driver.rightTrigger.longPressed()){
				diskIntake.conformToState(DiskIntake.State.HOLDING);
				ballCarriage.conformToState(BallCarriage.State.EJECTING, Constants.kBallCarriageWeakEjectOutput);
			} else if(driver.yButton.shortReleased()){
				//diskIntake.conformToState(diskIntake.State.SCORING, Constants.kDiskEjectTreemap.getInterpolated(new InterpolatingDouble(elevator.getHeight())).value);
				s.diskScoringState();
				robotCentric = false;
			} else if (operator.aButton.wasActivated()) {
				s.ballIntakingState();
			} else if (operator.aButton.wasReleased()) {
				//s.fullBallCycleState();
			} else if (operator.leftTrigger.wasActivated()) {
				if(!swerve.isTracking()){
				}
			} else if (operator.xButton.wasActivated()) {
				if(operator.leftTrigger.isBeingPressed()){
					if(!swerve.isTracking()){
						if(diskIntake.hasDisk() || diskIntake.isExtended()){
							limelight.setPipeline(Pipeline.LOWEST);
							s.diskTrackingState();
						}
					}
				}else{
					if(diskIntake.hasDisk() || diskIntake.isExtended()){
						s.diskScoringState();
					}
				}
			} else if (operator.yButton.wasActivated()) {
				if(operator.leftTrigger.isBeingPressed()){
					if(!swerve.isTracking()){
						if(diskIntake.hasDisk() || diskIntake.isExtended()){
							limelight.setPipeline(Pipeline.LOWEST);
							s.diskTrackingState();
						}
					}
				}else{
					if(diskIntake.hasDisk() || diskIntake.isExtended()){
						s.diskScoringState();
					}else if(ballCarriage.getState() != BallCarriage.State.RECEIVING){
						s.ballScoringState();
					}
				}
			} else if (operator.bButton.shortReleased()) {
				if(operator.leftTrigger.isBeingPressed()){
					if(!swerve.isTracking()){
						if(diskIntake.hasDisk() || diskIntake.isExtended()){
							limelight.setPipeline(Pipeline.LOWEST);
							s.diskTrackingState();
						}
					}
				}else{
					if(diskIntake.hasDisk() || diskIntake.isExtended()){
						s.diskScoringState();
					}else{
						s.ballScoringState();
					}
				}
			} else if (operator.bButton.longPressed()) {
				s.diskScoringState();
			} else if (operator.rightBumper.longPressed()) {
				diskIntake.conformToState(DiskIntake.State.SCORING);
			} else if (operator.leftCenterClick.shortReleased()) {
				ballIntake.conformToState(BallIntake.State.EJECTING);
			} else if(driver.leftCenterClick.shortReleased()){
				limelight.setPipeline(Pipeline.LOWEST);
				s.diskTrackingState();
			} else if(driver.POV180.wasActivated()){
				swerve.setState(Swerve.ControlState.MANUAL);
				robotCentric = false;
			} else if (driver.POV270.wasActivated()) {
				robotCentric = !robotCentric;
				if(robotCentric){
					driver.rumble(1.0, 1.0);
				}
			}
		}else{
			
		} */

		if (diskIntake.needsToNotifyDivers() || ballIntake.needsToNotifyDrivers() || swerve.needsToNotifyDrivers()) {
			driver.rumble(1.0, 2.0);
			operator.rumble(1.0, 2.0);
		}
	}

	private void oneControllerMode() {

	}
}
