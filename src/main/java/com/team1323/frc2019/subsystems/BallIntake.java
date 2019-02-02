/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.team1323.frc2019.Ports;
import com.team1323.frc2019.loops.ILooper;
import com.team1323.frc2019.loops.Loop;
import com.team1323.frc2019.subsystems.Subsystem;
import com.team1323.frc2019.subsystems.requests.Prerequisite;
import com.team1323.frc2019.subsystems.requests.Request;
import com.team254.drivers.LazyTalonSRX;
import com.team1323.frc2019.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class BallIntake extends Subsystem {
  private static BallIntake instance = null;
	public static BallIntake getInstance(){
		if(instance == null)
			instance = new BallIntake();
		return instance;
  }
  
  boolean hasBall = false;
	public synchronized boolean hasBall(){
		return hasBall;
  }
  
  private LazyTalonSRX intakeMotor;
  private DigitalInput banner;
  
  public boolean getBanner() {
    return banner.get();
  }

  private BallIntake() {
    intakeMotor = new LazyTalonSRX(Ports.BALL_INTAKE);
    banner = new DigitalInput(Ports.BALL_INTAKE_BANNER);

    intakeMotor.setInverted(false);

    intakeMotor.setNeutralMode(NeutralMode.Brake);

    intakeMotor.configVoltageCompSaturation(12.0, 10);
    intakeMotor.enableVoltageCompensation(true);
    
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, 10);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, 10);

  }

  public void setCurrentLimit(int amps) {
    intakeMotor.configContinuousCurrentLimit(amps, 10);
    intakeMotor.configPeakCurrentDuration(10, 10);
    intakeMotor.enableCurrentLimit(true);
  }

  public void enableCurrrentLimit(boolean enable) {
    intakeMotor.enableCurrentLimit(enable);
  }

  private void setRampRate(double secondsToMax) {
    intakeMotor.configOpenloopRamp(secondsToMax, 0);
  }

  public enum State {
    OFF(0), INTAKING(Constants.kIntakingOutput), EJECTING(Constants.kIntakingOutput), HOLDING(Constants.kIntakeWeakHoldingOutput);

    public double intakeOutput = 0;

    private State(double intakeSpeed) {
      intakeOutput = intakeSpeed;
    }
  }
  private State currentState = State.OFF;
  private boolean stateChanged = false;
  private double bannerSensorBeganTimestamp = Double.POSITIVE_INFINITY;
  private double stateEnteredTimestamp = 0;
  private double holdingOutput = Constants.kIntakeWeakHoldingOutput;
  private boolean isConstantSuck = false;

  public State getState() {
    return currentState;
  }

  private synchronized void setState(State newState) {
    if(newState != currentState)
      stateChanged = true;
    currentState = newState;
    stateEnteredTimestamp = Timer.getFPGATimestamp();
  }

  public void setHoldingOutput(double output) {
    holdingOutput = output;
  }

  private boolean needsToNotifyDrivers = false;

  public boolean needsToNotifyDrivers() {
    if (needsToNotifyDrivers) {
      needsToNotifyDrivers = false;
      return true;
    }
    return false;
  }

  private void setIntakeSpeed(double output) {
    setRampRate(0.0);
    intakeMotor.set(ControlMode.PercentOutput, output);
  }

  private void holdRollers() {
    setIntakeSpeed(holdingOutput);
  }

  private final Loop loop = new Loop() {

    @Override
    public void onStart(double timestamp) {
      hasBall = false;
      needsToNotifyDrivers = false;
      setState(State.OFF);
      stop();
    }

    @Override
    public void onLoop(double timestamp) {
      switch(currentState) {
        case OFF:
          break;
        case INTAKING:
          if(stateChanged)
            hasBall = false;
          if(banner.get()) {
            if(Double.isInfinite(bannerSensorBeganTimestamp)) {
              bannerSensorBeganTimestamp = timestamp;
            } else {
              if(timestamp - bannerSensorBeganTimestamp > 0.1) {
                hasBall = true;
                needsToNotifyDrivers = true;
              }
            }
          } else if(!Double.isFinite(bannerSensorBeganTimestamp)) {
            bannerSensorBeganTimestamp = Double.POSITIVE_INFINITY;;
          }
          break;
        case EJECTING:
          if(stateChanged) {
            setRampRate(0.0);
            hasBall = false;
          }
          if(timestamp - stateEnteredTimestamp > 2.0) {
            stop();
            setRampRate(Constants.kIntakeRampRate);
          }
          break;
        case HOLDING:
          if(banner.get()) {
            if(isConstantSuck) {
              holdRollers();
              isConstantSuck = false;
            }
          } else {
            if(!isConstantSuck) {
              setIntakeSpeed(Constants.kIntakingResuckingOutput);
              isConstantSuck = true;
            }
          }
        default:
          break;
      }
      if (stateChanged)
        stateChanged = false;
    }

    @Override
    public void onStop(double timestamp) {
      setState(State.OFF);
      stop();
    }

  };

  public void eject(double output) {
    setState(State.EJECTING);
    setIntakeSpeed(output);
    hasBall = false;
  }

  private void conformToState(State desiredState) {
    setState(desiredState);
    setIntakeSpeed(desiredState.intakeOutput);
  }

  private void conformToState(State desiredState, double outputOverride) {
    setState(desiredState);
    setIntakeSpeed(outputOverride);
  }

  public Request stateRequest(State desiredState) {
    return new Request() {

      @Override
      public void act() {
        conformToState(desiredState);
      }

    };

  }

  public Request waitForBallRequest() {
    return new Request(){
    
      @Override
      public void act() {
        conformToState(State.INTAKING);
      }

      @Override
      public boolean isFinished() {
        return !stateChanged && hasBall;
      }

    };
  }

  public Request ejectRequest(double output) {
    return new Request(){
    
      @Override
      public void act() {
        conformToState(State.EJECTING, output);
      }
    };
  }

  public Prerequisite ballRequisite() {
    return new Prerequisite(){
    
      @Override
      public boolean met() {
        return hasBall();
      }
    };
  }

  @Override
  public void outputTelemetry() {
    if(Constants.kDebuggingOutput) {
      SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());
      SmartDashboard.putNumber("Intake Voltage", intakeMotor.getMotorOutputVoltage());
      SmartDashboard.putBoolean("Intake Has Ball", hasBall);
      SmartDashboard.putBoolean("Intake Banner", banner.get());
    }

  }

  @Override
  public synchronized void stop() {
    conformToState(State.OFF);
  }

  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    enabledLooper.register(loop);
  }

}
