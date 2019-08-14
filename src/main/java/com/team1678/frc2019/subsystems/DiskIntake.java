/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1678.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.team1678.frc2019.Ports;
import com.team1678.frc2019.loops.ILooper;
import com.team1678.frc2019.loops.Loop;
import com.team1678.frc2019.subsystems.Subsystem;
import com.team1678.frc2019.subsystems.requests.Prerequisite;
import com.team1678.frc2019.subsystems.requests.Request;
import com.team254.drivers.LazyTalonSRX;
import com.team1678.frc2019.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Manages the disk intake
 **/
public class DiskIntake extends Subsystem {
  private static DiskIntake instance = null;
  public static DiskIntake getInstance() {
    if(instance == null) 
      instance = new DiskIntake();
    return instance;
  }

  private boolean hasDisk = false;
  public synchronized boolean hasDisk() {
    return hasDisk;
  }
  public void feignDisk(){
    hasDisk = false;
  }

  private LazyTalonSRX pumpMotor;
  public LazyTalonSRX getTalon(){
    return pumpMotor;
  }

  private Solenoid extend;
  private Solenoid release;

  private DiskIntake() {
    pumpMotor = new LazyTalonSRX(Ports.DISK_INTAKE);
    extend = new Solenoid(Ports.DRIVEBASE_PCM, Ports.DISK_INTAKE_EXTEND);
    release = new Solenoid(Ports.DRIVEBASE_PCM, Ports.DISK_INTAKE_RELEASE);

    pumpMotor.setInverted(false);

    pumpMotor.setNeutralMode(NeutralMode.Brake);

    pumpMotor.configVoltageCompSaturation(12.0, 10);
    pumpMotor.enableVoltageCompensation(true);
  }

  private void configureTalon(){
    pumpMotor.configForwardSoftLimitEnable(false);
    pumpMotor.configReverseSoftLimitEnable(false);

    setCurrentLimit(Constants.kJackCurrentLimit);
  }

  public void setCurrentLimit(int amps) {
    pumpMotor.configContinuousCurrentLimit(amps, 10);
    pumpMotor.configPeakCurrentLimit(amps, 10);
    pumpMotor.configPeakCurrentDuration(10, 10);
    pumpMotor.enableCurrentLimit(true);
  }

  public void enableCurrentLimit(boolean enable) {
    pumpMotor.enableCurrentLimit(enable);
  }

  private void setRampRate(double secondsToMax) {
    pumpMotor.configOpenloopRamp(secondsToMax, 0);
  }

  public enum State {
    OFF(0, false, false), 
    INTAKING(Constants.kPumpBuildOutput, true, false),
    BUILDING_PRESSURE(Constants.kPumpBuildOutput, false, false),
    HOLDING(0, false, false),
    SCORING(0, true, true);

    public double pumpPower = 0;
    public boolean extended = false;
    public boolean released = false;

    private State(double output, boolean extend, boolean release) {
      pumpPower = output;
      extended = extend;
      released = release;
    }

  }

  private State currentState = State.OFF;
  public State getState() {
    return currentState;
  }

  private double stateEnteredTimestamp = 0;
  private boolean stateChanged = false;
  private double currentSpikeTimestamp = Double.POSITIVE_INFINITY;
  private boolean needsToNotifyDrivers = false;

  private synchronized void setState(State newState) {
    if (newState != currentState) {
      stateChanged = true;
    }
    currentState = newState;
    stateEnteredTimestamp = Timer.getFPGATimestamp();
  }

  public boolean needsToNotifyDivers() {
    if (needsToNotifyDrivers) {
      needsToNotifyDrivers = false;
      return true;
    }
    return false;
  }

  public void fireExtend(boolean fire) {
    extend.set(fire);
  }

  public void fireRelease(boolean fire) {
    release.set(!fire);
  }

  private void setPump(double power) {
    setRampRate(0.0);
    pumpMotor.set(ControlMode.PercentOutput, power);
  }

  private void buildPressure() {
    setPump(Constants.kPumpBuildOutput);
  }

  private void holdPressure() {
    setPump(Constants.kPumpHoldOutput)
  }

  private final Loop loop = new Loop() {

    @Override
    public void onStart(double timestamp) {
      hasDisk = false;
      needsToNotifyDrivers = false;
      setState(State.OFF);
      stop();
    }

    @Override
    public void onLoop(double timestamp) {

      switch (currentState) {
        case OFF:
          
          break;
        case INTAKING:
          if(stateChanged)
            hasDisk = false;
          if(pumpMotor.getOutputCurrent() >= 10.0 && (timestamp - stateEnteredTimestamp) >= 0.5) {
            if(Double.isInfinite(currentSpikeTimestamp)) {
              currentSpikeTimestamp = timestamp;
            } else {
              if(timestamp - currentSpikeTimestamp > 0.375) {
                hasDisk = true;
                needsToNotifyDrivers = true;
              }
            }
          } else if (!Double.isInfinite(currentSpikeTimestamp)) {
            currentSpikeTimestamp = Double.POSITIVE_INFINITY;
          }
          break;
        case SCORING:
          if(stateChanged) {
            //setRampRate(0.0);
            hasDisk = false;
          }
          if (timestamp - stateEnteredTimestamp > 2.0) {
            stop();
            //setRampRate(Constants.kDiskIntakeRampRate);
          }
          break;
        case HOLDING:
          /*if(banner.get()) {
            if(isResucking) {
              holdRollers();
              isResucking = false;
            }
          } else {
            if (!isResucking) {
              setRollers(Constants.kDiskIntakingResuckingOutput);
              isResucking = true;
            }
          }*/
          break;
        default:
          break;
      }

      if(stateChanged)
        stateChanged = false;

    }

    @Override
    public void onStop(double timestamp) {
      setState(State.OFF);
      stop();
    }

  };

  public void drop() {
    setState(State.OFF);
    fireRelease(true);
    hasDisk = false;
  }

  public void conformToState(State desiredState) {
    conformToState(desiredState, desiredState.pumpPower);
  }

  public void conformToState(State desiredState, double outputOverride) {
    setState(desiredState);
    setPump(outputOverride);
    fireExtend(desiredState.extended);
    fireRelease(desiredState.released);
  }

  public Request stateRequest(State desiredState) {
    return new Request(){
    
      @Override
      public void act() {
        conformToState(desiredState);
      }
    };
  }

  public Request waitForDiskRequest() {
    return new Request(){
    
      @Override
      public void act() {
        conformToState(State.INTAKING);
      }

      @Override
      public boolean isFinished() {
        return !stateChanged && hasDisk;
      }

    };
  }

  public Request scoreRequest(double output) {
    return new Request(){
    
      @Override
      public void act() {
        conformToState(State.SCORING, output);
      }

    };
  }

  public Prerequisite diskRequisite() {
    return new Prerequisite(){
    
      @Override
      public boolean met() {
        return hasDisk();
      }
    };
  }

  @Override
  public synchronized void stop() {
    conformToState(State.OFF);
  }

  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    enabledLooper.register(loop);
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putString("Disk intake state", currentState.toString());
    if(Constants.kDebuggingOutput) {
      SmartDashboard.putNumber("Disk Intake Pump Current", pumpMotor.getOutputCurrent());
      SmartDashboard.putNumber("Disk Intake Pump Voltage", pumpMotor.getMotorOutputVoltage());
      SmartDashboard.putBoolean("Disk Intake Has Disk", hasDisk);
    }

  }
}
