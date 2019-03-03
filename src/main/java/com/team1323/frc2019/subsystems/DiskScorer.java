/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team1323.frc2019.Ports;
import com.team1323.frc2019.loops.ILooper;
import com.team1323.frc2019.loops.Loop;
import com.team1323.frc2019.subsystems.requests.Request;
import com.team254.drivers.LazyTalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.team1323.frc2019.Constants;

/**
 * 
 */
public class DiskScorer extends Subsystem {
    private static DiskScorer instance = null;

    public static DiskScorer getInstance() {
        if(instance == null)
            instance = new DiskScorer();
        return instance;
    }

    Solenoid extender;
    LazyTalonSRX motor;
    DigitalInput banner;

    public boolean getBanner(){
        return banner.get();
    }

    public DiskScorer(){
        extender = new Solenoid(Ports.DRIVEBASE_PCM, Ports.PROBE_EXTENDER);
        motor = new LazyTalonSRX(Ports.DISK_SCORER);

        motor.setInverted(true);
        setCurrentLimit(20);

        banner = new DigitalInput(Ports.DISK_INTAKE_BANNER);
    }

    private void setCurrentLimit(int amps){
        motor.configContinuousCurrentLimit(amps);
        motor.configPeakCurrentLimit(amps);
        motor.configPeakCurrentDuration(10);
        motor.enableCurrentLimit(true);
    }

    public enum State{
        SCORING(true, Constants.kDiskScorerEjectOutput), STOWED(false, 0.0),
        HOLDING(true, Constants.kDiskScorerHoldingOutput), RECEIVING(true, Constants.kDiskScorerIntakingOutput),
        STOWED_HOLDING(false, Constants.kDiskScorerHoldingOutput),
        GROUND_INTAKING(false, Constants.kDiskScorerIntakingOutput),
        NEUTRAL_EXTENDED(true, 0.0), AUTO_RECEIVING(true, Constants.kDiskScorerIntakingOutput),
        DETECTED(true, Constants.kDiskScorerIntakingOutput),
        GROUND_DETECTED(false, Constants.kDiskScorerIntakingOutput);

        boolean extended;
        double output;

        private State(boolean extended, double output){
            this.extended = extended;
            this.output = output;
        }
    }
    private State currentState = State.STOWED;
    public State getState(){ return currentState; }
    private synchronized void setState(State newState) {
        if (newState != currentState)
          stateChanged = true;
        currentState = newState;
        stateEnteredTimestamp = Timer.getFPGATimestamp();
    }

    public boolean isExtended(){
        return currentState.extended;
    }

    private boolean stateChanged = false;
    private double bannerSensorBeganTimestamp = Double.POSITIVE_INFINITY;
    private double stateEnteredTimestamp = 0;

    private boolean hasDisk = false;
    public boolean hasDisk(){ return hasDisk; }
    public void feignDisk(){ hasDisk = true; }

    private boolean needsToNotifyDrivers = false;
    public boolean needsToNotifyDrivers() {
        if (needsToNotifyDrivers) {
        needsToNotifyDrivers = false;
        return true;
        }
        return false;
    }

    private void setOpenLoop(double output){
        motor.set(ControlMode.PercentOutput, output);
    }

    public void conformToState(State newState){
        extender.set(newState.extended);
        setOpenLoop(newState.output);
        setState(newState);
    }

    public Request stateRequest(State newState){
        return new Request(){
        
            @Override
            public void act() {
                conformToState(newState);
            }

        };
    }

    public Request waitForDiskRequest(){
        return new Request(){

            @Override
            public void act(){

            }

            @Override
            public boolean isFinished(){
                return hasDisk() && !stateChanged;
            }

        };
    }

    private final Loop loop = new Loop(){
    
        @Override
        public void onStart(double timestamp) {
            
        }
    
        @Override
        public void onLoop(double timestamp) {
            switch(currentState){
                case SCORING:
                    if(stateChanged)
                        hasDisk = false;
                    if((timestamp - stateEnteredTimestamp) >= 1.0){
                        conformToState(State.NEUTRAL_EXTENDED);
                    }
                    break;
                case STOWED:
                    break;
                case DETECTED:
                    if((timestamp - stateEnteredTimestamp) >= 0.5){
                        conformToState(State.HOLDING);
                    }
                    break;
                case GROUND_DETECTED:
                    if((timestamp - stateEnteredTimestamp) >= 0.5){
                        setOpenLoop(Constants.kDiskScorerHoldingOutput);
                    }
                    break;
                case HOLDING:
                    
                    break;
                case RECEIVING:
                    if (stateChanged)
                        hasDisk = false;
                    if (motor.getOutputCurrent() > 15.0 && (timestamp - stateEnteredTimestamp) > 0.5) {
                        if (Double.isInfinite(bannerSensorBeganTimestamp)) {
                            bannerSensorBeganTimestamp = timestamp;
                        } else {
                            if (timestamp - bannerSensorBeganTimestamp >= 0.5) {
                                hasDisk = true;
                                needsToNotifyDrivers = true;
                            }
                        }
                    } else if (!Double.isFinite(bannerSensorBeganTimestamp)) {
                        bannerSensorBeganTimestamp = Double.POSITIVE_INFINITY;
                    }
                    break;
                case AUTO_RECEIVING:
                    if (stateChanged)
                        hasDisk = false;
                    if (getBanner()) {
                        if (Double.isInfinite(bannerSensorBeganTimestamp)) {
                            bannerSensorBeganTimestamp = timestamp;
                        } else {
                            if (timestamp - bannerSensorBeganTimestamp >= 0.1) {
                                hasDisk = true;
                                needsToNotifyDrivers = true;
                            }
                        }
                    } else if (!Double.isFinite(bannerSensorBeganTimestamp)) {
                        bannerSensorBeganTimestamp = Double.POSITIVE_INFINITY;
                    }
                    break;
                case GROUND_INTAKING:
                    if (stateChanged)
                        hasDisk = false;
                    if (getBanner()) {
                        if (Double.isInfinite(bannerSensorBeganTimestamp)) {
                            bannerSensorBeganTimestamp = timestamp;
                        } else {
                            if (timestamp - bannerSensorBeganTimestamp >= 0.1) {
                                hasDisk = true;
                                needsToNotifyDrivers = true;
                            }
                        }
                    } else if (!Double.isFinite(bannerSensorBeganTimestamp)) {
                        bannerSensorBeganTimestamp = Double.POSITIVE_INFINITY;
                    }
                    break;
                default:
                    break;
            }
            if(stateChanged)
                stateChanged = false;
        }

        @Override
        public void onStop(double timestamp) {
            conformToState(State.STOWED);
        }

    };

    @Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(loop);
	}

    @Override
    public void outputTelemetry() {
        if(Constants.kDebuggingOutput){
            SmartDashboard.putBoolean("Disk Scorer Has Disk", hasDisk());
            SmartDashboard.putBoolean("Disk Scorer Banner", getBanner());
            SmartDashboard.putNumber("Disk Scorer Current", motor.getOutputCurrent());
        }
    }

    @Override
    public void stop() {

    }
}