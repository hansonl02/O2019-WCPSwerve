package com.team1678.frc2019.subsystems;

import com.team1678.frc2019.Constants;
import com.team1678.frc2019.loops.ILooper;
import com.team1678.frc2019.loops.Loop;
import com.team1678.frc2019.Ports;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

import com.team254.lib.util.TimeDelayedBoolean;

public class BallIntake extends Subsystem {
    // Intaking is positive
    public static double kIntakeVoltage = -12.0;
    public static double kHoldingVoltage = 0;
    public static double kCarriageVoltage = 6.0;
    public static double kOuttakeVoltage = 10.0;

    private boolean mReceivedCargo = false;

    private static BallIntake mInstance;
    
    private DigitalInput mProxy;

    public enum WantedAction {
        NONE, INTAKE, OUTTAKE,
    }

    private enum State {
        INTAKING, OUTTAKING, HOLDING,
    }

    private State mState = State.HOLDING;

    private TimeDelayedBoolean mLastSeenCargo = new TimeDelayedBoolean();
    private boolean mDebouncedCargo = false;

    private boolean mRunningManual = false;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private final TalonSRX mMaster;
    private final TalonSRX mCarriage;
    private final Solenoid mPopoutSolenoid;

    private BallIntake() {
        mPopoutSolenoid = new Solenoid(Ports.CARRIAGE_PCM, Ports.BALL_INTAKE_EXTEND);

        mMaster = new TalonSRX(6);
        mCarriage = new TalonSRX(10);
        mMaster.configFactoryDefault();
        mCarriage.configFactoryDefault();

        mProxy = new DigitalInput(3);

        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(false);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);

        mCarriage.set(ControlMode.PercentOutput, 0);
        mCarriage.setInverted(false);
        mCarriage.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mCarriage.enableVoltageCompensation(true);
    }

    public synchronized static BallIntake getInstance() {
        if (mInstance == null) {
            mInstance = new BallIntake();
        }
        return mInstance;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putBoolean("CargoProxy", mPeriodicIO.has_cargo);
        SmartDashboard.putNumber("MotorSetpoint", mPeriodicIO.demand);
    }

    @Override
    public void stop() {
        setOpenLoop(0);
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mRunningManual = false;
                mState = State.HOLDING;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (BallIntake.this) {
                    if (mRunningManual) {
                        runStateMachine(false);
                        return;
                    } else {
                        runStateMachine(true);
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                mRunningManual = false;
                mState = State.HOLDING;
            }
        });
    }

    public synchronized boolean getIntakeOut() {
        return mPopoutSolenoid.get()
         && mPeriodicIO.pop_out_solenoid; // Cut latency on the hatch intake
    }

    public void runStateMachine(boolean modifyOutputs) {
        switch (mState) {
        case INTAKING:
            if (modifyOutputs) {
                mPeriodicIO.demand = kIntakeVoltage;
                mPeriodicIO.demand_carriage = kCarriageVoltage;
                mPeriodicIO.pop_out_solenoid = true;
            }
            if (hasCargo()) {
                mState = State.HOLDING;
            }
            break;
        case OUTTAKING:
            if (modifyOutputs) {
                mPeriodicIO.demand = 0;
                mPeriodicIO.demand_carriage = kOuttakeVoltage;
                mPeriodicIO.pop_out_solenoid = true;
            } else if (hasCargo()) {
                mState = State.HOLDING;
            }
            break;
        case HOLDING:
            if (modifyOutputs) {
                mPeriodicIO.demand = 0;
                mPeriodicIO.demand_carriage = hasCargo() ? kHoldingVoltage : 0.0;
                if (hasCargo()) {
                    mPeriodicIO.pop_out_solenoid = false;
                }
            }
            break;
        default:
            System.out.println("Fell through on Cargo Intake states!");
        }
    }

    public void forceIntakeIn() {
        mPeriodicIO.pop_out_solenoid = false;
    }

    public boolean hasCargo() {
        return mDebouncedCargo;
    }

    public synchronized void setOpenLoop(double percentage) {
        mRunningManual = true;
        mPeriodicIO.demand = percentage;
    }

    public double getVoltage() {
        return mPeriodicIO.demand;
    }

    public double getCarriageVoltage() {
        return mPeriodicIO.demand_carriage;
    }

    public void setState(WantedAction wanted_state) {
        mRunningManual = false;
        switch (wanted_state) {
        case NONE:
            mState = State.HOLDING;
            break;
        case INTAKE:
            mState = State.INTAKING;
            break;
        case OUTTAKE:
            mState = State.OUTTAKING;
            break;
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mDebouncedCargo = mLastSeenCargo.update(!mProxy.get(), 0.1);
        mReceivedCargo = mDebouncedCargo != mPeriodicIO.has_cargo;
        mPeriodicIO.has_cargo = mDebouncedCargo;
        mPeriodicIO.cargo_proxy = !mProxy.get();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
        mCarriage.set(ControlMode.PercentOutput, mPeriodicIO.demand_carriage / 12.0);

        mPopoutSolenoid.set(mPeriodicIO.pop_out_solenoid);
    }

    public boolean needsToNotifyDrivers() {
        return mReceivedCargo;
    }

    public static class PeriodicIO {
        // INPUTS
        public boolean has_cargo;
        public boolean cargo_proxy;

        // OUTPUTS
        public double demand;
        public double demand_carriage;
        public boolean pop_out_solenoid;
    }
}