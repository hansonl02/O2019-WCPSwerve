/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team1323.frc2019.Constants;
import com.team1323.frc2019.Ports;
import com.team254.drivers.LazyTalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 */
public class BallCarriage extends Subsystem{
    private static BallCarriage instance = null;
    public static BallCarriage getInstance(){
        if(instance == null)
            instance = new BallCarriage();
        return instance;
    }

    LazyTalonSRX motor;

    public BallCarriage(){
        motor = new LazyTalonSRX(Ports.BALL_CARRIAGE);
        motor.configVoltageCompSaturation(12.0);
        motor.enableVoltageCompensation(true);
        setOpenLoop(0.0);
    }

    public synchronized void setOpenLoop(double percentOutput){
        motor.set(ControlMode.PercentOutput, percentOutput);
    }

    @Override
    public void outputTelemetry() {
        if(Constants.kDebuggingOutput){
            SmartDashboard.putNumber("Ball Carriage Voltage", motor.getMotorOutputVoltage());
            SmartDashboard.putNumber("Ball Carriage Current", motor.getOutputCurrent());
        }
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }
}