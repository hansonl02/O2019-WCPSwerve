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
 * Manages the pump for the disk intake
 **/
public class DiskIntake extends Subsystem {