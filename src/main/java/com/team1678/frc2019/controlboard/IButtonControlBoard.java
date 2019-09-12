package com.frc1678.c2019.controlboard;

public interface IButtonControlBoard {

    // Cargo Intake
    boolean getRunIntake();
    
    boolean getRunOuttake();

    // Hatch Intake
    boolean getScoreHatch();

    void setRumble(boolean on);
}