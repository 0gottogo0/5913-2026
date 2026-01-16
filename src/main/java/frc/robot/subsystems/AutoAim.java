// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoAim extends SubsystemBase {

    private double robotPos[] = {0.00, 0.00, 0.00};
    private double robotSpeed[] = {0.00, 0.00, 0.00};

    public AutoAim() {

    }

    @Override
    public void periodic() {
      
    }

    public void setAutoAimPos(double x, double y, double rot) {
        robotPos[0] = x;
        robotPos[1] = y;
        robotPos[2] = rot;
    }

    public void setAutoAimSpeed(double x, double y, double rot) {
        robotSpeed[0] = x;
        robotSpeed[1] = y;
        robotSpeed[2] = rot;
    }

    public double setAimTargetInDegrees() {
        return 0.00;
    }
    
    public double setAimTargetInDistace() {
        return 0.00;
    }
}
