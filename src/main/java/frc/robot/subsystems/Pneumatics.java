// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.Constants.PneumaticConstants.*;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  
    private final Compressor compressor = new Compressor(PneumaticsHubID, PneumaticsModuleType.CTREPCM);

    public Pneumatics() {}
  
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Pressure Switch", compressor.getPressureSwitchValue());
        SmartDashboard.putBoolean("Compressor Enabled", compressor.isEnabled());
    }
}