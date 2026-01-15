// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

    private TalonFX feeder = new TalonFX(1); // create constants and change motor names
    private TalonFXConfiguration feederConfig = new TalonFXConfiguration();

    private boolean runFeeder = false;

    public Feeder() {
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  	  	feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		
  	  	feeder.clearStickyFaults();
  	  	feeder.getConfigurator().apply(feederConfig);
    }

    @Override
    public void periodic() {
        if (runFeeder) {
            feeder.set(0.60);
        } else {
            feeder.set(0.00);
        }
        
        SmartDashboard.putBoolean("Feeder Running", runFeeder);
    }

    public void startFeeder() {
        runFeeder = true;
    }

    public void stopFeeder() {
        runFeeder = false;
    }
}
