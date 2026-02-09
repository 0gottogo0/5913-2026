// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.Constants.HopperConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

    private TalonFX hopper = new TalonFX(MotorID);
    private TalonFXConfiguration hopperConfig = new TalonFXConfiguration();

    private VelocityVoltage hopperVelocityVoltage = new VelocityVoltage(0);

    public Hopper() {
        hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  	  	hopperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		hopperConfig.Slot0.kV = PIDkV;
		hopperConfig.Slot0.kP = PIDkP;
		hopperConfig.Slot0.kI = PIDkI;
		hopperConfig.Slot0.kD = PIDkD;


  	  	hopper.clearStickyFaults();
  	  	hopper.getConfigurator().apply(hopperConfig);
    }

    @Override
    public void periodic() {
        
    }
}
