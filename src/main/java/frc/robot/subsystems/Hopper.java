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
import frc.robot.constants.Constants.HopperConstants.State;

public class Hopper extends SubsystemBase {

    private TalonFX hopper = new TalonFX(MotorID);
    private TalonFXConfiguration hopperConfig = new TalonFXConfiguration();

    private VelocityVoltage hopperVelocityVoltage = new VelocityVoltage(0);

    private double targetSpeed = 0.00;

    public State state = State.Idle;

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
         if (state == State.Idle) {
            hopper.set(0.00);
        } else if (state == State.Intake) {
            hopper.setControl(hopperVelocityVoltage.withVelocity(IntakeingSpeed));
        } else if (state == State.Outtake) {
            hopper.setControl(hopperVelocityVoltage.withVelocity(-IntakeingSpeed));
        } else {
            hopper.set(targetSpeed);
        }
    }

    /**
	 * Sets the state of the hopper.
	 * <p> 
	 * If wanting to control the hopper without PID
	 * then use setHopperDumbControl()
	 * 
	 * @param stateToChangeTo Using HopperConstants.State
	 */
    public void setHopperState(State stateToChangeTo) {
        state = stateToChangeTo;
    }

	/**
	 * Sets the state of the hopper to DumbControl
	 * <p>
	 * Used if want to control the hopper open loop without
	 * the PID. Uses the TalonFX .set() function 
	 * 
	 * @param speedInPercent The speed to control the hopper
	 * 						 motor in percent
	 */
    public void setHopperDumbControl(double speedInPercent) {
        targetSpeed = speedInPercent;
        state = State.DumbControl;
    }
}
