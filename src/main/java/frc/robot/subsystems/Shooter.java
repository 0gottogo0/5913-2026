// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  	private TalonFX shooter = new TalonFX(MotorID);
  	private TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

	private VelocityVoltage shooterVelocityVoltage = new VelocityVoltage(0);

  	private double targetSpeed = 0;

	public State state = State.Idle;

  	public Shooter() {
  	  	shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		shooterConfig.Slot0.kV = PIDkV;
		shooterConfig.Slot0.kP = PIDkP;
		shooterConfig.Slot0.kI = PIDkI;
		shooterConfig.Slot0.kD = PIDkD;

  	  	shooter.clearStickyFaults();
  	  	shooter.getConfigurator().apply(shooterConfig);
	}

  	@Override
  	public void periodic() {
  	  	if (state == State.Idle) {
			shooter.set(0.00);
		} else if (state == State.Spinup) {
			shooter.setControl(shooterVelocityVoltage.withVelocity(targetSpeed));
		} else if (state == State.Shoot) {
			// Overshoot target RPS due to heavy ball compression slowing down shooter
			shooter.setControl(shooterVelocityVoltage.withVelocity(targetSpeed + ShootRPSAdjustment));
		} else if (state == State.Unstick){
			shooter.setControl(shooterVelocityVoltage.withVelocity(UnstickRPS));
		} else {
			shooter.set(targetSpeed);
		}

		SmartDashboard.putNumber("Shooter RPS", getShooterSpeed());
		SmartDashboard.putNumber("Shooter Target RPS", targetSpeed);
		SmartDashboard.putNumber("Shooter Target Diff", targetSpeed - getShooterSpeed());
		SmartDashboard.putNumber("Shooter Target Percentage", Math.abs(getShooterSpeed() / targetSpeed - 1));

		SmartDashboard.putString("Shooter State", state.toString());

		SmartDashboard.putBoolean("Shooter At Speed", isShooterAtSpeed());
  	}

	/**
	 * Returns the speed of the shooter motor
	 * 
	 * @return Speed in RPS
	 */
	private double getShooterSpeed() {
		return shooter.getRotorVelocity().getValueAsDouble();
	}

	/**
	 * Sets the state of the shooter.
	 * <p> 
	 * If wanting to control the shooter without PID
	 * then use setShooterDumbControl()
	 * 
	 * @param stateToChangeTo Using ShooterConstants.State
	 * @param speedInRPS The target speed to set in rps.
	 * 					 When setting the state to idle,
	 * 					 target speed is not used and can
	 * 					 be set to 0.	
	 */
	public void setShooterState(State stateToChangeTo, double speedInRPS) {
		targetSpeed = speedInRPS;
		state = stateToChangeTo;
	}

	/**
	 * Sets the state of the shooter to DumbControl
	 * <p>
	 * Used if want to control the shooter open loop without
	 * the PID. Uses the TalonFX .set() function 
	 * 
	 * @param speedInPercent The speed to control the shooter
	 * 						 motor in percent
	 */
	public void setShooterDumbControl(double speedInPercent) {
		targetSpeed = speedInPercent;
		state = State.DumbControl;
	}

	/**
	 * Checks if the shooter is up to speed then
	 * returns a boolean
	 * 
	 * @return If the shooter is up to speed
	 */
	public boolean isShooterAtSpeed() {
		if (state == State.Spinup || state == State.Shoot) {
			return Math.abs(getShooterSpeed() - targetSpeed) < RPSThreshold;
		} else {
			return false;
		}
	}
}
