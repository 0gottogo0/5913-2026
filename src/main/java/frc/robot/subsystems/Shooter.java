// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.Constants.ShooterConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShooterConstants.State;

public class Shooter extends SubsystemBase {

  	private TalonFX bottomShooter = new TalonFX(BottomMotorID);
  	private TalonFXConfiguration bottomShooterConfig = new TalonFXConfiguration();

	// Sadly we cant put a pid on the top shooter motor because we
	// have no encoder on it, percent output should be good enough
	// for our purposes though
	private TalonSRX topShooter = new TalonSRX(TopMotorID);

	private VelocityVoltage shooterVelocityVoltage = new VelocityVoltage(0);

  	private double bottomTargetSpeed = 0;
	private double topTargetSpeed = 0;

	public State state = State.Idle;

  	public Shooter() {
  	  	bottomShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	bottomShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		bottomShooterConfig.Slot0.kV = PIDkV;
		bottomShooterConfig.Slot0.kP = PIDkP;
		bottomShooterConfig.Slot0.kI = PIDkI;
		bottomShooterConfig.Slot0.kD = PIDkD;

  	  	bottomShooter.clearStickyFaults();
  	  	bottomShooter.getConfigurator().apply(bottomShooterConfig);

    	topShooter.clearStickyFaults();
		topShooter.setNeutralMode(NeutralMode.Coast);
	}

  	@Override
  	public void periodic() {
  	  	if (state == State.Idle) {
			bottomShooter.set(0.00);
			topShooter.set(ControlMode.PercentOutput, 0.00);
		} else if (state == State.Spinup) {
			bottomShooter.setControl(shooterVelocityVoltage.withVelocity(bottomTargetSpeed));
			topShooter.set(ControlMode.PercentOutput, topTargetSpeed);
		} else if (state == State.Shoot) {
			// Overshoot target RPS due to heavy ball compression slowing down shooter
			bottomShooter.setControl(shooterVelocityVoltage.withVelocity(bottomTargetSpeed + ShootRPSAdjustment));
			topShooter.set(ControlMode.PercentOutput, topTargetSpeed);
		} else if (state == State.Unstick){
			bottomShooter.setControl(shooterVelocityVoltage.withVelocity(UnstickRPS));
			topShooter.set(ControlMode.PercentOutput, topTargetSpeed);
		} else {
			bottomShooter.set(bottomTargetSpeed);
			topShooter.set(ControlMode.PercentOutput, topTargetSpeed);
		}

		SmartDashboard.putNumber("Bottom Shooter RPS", getShooterSpeed());
		SmartDashboard.putNumber("Bottom Shooter Target RPS", bottomTargetSpeed);
		SmartDashboard.putNumber("Bottom Shooter Target Diff", bottomTargetSpeed - getShooterSpeed());
		SmartDashboard.putNumber("Bottom Shooter Target Percentage", Math.abs(getShooterSpeed() / bottomTargetSpeed - 1));

		SmartDashboard.putString("Shooter State", state.toString());

		SmartDashboard.putBoolean("Shooter At Speed", isShooterAtSpeed());
  	}

	/**
	 * Returns the speed of the shooter motor
	 * 
	 * @return Speed in RPS
	 */
	private double getShooterSpeed() {
		return bottomShooter.getRotorVelocity().getValueAsDouble();
	}

	/**
	 * Sets the state of the shooter.
	 * <p> 
	 * If wanting to control the shooter without PID
	 * then use setShooterDumbControl()
	 * 
	 * @param stateToChangeTo Using ShooterConstants.State
	 * @param bottomSpeedInRPS The target speed to set in rps.
	 * 					 	   When setting the state to idle,
	 * 					 	   target speed is not used and can
	 * 						   be set to 0.	
	 * @param topSpeedInRPS The target speed to set in percent.
	 * 					   When setting the state to idle,
	 * 					   target speed is not used and can
	 * 					   be set to 0.
	 */
	public void setShooterState(State stateToChangeTo, double bottomSpeedInRPS, double topSpeedInRPS) {
		bottomTargetSpeed = bottomSpeedInRPS;
		topTargetSpeed = topSpeedInRPS;
		state = stateToChangeTo;
	}

	/**
	 * Sets the state of the shooter to DumbControl
	 * <p>
	 * Used if want to control the shooter open loop without
	 * the PID. Uses the TalonFX .set() function 
	 * 
	 * @param bottomSpeedInPercent The speed to control in percent
	 * @param topSpeedInPercent The speed to control in percent
	 */
	public void setShooterDumbControl(double bottomSpeedInPercent, double topSpeedInPercent) {
		bottomTargetSpeed = bottomSpeedInPercent;
		topTargetSpeed = topSpeedInPercent;
		state = State.DumbControl;
	}

	/**
	 * Checks if the shooter is up to speed then
	 * returns a boolean, uses ShooterConstants.RPSThreshold
	 * <p>
	 * Note: We only have an encoder on the bottom
	 * shooter so it only checks the bottom shooter
	 * motor if it is up to speed
	 * 
	 * @return True if the shooter is up to speed
	 */
	public boolean isShooterAtSpeed() {
		if (state == State.Spinup || state == State.Shoot) {
			return Math.abs(getShooterSpeed() - bottomTargetSpeed) < RPSThreshold;
		} else {
			return false;
		}
	}
}
