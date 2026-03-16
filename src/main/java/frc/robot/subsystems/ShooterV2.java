// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterV2 extends SubsystemBase {

	// Placeholder names

	// Kraken X60
	private TalonFX Right3 = new TalonFX(50);
    private TalonFXConfiguration Right3Config = new TalonFXConfiguration();

	// Kraken X60
	private TalonFX Left4 = new TalonFX(51);
    private TalonFXConfiguration Left4Config = new TalonFXConfiguration();

	// Kraken X60
  	private TalonFX Right4 = new TalonFX(52);
  	private TalonFXConfiguration Right4Config = new TalonFXConfiguration();

	// Kraken X44
	private TalonFX Left2 = new TalonFX(53);
  	private TalonFXConfiguration Left2Config = new TalonFXConfiguration();

	private double right3TargetSpeed = 0.00;
	private double left4TargetSpeed = 0.00;
  	private double right4TargetSpeed = 0.00;
	private double left2TargetSpeed = 0.00;

	public State state = State.Idle;

  	public ShooterV2() {
		Right3Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		Right3Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		Right3.clearStickyFaults();
		Right3.getConfigurator().apply(Right3Config);

        Left4Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	Left4Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        Left4.clearStickyFaults();
  	  	Left4.getConfigurator().apply(Left4Config);

  	  	Right4Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	Right4Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

  	  	Right4.clearStickyFaults();
  	  	Right4.getConfigurator().apply(Right4Config);

        Left2Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	Left2Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

  	  	Left2.clearStickyFaults();
  	  	Left2.getConfigurator().apply(Left2Config);
	}

  	@Override
  	public void periodic() {
  	  	switch (state) {
			case Idle:
				Right3.set(0.00);
				Left4.set(0.00);
				Right4.set(0.00);
				Left2.set(0.00);
				break;
			case Spinup:
				Right3.set(0.00);
				Left4.set(0.00);
				Right4.set(0.00);
				Left2.set(0.00);
				break;
			case Shoot:
				Right3.set(0.00);
				Left4.set(0.00);
				Right4.set(0.00);
				Left2.set(0.00);
				break;
			case Unstick:
				Right3.set(0.00);
				Left4.set(0.00);
				Right4.set(0.00);
				Left2.set(0.00);
				break;
			case DumbControl:
				Right3.set(-right3TargetSpeed);
				Left4.set(-left4TargetSpeed);
				Right4.set(right4TargetSpeed);
				Left2.set(left2TargetSpeed);
				break;
		} 

		SmartDashboard.putString("ShooterV2 State", state.toString());
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
	 * 
	 * @param hoodSpeedInRPS The target speed to set in rps.
	 * 					     When setting the state to idle,
	 * 					     target speed is not used and can
	 * 					     be set to 0.
	 */
	public void setShooterState(State stateToChangeTo, double speedInRPS, double hoodSpeedInRPS) {
        right4TargetSpeed = speedInRPS;
		left2TargetSpeed = hoodSpeedInRPS;
		state = stateToChangeTo;
	}

	/**
	 * Sets the state of the shooter to DumbControl
	 * <p>
	 * Used if want to control the shooter open loop without
	 * the PID. Uses the TalonFX .set() function 
	 * 
	 * @param bottomSpeedInPercent The speed to control in percent
	 * 
	 * @param topSpeedInPercent The speed to control in percent
	 * 
	 * @param hoodSpeedInPercent The speed to control in percent
	 */
	public void setShooterDumbControl(double bottomSpeedInPercent, double topSpeedInPercent, double hoodSpeedInPercent) {
		right3TargetSpeed = bottomSpeedInPercent;
		left4TargetSpeed = topSpeedInPercent;
        right4TargetSpeed = topSpeedInPercent;
		left2TargetSpeed = hoodSpeedInPercent;
		state = State.DumbControl;
	}
}
