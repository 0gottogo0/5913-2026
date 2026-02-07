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

  	private TalonFX topShooter = new TalonFX(TopMotorID);
  	private TalonFXConfiguration topShooterConfig = new TalonFXConfiguration();

	private TalonFX hoodShooter = new TalonFX(HoodMotorID);
  	private TalonFXConfiguration hoodShooterConfig = new TalonFXConfiguration();

    private VelocityVoltage bottomShooterVelocityVoltage = new VelocityVoltage(0);
	private VelocityVoltage topShooterVelocityVoltage = new VelocityVoltage(0);
    private VelocityVoltage hoodShooterVelocityVoltage = new VelocityVoltage(0);

	private double bottomTargetSpeed = 0.00;
  	private double topTargetSpeed = 0.00;
	private double hoodTargetSpeed = 0.00;

	public State state = State.Idle;

  	public Shooter() {
        bottomShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	bottomShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		bottomShooterConfig.Slot0.kV = BottomShooterPIDkV;
		bottomShooterConfig.Slot0.kP = BottomShooterPIDkP;
		bottomShooterConfig.Slot0.kI = BottomShooterPIDkI;
		bottomShooterConfig.Slot0.kD = BottomShooterPIDkD;

        bottomShooter.clearStickyFaults();
  	  	bottomShooter.getConfigurator().apply(bottomShooterConfig);

  	  	topShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	topShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		topShooterConfig.Slot0.kV = TopShooterPIDkV;
		topShooterConfig.Slot0.kP = TopShooterPIDkP;
		topShooterConfig.Slot0.kI = TopShooterPIDkI;
		topShooterConfig.Slot0.kD = TopShooterPIDkD;

  	  	topShooter.clearStickyFaults();
  	  	topShooter.getConfigurator().apply(topShooterConfig);

        hoodShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	hoodShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		hoodShooterConfig.Slot0.kV = HoodShooterPIDkV;
		hoodShooterConfig.Slot0.kP = HoodShooterPIDkP;
		hoodShooterConfig.Slot0.kI = HoodShooterPIDkI;
		hoodShooterConfig.Slot0.kD = HoodShooterPIDkD;

  	  	hoodShooter.clearStickyFaults();
  	  	hoodShooter.getConfigurator().apply(hoodShooterConfig);
	}

  	@Override
  	public void periodic() {
  	  	if (state == State.Idle) {
            bottomShooter.set(0.00);
			topShooter.set(0.00);
			hoodShooter.set(0.00);
		} else if (state == State.Shoot) {
            bottomShooter.setControl(bottomShooterVelocityVoltage.withVelocity(topTargetSpeed * BottomRatio));
			topShooter.setControl(topShooterVelocityVoltage.withVelocity(topTargetSpeed));
			hoodShooter.setControl(hoodShooterVelocityVoltage.withVelocity(hoodTargetSpeed));
		} else if (state == State.Unstick){
            bottomShooter.setControl(bottomShooterVelocityVoltage.withVelocity(UnstickRPS));
			topShooter.setControl(topShooterVelocityVoltage.withVelocity(UnstickRPS));
			hoodShooter.setControl(hoodShooterVelocityVoltage.withVelocity(UnstickRPS));
		} else {
            bottomShooter.set(bottomTargetSpeed);
			topShooter.set(topTargetSpeed);
			hoodShooter.set(hoodTargetSpeed);
		}

        SmartDashboard.putNumber("Bottom Shooter RPS", getBottomShooterSpeed());
		SmartDashboard.putNumber("Bottom Shooter Target RPS", topTargetSpeed * BottomRatio);
		SmartDashboard.putNumber("Bottom Shooter Target Diff", (topTargetSpeed * BottomRatio) - getBottomShooterSpeed());
		SmartDashboard.putNumber("Bottom Shooter Target Percentage", Math.abs(getBottomShooterSpeed() / (topTargetSpeed * BottomRatio) - 1));

		SmartDashboard.putNumber("Top Shooter RPS", getTopShooterSpeed());
		SmartDashboard.putNumber("Top Shooter Target RPS", topTargetSpeed);
		SmartDashboard.putNumber("Top Shooter Target Diff", topTargetSpeed - getTopShooterSpeed());
		SmartDashboard.putNumber("Top Shooter Target Percentage", Math.abs(getTopShooterSpeed() / topTargetSpeed - 1));

		SmartDashboard.putNumber("Top Shooter RPS", getHoodShooterSpeed());
		SmartDashboard.putNumber("Top Shooter Target RPS", hoodTargetSpeed);
		SmartDashboard.putNumber("Top Shooter Target Diff", hoodTargetSpeed - getHoodShooterSpeed());
		SmartDashboard.putNumber("Top Shooter Target Percentage", Math.abs(getHoodShooterSpeed() / hoodTargetSpeed - 1));
        
		SmartDashboard.putString("Shooter State", state.toString());

		SmartDashboard.putBoolean("Shooter At Speed", isShooterAtSpeed());
  	}
      
    /**
     * Returns the speed of the bottom shooter motor
     * 
     * @return Speed in RPS
     */
    private double getBottomShooterSpeed() {
        return bottomShooter.getRotorVelocity().getValueAsDouble();
    }

	/**
	 * Returns the speed of the top shooter motor
	 * 
	 * @return Speed in RPS
	 */
	private double getTopShooterSpeed() {
		return topShooter.getRotorVelocity().getValueAsDouble();
	}

    /**
	 * Returns the speed of the top shooter motor
	 * 
	 * @return Speed in RPS
	 */
	private double getHoodShooterSpeed() {
		return hoodShooter.getRotorVelocity().getValueAsDouble();
	}

	/**
	 * Sets the state of the shooter.
	 * <p> 
	 * If wanting to control the shooter without PID
	 * then use setShooterDumbControl()
	 * 
	 * @param stateToChangeTo Using ShooterConstants.State
	 * @param SpeedInRPS The target speed to set in rps.
	 * 					 When setting the state to idle,
	 * 					 target speed is not used and can
	 * 					 be set to 0.	
	 * @param hoodSpeedInRPS The target speed to set in rps.
	 * 					     When setting the state to idle,
	 * 					     target speed is not used and can
	 * 					     be set to 0.
	 */
	public void setShooterState(State stateToChangeTo, double SpeedInRPS, double hoodSpeedInRPS) {
        topTargetSpeed = SpeedInRPS;
		hoodTargetSpeed = hoodSpeedInRPS;
		state = stateToChangeTo;
	}

	/**
	 * Sets the state of the shooter to DumbControl
	 * <p>
	 * Used if want to control the shooter open loop without
	 * the PID. Uses the TalonFX .set() function 
	 * 
	 * @param topSpeedInPercent The speed to control in percent
	 * @param hoodSpeedInPercent The speed to control in percent
	 */
	public void setShooterDumbControl(double topSpeedInPercent, double hoodSpeedInPercent) {
		bottomTargetSpeed = topSpeedInPercent;
        topTargetSpeed = topSpeedInPercent;
		hoodTargetSpeed = hoodSpeedInPercent;
		state = State.DumbControl;
	}

	/**
	 * Sets the state of the shooter to DumbControl
	 * <p>
	 * Used if want to control the shooter open loop without
	 * the PID. Uses the TalonFX .set() function 
	 * 
	 * @param topSpeedInPercent The speed to control in percent
	 * @param hoodSpeedInPercent The speed to control in percent
	 * @param bottomSpeedInPercent The speed to control in percent
	 */
	public void setShooterDumbControl(double topSpeedInPercent, double hoodSpeedInPercent, double bottomSpeedInPercent) {
        bottomTargetSpeed = bottomSpeedInPercent;
		topTargetSpeed = topSpeedInPercent;
		hoodTargetSpeed = hoodSpeedInPercent;
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
		if (state == State.Shoot) {
			return Math.abs(getTopShooterSpeed() - topTargetSpeed) < RPSThreshold;
		} else {
			return false;
		}
	}
}
