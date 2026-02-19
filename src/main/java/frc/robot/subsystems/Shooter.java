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
import frc.robot.constants.Constants.ShooterConstants.State;

public class Shooter extends SubsystemBase {

    // Kraken X44
    private TalonFX belts = new TalonFX(BeltsID);
    private TalonFXConfiguration beltsConfig = new TalonFXConfiguration();

	// Kraken X44
	private TalonFX feeder = new TalonFX(FeederMotorID);
	private TalonFXConfiguration feederConfig = new TalonFXConfiguration();

	// Kraken X60
	private TalonFX bottomShooter = new TalonFX(BottomMotorID);
    private TalonFXConfiguration bottomShooterConfig = new TalonFXConfiguration();

	// Kraken X60
  	private TalonFX topShooter = new TalonFX(TopMotorID);
  	private TalonFXConfiguration topShooterConfig = new TalonFXConfiguration();

	// Kraken X44
	private TalonFX hoodShooter = new TalonFX(HoodMotorID);
  	private TalonFXConfiguration hoodShooterConfig = new TalonFXConfiguration();

	private VelocityVoltage beltsVelocityVoltage = new VelocityVoltage(0);
	private VelocityVoltage feederVelocityVoltage = new VelocityVoltage(0);
    private VelocityVoltage bottomShooterVelocityVoltage = new VelocityVoltage(0);
	private VelocityVoltage topShooterVelocityVoltage = new VelocityVoltage(0);
    private VelocityVoltage hoodShooterVelocityVoltage = new VelocityVoltage(0);

	private double beltsTargetSpeed = 0.00;
	private double feederTargetSpeed = 0.00;
	private double bottomTargetSpeed = 0.00;
  	private double topTargetSpeed = 0.00;
	private double hoodTargetSpeed = 0.00;

	public State state = State.Idle;

  	public Shooter() {
		beltsConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	beltsConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		beltsConfig.Slot0.kV = BeltsPIDkV;
		beltsConfig.Slot0.kP = BeltsPIDkP;
		beltsConfig.Slot0.kI = BeltsPIDkI;
		beltsConfig.Slot0.kD = BeltsPIDkD;

  	  	belts.clearStickyFaults();
  	  	belts.getConfigurator().apply(beltsConfig);

		feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		feederConfig.Slot0.kV = FeederPIDkV;
		feederConfig.Slot0.kP = FeederPIDkP;
		feederConfig.Slot0.kI = FeederPIDkI;
		feederConfig.Slot0.kD = FeederPIDkD;

        feeder.clearStickyFaults();
  	  	feeder.getConfigurator().apply(feederConfig);

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
  	  	switch (state) {
			case Idle:
                belts.set(0.00);
				feeder.set(0.00);
				bottomShooter.set(0.00);
				topShooter.set(0.00);
				hoodShooter.set(0.00);
				break;
			case Spinup:
                belts.set(0.00);
				feeder.set(0.00);
				bottomShooter.setControl(bottomShooterVelocityVoltage.withVelocity(topTargetSpeed * BottomRatio));
				topShooter.setControl(topShooterVelocityVoltage.withVelocity(topTargetSpeed));
				hoodShooter.setControl(hoodShooterVelocityVoltage.withVelocity(hoodTargetSpeed));
				break;
			case Shoot:
                belts.setControl(beltsVelocityVoltage.withVelocity(BeltsSpeed));
				feeder.setControl(feederVelocityVoltage.withVelocity(FeedSpeed));
				bottomShooter.setControl(bottomShooterVelocityVoltage.withVelocity(topTargetSpeed * BottomRatio));
				topShooter.setControl(topShooterVelocityVoltage.withVelocity(topTargetSpeed));
				hoodShooter.setControl(hoodShooterVelocityVoltage.withVelocity(hoodTargetSpeed));
				break;
			case Unstick:
                belts.setControl(beltsVelocityVoltage.withVelocity(BeltsSpeed));
				feeder.setControl(feederVelocityVoltage.withVelocity(FeedSpeed));
				bottomShooter.setControl(bottomShooterVelocityVoltage.withVelocity(UnstickRPS));
				topShooter.setControl(topShooterVelocityVoltage.withVelocity(UnstickRPS));
				hoodShooter.setControl(hoodShooterVelocityVoltage.withVelocity(UnstickRPS));
				break;
			case DumbControl:
                belts.set(beltsTargetSpeed);
				feeder.set(feederTargetSpeed);
				bottomShooter.set(bottomTargetSpeed);
				topShooter.set(topTargetSpeed);
				hoodShooter.set(hoodTargetSpeed);
				break;
		}

        SmartDashboard.putNumber("Belts RPS", getBeltsSpeed());
		SmartDashboard.putNumber("Belts Target RPS", beltsTargetSpeed);
		SmartDashboard.putNumber("Belts Target Diff", beltsTargetSpeed - getBeltsSpeed());
		SmartDashboard.putNumber("Belts Target Percentage", Math.abs(getBeltsSpeed() / beltsTargetSpeed - 1));

        SmartDashboard.putNumber("Feeder Shooter RPS", getFeederShooterSpeed());
		SmartDashboard.putNumber("Feeder Shooter Target RPS", feederTargetSpeed);
		SmartDashboard.putNumber("Feeder Shooter Target Diff", feederTargetSpeed - getFeederShooterSpeed());
		SmartDashboard.putNumber("Feeder Shooter Target Percentage", Math.abs(getFeederShooterSpeed() / feederTargetSpeed - 1));

        SmartDashboard.putNumber("Bottom Shooter RPS", getBottomShooterSpeed());
		SmartDashboard.putNumber("Bottom Shooter Target RPS", bottomTargetSpeed * BottomRatio);
		SmartDashboard.putNumber("Bottom Shooter Target Diff", (bottomTargetSpeed * BottomRatio) - getBottomShooterSpeed());
		SmartDashboard.putNumber("Bottom Shooter Target Percentage", Math.abs(getBottomShooterSpeed() / (bottomTargetSpeed * BottomRatio) - 1));

		SmartDashboard.putNumber("Top Shooter RPS", getTopShooterSpeed());
		SmartDashboard.putNumber("Top Shooter Target RPS", topTargetSpeed);
		SmartDashboard.putNumber("Top Shooter Target Diff", topTargetSpeed - getTopShooterSpeed());
		SmartDashboard.putNumber("Top Shooter Target Percentage", Math.abs(getTopShooterSpeed() / topTargetSpeed - 1));

		SmartDashboard.putNumber("Hood Shooter RPS", getHoodShooterSpeed());
		SmartDashboard.putNumber("Hood Shooter Target RPS", hoodTargetSpeed);
		SmartDashboard.putNumber("Hood Shooter Target Diff", hoodTargetSpeed - getHoodShooterSpeed());
		SmartDashboard.putNumber("Hood Shooter Target Percentage", Math.abs(getHoodShooterSpeed() / hoodTargetSpeed - 1));
        
		SmartDashboard.putString("Shooter State", state.toString());

		SmartDashboard.putBoolean("Shooter At Speed", isShooterAtSpeed());
  	}
    
	/**
	 * Returns the speed of the belts motor
	 * 
	 * @return Speed in RPS
	 */
    private double getBeltsSpeed() {
        return belts.getRotorVelocity().getValueAsDouble();
    }

	/**
     * Returns the speed of the feeder shooter motor
     * 
     * @return Speed in RPS
     */
    private double getFeederShooterSpeed() {
        return feeder.getRotorVelocity().getValueAsDouble();
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
	 * 
	 * @param hoodSpeedInRPS The target speed to set in rps.
	 * 					     When setting the state to idle,
	 * 					     target speed is not used and can
	 * 					     be set to 0.
	 */
	public void setShooterState(State stateToChangeTo, double speedInRPS, double hoodSpeedInRPS) {
        topTargetSpeed = speedInRPS;
		hoodTargetSpeed = hoodSpeedInRPS;
		state = stateToChangeTo;
	}

	/**
	 * Sets the state of the shooter to DumbControl
	 * <p>
	 * Used if want to control the shooter open loop without
	 * the PID. Uses the TalonFX .set() function 
	 * 
	 * @param feedSpeedInPercent The speed to control in percent
	 * 
	 * @param topSpeedInPercent The speed to control in percent
	 * 
	 * @param hoodSpeedInPercent The speed to control in percent
	 */
	public void setShooterDumbControl(double feedSpeedInPercent, double topSpeedInPercent, double hoodSpeedInPercent) {
		beltsTargetSpeed = feedSpeedInPercent;
        feederTargetSpeed = feedSpeedInPercent;
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
	 * @param beltsSpeedInPercent The speed to control in percent
     * 
	 * @param feedSpeedInPercent The speed to control in percent
	 * 
	 * @param bottomSpeedInPercent The speed to control in percent
	 * 
	 * @param topSpeedInPercent The speed to control in percent
	 * 
	 * @param hoodSpeedInPercent The speed to control in percent
	 */
	public void setShooterDumbControl(double beltsSpeedInPercent, double feedSpeedInPercent, double bottomSpeedInPercent, double topSpeedInPercent, double hoodSpeedInPercent) {
        beltsTargetSpeed = beltsSpeedInPercent;
        feederTargetSpeed = feedSpeedInPercent;
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
		if (state == State.Spinup || state == State.Shoot) {
			return Math.abs(getTopShooterSpeed() - topTargetSpeed) < RPSThreshold && Math.abs(getBottomShooterSpeed() - topTargetSpeed) < RPSThreshold;
		} else {
			return false;
		}
	}
}
