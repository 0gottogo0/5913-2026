// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.Constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.IntakeConstants.State;

public class Intake extends SubsystemBase {

	// Neo Vortex
  	private SparkFlex intakeLeft = new SparkFlex(IntakeLeftID, MotorType.kBrushless);
  	private SparkFlexConfig intakeLeftConfig = new SparkFlexConfig();

	// Neo Vortex
	private SparkFlex intakeRight = new SparkFlex(IntakeRightID, MotorType.kBrushless);
  	private SparkFlexConfig intakeRightConfig = new SparkFlexConfig(); 

	// Kraken X44
	private TalonFX pivot = new TalonFX(PivotID);
	private TalonFXConfiguration pivotConfig = new TalonFXConfiguration(); 

	private PIDController intakeLeftController = new PIDController(IntakePIDkP, IntakePIDkI, IntakePIDkD);
	private PIDController intakeRightController = new PIDController(IntakePIDkP, IntakePIDkI, IntakePIDkD);
	private PIDController pivotController = new PIDController(PivotPIDkP, PivotPIDkI, PivotPIDkD);

	private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);
	
	private double intakeTargetSpeed = 0;
	private double pivotTargetSpeed = 0;
	private double intakeSetpoint = 0;
    private double pivotSetpoint = 0;
	private double intakeLeftPIDOutput = 0;
	private double intakeRightPIDOutput = 0;
	private double pivotPIDOutput = 0;

	public State state = State.IdleIn;

  	public Intake() {
		intakeLeftConfig.idleMode(IdleMode.kCoast);
        intakeLeftConfig.inverted(false);
		intakeLeftConfig.smartCurrentLimit(IntakeCurrentLimit);

        intakeLeft.configure(intakeLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		intakeRightConfig.idleMode(IdleMode.kCoast);
        intakeRightConfig.inverted(false);
		intakeRightConfig.smartCurrentLimit(IntakeCurrentLimit);

        intakeRight.configure(intakeRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		pivotConfig.withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(PivotCurrentLimit))
            .withStatorCurrentLimitEnable(true));

		pivot.clearStickyFaults();
		pivot.getConfigurator().apply(pivotConfig);
		
		pivotSetpoint = pivotEncoder.get();

		new Thread(() -> {
    		try {
    		    Thread.sleep(3000);
    		    pivotSetpoint = pivotEncoder.get();
    		} catch (Exception e) {
    		}
    	}).start();
  	}

  	@Override
  	public void periodic() {
		pivotPIDOutput = MathUtil.clamp(pivotController.calculate(pivotEncoder.get(), pivotSetpoint), -PivotExtensionSpeed, PivotRetractSpeed);
		intakeLeftPIDOutput = intakeLeftController.calculate(intakeLeft.getEncoder().getVelocity(), intakeSetpoint);
		intakeRightPIDOutput = intakeRightController.calculate(intakeRight.getEncoder().getVelocity(), -intakeSetpoint);

		switch (state) {
			case IdleIn:
				intakeLeft.set(0.00);
				intakeRight.set(0.00);
				pivot.set(pivotPIDOutput);
				intakeSetpoint = 0.00;
				pivotSetpoint = PivotInPos;
				break;
            case IdleOut:
                intakeLeft.set(0.00);
				intakeRight.set(0.00);
				pivot.set(pivotPIDOutput);
				intakeSetpoint = 0.00;
				pivotSetpoint = PivotOutPos;
                break;
			case IntakeIn:
				intakeLeft.set(1);
				intakeRight.set(-1);
				pivot.set(pivotPIDOutput);
				intakeSetpoint = IntakingSpeed;
				pivotSetpoint = PivotInPos;
				break;
			case IntakeOut:
				intakeLeft.set(1);
				intakeRight.set(-1);
				pivot.set(pivotPIDOutput);
				intakeSetpoint = IntakingSpeed;
				pivotSetpoint = PivotOutPos;
				break;
			case Outtake:
				intakeLeft.set(-1);
				intakeRight.set(1);
				pivot.set(pivotPIDOutput);
				intakeSetpoint = IntakingSpeed;
				pivotSetpoint = PivotOutPos;
				break;
			case Agitate:
				if (pivotEncoder.get() >= PivotRunIntakeTriggerPos) {
					intakeLeft.set(0.00);
					intakeRight.set(0.00);
					intakeSetpoint = 0.00;
				} else {
					intakeLeft.set(1);
					intakeRight.set(-1);
					intakeSetpoint = IntakingSpeed;
				}
				if (pivotEncoder.get() >= PivotInTriggerPos) {
					pivot.set(0.00);
				} else {
					pivot.set(PivotAgitateSpeed);
				}
				break;
			case DumbControl:
				intakeLeft.set(intakeTargetSpeed);
				intakeRight.set(intakeTargetSpeed);
				pivot.set(pivotTargetSpeed);
				break;
		}

		SmartDashboard.putNumber("Intake Pivot Position", pivotEncoder.get());
		SmartDashboard.putNumber("Intake Pivot PID Output", pivotPIDOutput);

		SmartDashboard.putNumber("Intake Left Speed", intakeLeft.getEncoder().getVelocity());
		SmartDashboard.putNumber("Intake Right Speed", intakeRight.getEncoder().getVelocity());

        SmartDashboard.putString("Intake State", state.toString());
  	}

	/**
	 * Gets the position of the intake.
	
	 * @return true if intake is in
	 */
	public Boolean getIntakePosition() {
		if (pivotEncoder.get() > PivotInTriggerPos) {
			return true;
		} else {
			return false;
		}
	}

	/**
	 * Sets the state of the intake.
	 * <p> 
	 * If wanting to control the intake without PID
	 * then use setIntakeDumbControl()
	 * 
	 * @param stateToChangeTo Using IntakeConstants.State
	 */
	public void setIntakeState(State stateToChangeTo) {
		state = stateToChangeTo;
	}

	/**
	 * Sets the state of the intake to DumbControl
	 * <p>
	 * Used if want to control the intake open loop without
	 * the PID. Uses the TalonFX .set() function 
	 * 
	 * @param intakeSpeedInPercent The speed to control the intake
	 * 						       motor in percent
     * 
	 * @param pivotSpeedInPercent The speed to control the pivot
	 * 						      motor in percent
     * 
     * @param shouldHopperExtend Should the hopper extend
	 */
	public void setIntakeDumbControl(double intakeSpeedInPercent, double pivotSpeedInPercent) {
		intakeTargetSpeed = intakeSpeedInPercent;
		pivotTargetSpeed = pivotSpeedInPercent;
		state = State.DumbControl;
	}

	/**
	 * Sets the state of the intake and only the intake to DumbControl
	 * <p>
	 * Used if want to control the intake open loop without
	 * the PID. Uses the TalonFX .set() function 
	 * 
	 * @param speedInPercent The speed to control the intake
	 * 						 motor in percent
	 */
	public void setIntakeDumbControl(double speedInPercent) {
		intakeTargetSpeed = speedInPercent;
        pivotTargetSpeed = 0.00;
		state = State.DumbControl;
	}
}


