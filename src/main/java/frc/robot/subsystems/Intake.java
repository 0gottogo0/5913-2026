// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.constants.Constants.IntakeConstants.IntakeCurrentLimit;
import static frc.robot.constants.Constants.IntakeConstants.IntakeLeftID;
import static frc.robot.constants.Constants.IntakeConstants.IntakePIDkD;
import static frc.robot.constants.Constants.IntakeConstants.IntakePIDkI;
import static frc.robot.constants.Constants.IntakeConstants.IntakePIDkP;
import static frc.robot.constants.Constants.IntakeConstants.IntakePIDkV;
import static frc.robot.constants.Constants.IntakeConstants.IntakeRightID;
import static frc.robot.constants.Constants.IntakeConstants.IntakingSpeed;
import static frc.robot.constants.Constants.IntakeConstants.PivotAgitateSpeed;
import static frc.robot.constants.Constants.IntakeConstants.PivotCurrentLimit;
import static frc.robot.constants.Constants.IntakeConstants.PivotExtensionSpeed;
import static frc.robot.constants.Constants.IntakeConstants.PivotID;
import static frc.robot.constants.Constants.IntakeConstants.PivotInPos;
import static frc.robot.constants.Constants.IntakeConstants.PivotInTriggerPos;
import static frc.robot.constants.Constants.IntakeConstants.PivotOutPos;
import static frc.robot.constants.Constants.IntakeConstants.PivotPIDkD;
import static frc.robot.constants.Constants.IntakeConstants.PivotPIDkI;
import static frc.robot.constants.Constants.IntakeConstants.PivotPIDkP;
import static frc.robot.constants.Constants.IntakeConstants.PivotRetractSpeed;
import static frc.robot.constants.Constants.IntakeConstants.PivotRunIntakeTriggerPos;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
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

	private PIDController pivotController = new PIDController(PivotPIDkP, PivotPIDkI, PivotPIDkD);

	private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);
	
	private double intakeTargetSpeed = 0;
	private double pivotTargetSpeed = 0;
    private double pivotSetpoint = 0;
	private double pivotPIDOutput = 0;

	public State state = State.IdleIn;

  	public Intake() {
		intakeLeftConfig.idleMode(IdleMode.kCoast);
        intakeLeftConfig.inverted(false);
		intakeLeftConfig.smartCurrentLimit(IntakeCurrentLimit);
		intakeLeftConfig.closedLoop
    		.p(IntakePIDkP)
    		.i(IntakePIDkI)
    		.d(IntakePIDkD)
			.feedForward.kV(IntakePIDkV);

        intakeLeft.configure(intakeLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		intakeRightConfig.idleMode(IdleMode.kCoast);
        intakeRightConfig.inverted(false);
		intakeRightConfig.smartCurrentLimit(IntakeCurrentLimit);
		intakeRightConfig.closedLoop
    		.p(IntakePIDkP)
    		.i(IntakePIDkI)
    		.d(IntakePIDkD)
			.feedForward.kV(IntakePIDkV);

        intakeRight.configure(intakeRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		pivotConfig.withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(PivotCurrentLimit))
            .withStatorCurrentLimitEnable(true));

		pivot.clearStickyFaults();
		pivot.getConfigurator().apply(pivotConfig);
		
		pivotSetpoint = getPivotEncoder();

		new Thread(() -> {
    		try {
    		    Thread.sleep(3000);
    		    pivotSetpoint = getPivotEncoder();
    		} catch (Exception e) {
    		}
    	}).start();
  	}

  	@Override
  	public void periodic() {
		pivotPIDOutput = MathUtil.clamp(pivotController.calculate(getPivotEncoder(), pivotSetpoint), -PivotExtensionSpeed, PivotRetractSpeed);

		switch (state) {
			case IdleIn:
				intakeLeft.set(0.00);
				intakeRight.set(0.00);
				pivot.set(pivotPIDOutput);
				pivotSetpoint = PivotInPos;
				break;
            case IdleOut:
                intakeLeft.set(0.00);
				intakeRight.set(0.00);
				pivot.set(pivotPIDOutput);
				pivotSetpoint = PivotOutPos;
                break;
			case IntakeIn:
				intakeLeft.getClosedLoopController().setSetpoint(IntakingSpeed, ControlType.kVelocity);
				intakeRight.getClosedLoopController().setSetpoint(-IntakingSpeed, ControlType.kVelocity);
				pivot.set(pivotPIDOutput);
				pivotSetpoint = PivotInPos;
				break;
			case IntakeOut:
				intakeLeft.getClosedLoopController().setSetpoint(IntakingSpeed, ControlType.kVelocity);
				intakeRight.getClosedLoopController().setSetpoint(-IntakingSpeed, ControlType.kVelocity);
				pivot.set(pivotPIDOutput);
				pivotSetpoint = PivotOutPos;
				break;
			case Outtake:
				intakeLeft.getClosedLoopController().setSetpoint(-IntakingSpeed, ControlType.kVelocity);
				intakeRight.getClosedLoopController().setSetpoint(IntakingSpeed, ControlType.kVelocity);
				pivot.set(pivotPIDOutput);
				pivotSetpoint = PivotOutPos;
				break;
			case Agitate:
				if (getPivotEncoder() >= PivotRunIntakeTriggerPos) {
					intakeLeft.set(0.00);
					intakeRight.set(0.00);
				} else {
					intakeLeft.getClosedLoopController().setSetpoint(IntakingSpeed, ControlType.kVelocity);
					intakeRight.getClosedLoopController().setSetpoint(-IntakingSpeed, ControlType.kVelocity);
				}
				if (getPivotEncoder() >= PivotInTriggerPos) {
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

		SmartDashboard.putNumber("Intake Pivot Position", getPivotEncoder());
		SmartDashboard.putNumber("Intake Pivot PID Output", pivotPIDOutput);
		SmartDashboard.putNumber("Intake Left Speed", intakeLeft.getEncoder().getVelocity());
		SmartDashboard.putNumber("Intake Right Speed", intakeRight.getEncoder().getVelocity());
		SmartDashboard.putNumber("Intake Left Current Draw", intakeLeft.getOutputCurrent());
		SmartDashboard.putNumber("Intake Right Current Draw", intakeRight.getOutputCurrent());

        SmartDashboard.putString("Intake State", state.toString());
  	}

	public double getPivotEncoder() {

		// Should unhard code this value
		// We get a coterminal angle because encoder wraps around after 1.00
		// and we only realy need 0.6 - 1.1 or sumthin
		if (pivotEncoder.get() < 0.4) {
			return pivotEncoder.get() + 1.00;
		} else {
			return pivotEncoder.get();
		}
	}

	/**
	 * Gets the position of the intake.
	
	 * @return true if intake is in
	 */
	public Boolean getIntakePosition() {
		if (getPivotEncoder() > PivotInTriggerPos) {
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


