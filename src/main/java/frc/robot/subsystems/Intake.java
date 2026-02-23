// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.Constants.IntakeConstants.*;
import static frc.robot.constants.Constants.PneumaticConstants.PneumaticsHubID;

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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.IntakeConstants.State;

public class Intake extends SubsystemBase {

	// Neo Vortex
  	private SparkFlex intake = new SparkFlex(IntakeID, MotorType.kBrushless);
  	private SparkFlexConfig intakeConfig = new SparkFlexConfig();

	// Kraken X44
	private TalonFX pivot = new TalonFX(PivotID);
	private TalonFXConfiguration pivotConfig = new TalonFXConfiguration(); 

	private DoubleSolenoid hopperSolenoid = new DoubleSolenoid(PneumaticsHubID, PneumaticsModuleType.CTREPCM, HopperIn, HopperOut);

	private PIDController pivotController = new PIDController(1.50, 0.00, 0.00);

	private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);
	
  	private double intakeTargetSpeed = 0;
	private double pivotTargetSpeed = 0;
    private double pivotSetpoint = 0;
	private double pivotPIDOutput = 0;
	private boolean hopperExtension = false;

	public State state = State.IdleOut;

  	public Intake() {
		intakeConfig.idleMode(IdleMode.kCoast);
        intakeConfig.inverted(false);
		intakeConfig.smartCurrentLimit(IntakeCurrentLimit);

        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		pivotConfig.Slot0.kG = PivotPIDkG;			
		pivotConfig.Slot0.kP = PivotPIDkP;
		pivotConfig.Slot0.kI = PivotPIDkI;
		pivotConfig.Slot0.kD = PivotPIDkD;

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
		pivotPIDOutput = pivotController.calculate(pivotEncoder.get(), pivotSetpoint);

		switch (state) {
			case IdleIn:
				intake.set(0.00);
				pivot.set(pivotPIDOutput);
				pivotSetpoint = PivotInPos;
				// Check if it is safe to retract hopper
				if (pivotEncoder.get() > SafeToRetractHopperPos) {
					hopperSolenoid.set(DoubleSolenoid.Value.kForward);
				}
				break;
            case IdleOut:
                intake.set(0.00);
				pivot.set(pivotPIDOutput);
				pivotSetpoint = PivotOutPos;
				hopperSolenoid.set(DoubleSolenoid.Value.kReverse);
                break;
			case IntakeIn:
				intake.set(IntakingSpeed);
				pivot.set(pivotPIDOutput);
				pivotSetpoint = PivotOutPos;
				// Check if it is safe to retract hopper
				if (pivotEncoder.get() > SafeToRetractHopperPos) {
					hopperSolenoid.set(DoubleSolenoid.Value.kForward);
				}
				break;
			case IntakeOut:
				intake.set(IntakingSpeed);
				pivot.set(pivotPIDOutput);
				pivotSetpoint = PivotOutPos;
				hopperSolenoid.set(DoubleSolenoid.Value.kReverse);
				break;
			case Outtake:
				intake.set(-IntakingSpeed);
				pivot.set(pivotPIDOutput);
				pivotSetpoint = PivotOutPos;
				hopperSolenoid.set(DoubleSolenoid.Value.kReverse);
				break;
			case DumbControl:
				intake.set(intakeTargetSpeed);
				pivot.set(pivotTargetSpeed);
				if (hopperExtension) {
					hopperSolenoid.set(DoubleSolenoid.Value.kReverse);
				} else {
					hopperSolenoid.set(DoubleSolenoid.Value.kForward);
				}
				break;
		}

		SmartDashboard.putNumber("Intake Pivot Position", pivotEncoder.get());
		SmartDashboard.putNumber("Intake Pivot PID Output", pivotPIDOutput);
		SmartDashboard.putBoolean("Hopper Safe To Retract", pivotEncoder.get() > SafeToRetractHopperPos);

        SmartDashboard.putString("Intake State", state.toString());
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
	public void setIntakeDumbControl(double intakeSpeedInPercent, double pivotSpeedInPercent, boolean shouldHopperExtend) {
		intakeTargetSpeed = intakeSpeedInPercent;
		pivotTargetSpeed = pivotSpeedInPercent;
        hopperExtension = shouldHopperExtend;
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


