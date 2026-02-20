// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.Constants.IntakeConstants.HopperIn;
import static frc.robot.constants.Constants.IntakeConstants.HopperOut;
import static frc.robot.constants.Constants.IntakeConstants.IntakeCurrentLimit;
import static frc.robot.constants.Constants.IntakeConstants.IntakeID;
import static frc.robot.constants.Constants.IntakeConstants.IntakingSpeed;
import static frc.robot.constants.Constants.IntakeConstants.PivotID;
import static frc.robot.constants.Constants.IntakeConstants.PivotInPos;
import static frc.robot.constants.Constants.IntakeConstants.PivotOutPos;
import static frc.robot.constants.Constants.IntakeConstants.PivotPIDkD;
import static frc.robot.constants.Constants.IntakeConstants.PivotPIDkG;
import static frc.robot.constants.Constants.IntakeConstants.PivotPIDkI;
import static frc.robot.constants.Constants.IntakeConstants.PivotPIDkP;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.IntakeConstants.State;
import frc.robot.constants.Constants.PneumaticConstants;

public class Intake extends SubsystemBase {

	// Neo Vortex
  	private SparkFlex intake = new SparkFlex(IntakeID, MotorType.kBrushless);
  	private SparkFlexConfig intakeConfig = new SparkFlexConfig();

	// Kraken X44
	private TalonFX pivot = new TalonFX(PivotID);
	private TalonFXConfiguration pivotConfig = new TalonFXConfiguration(); 

    private PneumaticHub pneumaticHub = new PneumaticHub(PneumaticConstants.PneumaticsHubID);
	private DoubleSolenoid hopperSolenoid = pneumaticHub.makeDoubleSolenoid(HopperIn, HopperOut);

	private PositionVoltage pivotPositionVoltage = new PositionVoltage(0);
	private PIDController pivotController = new PIDController(0.3, 0, 0);

	private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);
	
  	private double intakeTargetSpeed = 0;
	private double pivotTargetSpeed = 0;
    private double pivotSetpoint = 0;
	private double pivotPIDOutput = 0;
	private boolean pivotExtension = false;
	private boolean hopperExtension = false;

	private Timer unstickPivotTimer = new Timer();

	public State state = State.Idle;

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
		
		unstickPivotTimer.stop();
		unstickPivotTimer.reset();

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
			case Idle:
				intake.set(0.00);
				//pivot.setControl(pivotPositionVoltage.withPosition(PivotInPos));
				pivot.set(pivotPIDOutput);
				pivotSetpoint = PivotInPos;
				hopperSolenoid.set(DoubleSolenoid.Value.kForward);
				unstickPivotTimer.stop();
				break;
            case IdleOut:
                intake.set(0.00);
				//pivot.setControl(pivotPositionVoltage.withPosition(PivotInPos));
				pivot.set(pivotPIDOutput);
				pivotSetpoint = PivotOutPos;
				hopperSolenoid.set(DoubleSolenoid.Value.kReverse);
				unstickPivotTimer.stop();
                break;
			case Intake:
				intake.set(IntakingSpeed);
				//pivot.setControl(pivotPositionVoltage.withPosition(PivotOutPos));
				pivot.set(pivotPIDOutput);
				pivotSetpoint = PivotOutPos;
				hopperSolenoid.set(DoubleSolenoid.Value.kReverse);
				unstickPivotTimer.stop();
				break;
			case Unstick:
				intake.set(IntakingSpeed);
				pivotSetpoint = PivotOutPos;
				hopperSolenoid.set(DoubleSolenoid.Value.kReverse);
				//unstickPivotTimer.start();
				break;
			case Outtake:
				intake.set(-IntakingSpeed);
				//pivot.setControl(pivotPositionVoltage.withPosition(PivotOutPos));
				pivot.set(pivotPIDOutput);
				pivotSetpoint = PivotOutPos;
				hopperSolenoid.set(DoubleSolenoid.Value.kReverse);
				unstickPivotTimer.stop();
				break;
			case DumbControl:
				intake.set(intakeTargetSpeed);
				pivot.set(pivotTargetSpeed);
				if (hopperExtension) {
					hopperSolenoid.set(DoubleSolenoid.Value.kReverse);
				} else {
					hopperSolenoid.set(DoubleSolenoid.Value.kForward);
				}

				unstickPivotTimer.stop();
				break;
		}

		// Cycle between pivot in and out to agitate
		// balls in hopper
		if (unstickPivotTimer.isRunning() && unstickPivotTimer.hasElapsed(1)) {
			unstickPivotTimer.reset();
			if (pivotExtension) {
				pivot.setControl(pivotPositionVoltage.withPosition(PivotInPos));
				pivotExtension = false;
			} else {
				pivot.setControl(pivotPositionVoltage.withPosition(PivotOutPos));
				pivotExtension = true;
			}
		}


		SmartDashboard.putNumber("Intake Pivot Position", pivot.getPosition().getValueAsDouble());
		SmartDashboard.putNumber("Intake Pivot Unstick Timer", unstickPivotTimer.get());
		
		// temp
		SmartDashboard.putNumber("Intake Pivot PID Output", pivotPIDOutput);
		SmartDashboard.putNumber("Intake Pivot Position 2 Electric Boogaloo", pivotEncoder.get());

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


