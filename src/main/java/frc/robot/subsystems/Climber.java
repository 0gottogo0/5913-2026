// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.Constants.ClimberConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private TalonFX hookPivot = new TalonFX(MotorID);
    private TalonFXConfiguration hootPivotConfig = new TalonFXConfiguration();

	private PositionVoltage hootPivotPositionVoltage = new PositionVoltage(0);

    private double targetSpeed = 0;

    public State state = State.Idle;

    public Climber() {
        hootPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  	  	hootPivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		hootPivotConfig.Slot0.kG = PIDkG;
		hootPivotConfig.Slot0.kP = PIDkP;
		hootPivotConfig.Slot0.kI = PIDkI;
		hootPivotConfig.Slot0.kD = PIDkD;

  	  	hookPivot.clearStickyFaults();
  	  	hookPivot.getConfigurator().apply(hootPivotConfig);
    }

    @Override
    public void periodic() {
        switch (state) {
			case Idle:
            	hookPivot.set(0);
				break;
			case ClimbUp:
				hookPivot.setControl(hootPivotPositionVoltage.withPosition(ClimbUpSetpoint * PivotMotorRotationsToOneDegree));
				break;
			case ClimbDown:
				hookPivot.setControl(hootPivotPositionVoltage.withPosition(ClimbDownSetpoint * PivotMotorRotationsToOneDegree));
				break;
			case DumbControl:
				hookPivot.set(targetSpeed);
				break;
		}

		SmartDashboard.putNumber("Pivot Pos", hookPivot.getPosition().getValueAsDouble());

		SmartDashboard.putString("Climber State", state.toString());
    }

    /**
	 * Sets the state of the climber.
	 * <p> 
	 * If wanting to control the climber without PID
	 * then use setClimberDumbControl()
	 * 
	 * @param stateToChangeTo Using climberConstants.State
	 */
	public void setClimberState(State stateToChangeTo) {
		state = stateToChangeTo;
	}

    /**
	 * Sets the state of the climber to DumbControl
	 * <p>
	 * Used if want to control the climber open loop without
	 * the PID. Uses the TalonFX .set() function 
	 * 
	 * @param speedInPercent The speed to control the climber
	 * 						 motor in percent
	 */
    public void setClimberDumbControl(double speedInPercent) {
        targetSpeed = speedInPercent;
        state = State.DumbControl;
    }
}
