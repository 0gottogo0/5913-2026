// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.FeederConstants;
import frc.robot.constants.Constants.FeederConstants.State;

public class Feeder extends SubsystemBase {

    private TalonFX feeder = new TalonFX(FeederConstants.MotorID); // create constants and change motor names
    private TalonFXConfiguration feederConfig = new TalonFXConfiguration();

	private VelocityVoltage feederVelocityVoltage = new VelocityVoltage(0);

    private double targetSpeed = 0;

    public State state = State.Idle;

    public Feeder() {
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  	  	feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		feederConfig.Slot0.kV = FeederConstants.PIDkV;
		feederConfig.Slot0.kP = FeederConstants.PIDkP;
		feederConfig.Slot0.kI = FeederConstants.PIDkI;
		feederConfig.Slot0.kD = FeederConstants.PIDkD;


  	  	feeder.clearStickyFaults();
  	  	feeder.getConfigurator().apply(feederConfig);
    }

    @Override
    public void periodic() {

        if (state == State.Idle) {
            feeder.set(0.00);
        } else if (state == State.Feed) {
            feeder.setControl(feederVelocityVoltage.withVelocity(FeederConstants.FeedingSpeed));
        } else if (state == State.Unstick) {
            feeder.setControl(feederVelocityVoltage.withVelocity(FeederConstants.UnstickSpeed));
        } else if (state == State.Outtake) {
            feeder.setControl(feederVelocityVoltage.withVelocity(FeederConstants.OuttakeSpeed));
        } else {
            feeder.set(targetSpeed);
        }

        SmartDashboard.putNumber("Feeder RPS", feeder.getVelocity().getValueAsDouble());

        SmartDashboard.putString("Feeder State", state.toString());
    }

    public void SetFeederState(State stateToChangeTo) {
        state = stateToChangeTo;

    }

    public void SetFeederDumbControl(double speedInPercent) {
        targetSpeed = speedInPercent;
        state = State.DumbControl;
    }
}
