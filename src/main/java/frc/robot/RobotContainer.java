// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ControlSub;

public class RobotContainer {
    private final SendableChooser<Command> autoChooser;

    ControlSub control = new ControlSub();

    public RobotContainer() {
        NamedCommands.registerCommand("Run Intake Out", control.run(
            () -> control.startIntakeOut()
        ));

        configureBindings();

        control.drivetrain.configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser("None");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
