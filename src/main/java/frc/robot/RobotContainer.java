// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ControlSub;

public class RobotContainer {
    //private final SendableChooser<Command> autoChooser;

    ControlSub control = new ControlSub();

    public RobotContainer() {
        //NamedCommands.registerCommand("tempAutoCommand", control.run(
        //    () -> control.tempAutoCommand()
        //));

        configureBindings();

        //autoChooser = AutoBuilder.buildAutoChooser();

        //SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {

    }

    public Command getAutonomousCommand() {
        //return autoChooser.getSelected();
        return Commands.print("null");
    }
}
