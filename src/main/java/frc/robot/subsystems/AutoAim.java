// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoAim extends SubsystemBase {

    Pose2d robotPos = new Pose2d();
    ChassisSpeeds robotSpeed = new ChassisSpeeds();

    Pose2d leftTurretPose = new Pose2d();
    Pose2d rightTurretPose = new Pose2d();

    public AutoAim() {

    }

    @Override
    public void periodic() {
        leftTurretPose = robotPos.plus(frc.robot.constants.Constants.AutoAim.LeftTurretPos);
        rightTurretPose = robotPos.plus(frc.robot.constants.Constants.AutoAim.RightTurretPos);

        SmartDashboard.putNumber("Robot X", robotPos.getX());
        SmartDashboard.putNumber("Robot Y", robotPos.getY());
        SmartDashboard.putNumber("Left Turret X", leftTurretPose.getX());
        SmartDashboard.putNumber("Left Turret Y", leftTurretPose.getY());
        SmartDashboard.putNumber("Right Turret X", rightTurretPose.getX());
        SmartDashboard.putNumber("Right Turret Y", rightTurretPose.getY());
    }

    /**
     * Give the auto aim subsystem the robots state
     * 
     * @param drivetrainPos The robots position
     * @param drivetrainSpeed The robots speed
     */
    public void setAutoAimDrivetrainState(Pose2d drivetrainPos, ChassisSpeeds drivetrainSpeed) {
        robotPos = drivetrainPos;
        robotSpeed = drivetrainSpeed;
    }

    /**
     * Get angle from robot to target
     * 
     * @return Aim tagret in degrees
     */
    public double getAimTargetInDegrees() {
        return 0.00;
    }
    
    /**
     * Get distance from robot to target
     * 
     * @return Aim target in meters? prob should check what the units are lol
     */
    public double getAimTargetInDistace() {
        return 0.00;
    }
}
