// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoAim extends SubsystemBase {

    Pose2d robotPose = new Pose2d();
    ChassisSpeeds robotSpeed = new ChassisSpeeds();

    Pose2d TurretRotatePointPose = new Pose2d();

    Pose2d goalPose = new Pose2d();

    public AutoAim() {

    }

    @Override
    public void periodic() {
        try {
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                goalPose = frc.robot.constants.Constants.AutoAim.BlueGoal;
            } else {
                goalPose = frc.robot.constants.Constants.AutoAim.RedGoal;
            }
        } catch (Exception e) {
            goalPose = frc.robot.constants.Constants.AutoAim.RedGoal;
        }

        TurretRotatePointPose = robotPose.plus(frc.robot.constants.Constants.AutoAim.TurretRotatePoint);

        SmartDashboard.putNumber("Robot X", robotPose.getX());
        SmartDashboard.putNumber("Robot Y", robotPose.getY());
        SmartDashboard.putNumber("Robot ROt", robotPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Turret X", TurretRotatePointPose.getX());
        SmartDashboard.putNumber("Turret Y", TurretRotatePointPose.getY());

        SmartDashboard.putNumber("Target Angle", getAimTargetInDegrees());
        SmartDashboard.putNumber("Target Distance", getAimTargetInDistance());
    }

    /**
     * Give the auto aim subsystem the robots state
     * 
     * @param drivetrainPos The robots position
     * @param drivetrainSpeed The robots speed
     */
    public void setAutoAimDrivetrainState(Pose2d drivetrainPos, ChassisSpeeds drivetrainSpeed) {
        robotPose = drivetrainPos;
        robotSpeed = drivetrainSpeed;
    }

    /**
     * Get angle from robot to target
     * 
     * @return Aim tagret in degrees
     */
    public double getAimTargetInDegrees() {
        //return robotPose.getRotation().getDegrees() - Math.tan(TurretRotatePointPose.minus(goalPose).getY() / TurretRotatePointPose.minus(goalPose).getX());
        return 0.00;
    }
    
    /**
     * Get distance from robot to target
     * 
     * @return Aim target in meters? prob should check what the units are lol
     */
    public double getAimTargetInDistance() {
        return Math.abs(Math.sqrt(Math.pow(TurretRotatePointPose.minus(goalPose).getX(), 2) + Math.pow(TurretRotatePointPose.minus(goalPose).getY(), 2)));
    }
}
