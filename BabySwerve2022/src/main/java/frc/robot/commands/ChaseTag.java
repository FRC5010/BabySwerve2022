// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FRC5010.DrivetrainPoseEstimator;
import frc.robot.subsystems.SwerveSubsystem;

public class ChaseTag extends CommandBase {


  /** Creates a new ChaseTag. */
  private final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(3, 2); 
  private final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(3, 2); 
  private final TrapezoidProfile.Constraints omegaConstraints = new TrapezoidProfile.Constraints(8, 8); 

  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, xConstraints);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, yConstraints);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, omegaConstraints);

  private int tagToChase; 
  private SwerveSubsystem swerveSubsystem; 
  private PhotonCamera photonCamera; 
  // private Supplies<Pose2d>; 
  private Pose2d pose;
  private PhotonTrackedTarget lastTarget; 
  
  public ChaseTag(SwerveSubsystem swerveSubsystem, PhotonCamera photonCamera, 
  DrivetrainPoseEstimator poseEstimator, PhotonTrackedTarget lastTarget, int tagToChase) {

    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.photonCamera = photonCamera;
    // this.poseEstimator = poseEstimator;
    this.lastTarget = lastTarget;
    this.tagToChase = tagToChase; 

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    
    addRequirements(swerveSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastTarget = null; 
    // pose = poseEstimator.getCurrentPose(); 

    omegaController.reset(pose.getRotation().getRadians());
    xController.reset(pose.getX());
    yController.reset(pose.getY());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
