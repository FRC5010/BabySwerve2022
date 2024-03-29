// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.FRC5010.GenericGyro;
import frc.robot.FRC5010.Impl.NavXGyro;
import frc.robot.mechanisms.Drive;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Drive drive;

  GenericGyro gyro;

  SwerveSubsystem swerveSubsystem;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    gyro = new NavXGyro(Port.kMXP);
    drive = new Drive(gyro);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    .setKinematics(DriveConstants.kinematics);

    Trajectory trajectory = PathPlanner.loadPath("New Path",AutoConstants.kMaxSpeedMetersPerSecond,AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);

    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kThetaController, 0, 0, AutoConstants.kThetaControllerConstrants);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    swerveSubsystem = drive.getSwerveSubsystem();

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory,
      swerveSubsystem::getPose2d, 
      DriveConstants.kinematics, 
      xController, 
      yController, 
      thetaController, 
      swerveSubsystem::setModuleStates, 
      swerveSubsystem
    );
    
    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> swerveSubsystem.stopModules())

    );
  }
}
