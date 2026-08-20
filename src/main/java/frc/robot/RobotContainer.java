package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import yams.mechanisms.swerve.utility.SwerveInputStream;

public class RobotContainer
{

  final CommandPS4Controller ps4Controller = new CommandPS4Controller(0);

  private final SwerveDriveSubsystem swerve = new SwerveDriveSubsystem();

  
  private final SwerveInputStream driveAngularVelocity =
      swerve.getAngularVelocityStream(
                     () -> -ps4Controller.getLeftY(),
                     () -> ps4Controller.getLeftX(),
                     () -> -ps4Controller.getRightX())
                    .withAllianceRelativeControl();

  public RobotContainer()
  {
    configureBindings();
  }

  private void configureBindings()
  {
    // Default drive command
    swerve.setDefaultCommand(swerve.driveRobotRelative(driveAngularVelocity));

    // Zero the gyro with Start + Back — use this if the field-relative heading drifts

    //driverXbox.a().onTrue(swerve.driveToPose(new Pose2d(0, 2, Rotation2d.fromDegrees(0))));

    //driverXbox.b().onTrue(swerve.driveToPose(new Pose2d(0, 0, Rotation2d.fromDegrees(90))));

  }

}