package frc.robot.subsystems.swervedrive;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DrivebaseConstants;

import java.io.File;
import java.util.function.DoubleSupplier;
import swervelib.parser.SwerveParser;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

import static edu.wpi.first.units.Units.*;

public class SwerveDriveSubsystem extends SubsystemBase
{

  private SwerveDrive drive;
  private static final String[] ModuleNames = {
        "Front Left",
        "Front Right",
        "Back Left",
        "Back Right"
    };
  

  public SwerveDriveSubsystem()
  {
    var cfg = new SwerveDriveConfig()
        .withStartingPose(new Pose2d(0, 0, Rotation2d.kZero))
        .withSubsystem(this)
        .withTelemetry(TelemetryVerbosity.HIGH)
        .withTranslationController(new PIDController(1.0, 0, 0)) // input: meters of position error
        .withRotationController(new PIDController(1.0, 0, 0));   // input: radians of heading error
    try
    {
      drive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/base"))
          .createSwerveDrive(cfg);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
  }

  public SwerveInputStream getAngularVelocityStream(DoubleSupplier x, DoubleSupplier y,
                                                    DoubleSupplier rot)
  {
    return new SwerveInputStream(drive, x, y, rot);
  }

  public Command drive(SwerveInputStream stream)
  {
    return drive.drive(() -> ChassisSpeeds.fromFieldRelativeSpeeds(stream.get(),
                                                                   new Rotation2d(drive.getGyroAngle())));
  }

  /** Zero the gyro heading. Bind this to a button combo for field recovery. */
  public Command zeroGyro()
  {
    return runOnce(() -> drive.zeroGyro());
  }

  public Command driveToPose(Pose2d pose) {
  return drive.driveToPose(pose)
      .until(() -> drive.getDistanceFromPose(pose).in(Meters) < 0.1
                && Math.abs(drive.getAngleDifferenceFromPose(pose).in(Degrees)) < 2);
}

  // Andar 2 metros para frente
    public Command driveForward2m() {
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = new Pose2d(
        currentPose.getX() + 2.0,
        currentPose.getY(),
        currentPose.getRotation()
    );
    return driveToPose(targetPose);
}
  // Girar 90 graus
    public Command rotate90() {
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = new Pose2d(
        currentPose.getTranslation(),
        currentPose.getRotation().plus(
            Rotation2d.fromDegrees(90)
        )
    );
    return driveToPose(targetPose);
}

  public void resetOdometry(Pose2d pose) {
    drive.resetOdometry(pose);
  }
  private Field2d field = new Field2d();
   private void updateDashboardField() {
    Pose2d robotPose = drive.getPose();

    field.setRobotPose(robotPose);
    SmartDashboard.putData("Fieldlegal", field);


  }

  @Override
    public void periodic() {
    updateDashboardField();
    drive.updateTelemetry();
    
    Pose2d pose = drive.getPose();

    SmartDashboard.putNumber(
        "Pose Rotation",
        pose.getRotation().getDegrees()
    );

    SmartDashboard.putNumber(
        "Pose X",
        pose.getX()
    );

    SmartDashboard.putNumber(
        "Pose Y",
        pose.getY()
    );

    }
} 
  /**)
  }
}

  /**
   * Create a {@link Command} that runs a full SysId characterization routine (quasistatic and dynamic, forward and
   * reverse) on a single swerve module's drive motor. The module's azimuth is held pointed straight ahead for the
   * duration of the test so only the drive motor is characterized.
   *
   * @param moduleName Name of the module to test, e.g. "frontleft", "frontright", "backleft", or "backright".
   * @return {@link Command} that runs the full SysId routine on the given module.
   */
  
   /*public Command sysIdModule(String moduleName)
  {

    SwerveModule         module       = drive.getModule(moduleName).orElseThrow();
    SmartMotorController driveMotor   = module.getDriveMotorController();
    SmartMotorController azimuthMotor = module.getAzimuthMotorController();

    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(7), Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            azimuthMotor::setVoltage,
            log -> log.motor(moduleName + "-azimuth")
                      .voltage(azimuthMotor.getVoltage())
                      .angularPosition(azimuthMotor.getMechanismPosition())
                      .angularVelocity(azimuthMotor.getMechanismVelocity()),
            this,
            moduleName + "-azimuth" 
        )
    );

    return Commands.runOnce(() -> azimuthMotor.setPosition(Rotation2d.kZero.getMeasure()))
                   .andThen(routine.quasistatic(SysIdRoutine.Direction.kForward))
                   .andThen(Commands.waitSeconds(1))
                   .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse))
                   .andThen(Commands.waitSeconds(1))
                   .andThen(routine.dynamic(SysIdRoutine.Direction.kForward))
                   .andThen(Commands.waitSeconds(1))
                   .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse))
                   .withName("SysId " + moduleName + " Azimuth");
  }

}
*/