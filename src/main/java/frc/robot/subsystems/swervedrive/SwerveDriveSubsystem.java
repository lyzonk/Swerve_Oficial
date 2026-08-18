package frc.robot.subsystems.swervedrive;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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

import java.io.File;
import java.util.function.DoubleSupplier;

import swervelib.parser.SwerveParser;
import swervelib.parser.json.ModuleJson;
import swervelib.SwerveInputStream;
import swervelib.SwerveModule;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import static edu.wpi.first.units.Units.*;

public class SwerveDriveSubsystem extends SubsystemBase
{
    
    private SwerveDrive drive;
    double MAX_SPEED = Units.feetToMeters(4);
    File directory = new File(Filesystem.getDeployDirectory(),"swerve/base");
     private static final String[] ModuleNames = {
        "Front Left",
        "Front Right",
        "Back Left",
        "Back Right"
    };
  
    public SwerveDriveSubsystem() {

    try
    {
    drive = new SwerveParser(directory).createSwerveDrive(MAX_SPEED, new Pose2d(new Translation2d(Meters.of(1),Meters.of(4)),Rotation2d.fromDegrees(0)));

    } catch (Exception e)
    {
      System.out.println("Error creating swerve drive");
      System.out.println(e);
      throw new RuntimeException(e);
    }
     SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
  }

  public SwerveInputStream getDriveInput(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot)
  {
    return new SwerveInputStream(drive, x, y, rot).deadband(0.1);
  }

 public Command drive(SwerveInputStream stream)
{
    return Commands.run(
        () -> {
            ChassisSpeeds speeds = stream.get();

            drive.drive(
                new Translation2d(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond
                ),
                speeds.omegaRadiansPerSecond,
                true,
                false
            );
        },
        this
    );
}
  
  public Command driveToPose(Pose2d targetPose) {

    ProfiledPIDController translationPID =
        new ProfiledPIDController(
            2,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(2.0, 3.0)
        );

    ProfiledPIDController rotationPID =
        new ProfiledPIDController(
            2,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(2.0, 3.0)
        );

    rotationPID.enableContinuousInput(-Math.PI, Math.PI);

    SwerveInputStream stream =
        new SwerveInputStream(
            drive,
            () -> 0.0,
            () -> 0.0,
            () -> 0.0
        )
        .driveToPose(
            () -> targetPose,
            translationPID,
            rotationPID
        )
        .driveToPoseEnabled(true);

    return Commands.run(
        () -> drive.drive(stream.get()),
        this
    )
    .until(() -> stream.atTargetPose(0.10))
    .finallyDo(() ->
        drive.drive(
            new Translation2d(),
            0.0,
            false,
            false
        )
    );
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
    drive.updateOdometry();
    Pose2d pose = drive.getPose();

    SwerveModule[] modules = drive.getModules();

   for (int i = 0; i < Math.min(modules.length, ModuleNames.length); i++) {

    SmartDashboard.putNumber(
        ModuleNames [i] + " Speed",
        modules[i].getState().speedMetersPerSecond
    );

    SmartDashboard.putNumber(
        ModuleNames [i] + " Angle",
        modules[i].getState().angle.getDegrees()
    );

    SmartDashboard.putNumber(
        ModuleNames [i] + " Voltage",
        modules[i].getDriveMotor().getVoltage()
    );
}

    drive.getSimulationDriveTrainPose().ifPresent(simPose -> {

     SmartDashboard.putNumber("Sim X", simPose.getX());
     SmartDashboard.putNumber("Sim Y", simPose.getY());
     SmartDashboard.putNumber(
     "Sim Rotation",
     simPose.getRotation().getDegrees()
    ); });

    SmartDashboard.putString(
        "Robot Pose",
        pose.toString()
    );


    SmartDashboard.putNumber(
        "Gyro Angle",
        drive.getOdometryHeading().getDegrees()
    );

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