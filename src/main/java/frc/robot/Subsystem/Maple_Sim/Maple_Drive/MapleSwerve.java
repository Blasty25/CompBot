package frc.robot.Subsystem.Maple_Sim.Maple_Drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystem.Swerve.Drive;

import edu.wpi.first.wpilibj2.command.RunCommand;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoral;

public class MapleSwerve implements SwerveDrive {
  private final SelfControlledSwerveDriveSimulation simulatedDrive;
  private final Field2d field2d;

  public MapleSwerve() {
    final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
        // Specify gyro type (for realistic gyro drifting and error simulation)
        .withGyro(COTS.ofPigeon2())
        // Specify swerve module (for realistic swerve dynamics)
        .withSwerveModule(COTS.ofMark4(
            DCMotor.getNEO(1), // Drive motor is a Kraken X60
            DCMotor.getNEO(1), // Steer motor is a Falcon 500
            COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
            3)) // L3 Gear ratio
        // Configures the track length and track width (spacing between swerve modules)
        .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
        // Configures the bumper size (dimensions of the robot bumper)
        .withBumperSize(Inches.of(30), Inches.of(30));

        this.simulatedDrive = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(driveTrainSimulationConfig, new Pose2d(0, 0, new Rotation2d())));

    // register the drivetrain simulation to the simulation world
    SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());
    SimulatedArena.getInstance().addGamePiece(new ReefscapeCoral(
        // We must specify a heading since the coral is a tube
        new Pose2d(2, 2, Rotation2d.fromDegrees(90))));

    field2d = new Field2d();
    SmartDashboard.putData("simulation field", field2d);
  }

  @Override
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    this.simulatedDrive.runChassisSpeeds(
        new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
        new Translation2d(),
        fieldRelative,
        true);
      }

public Command mapleDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return new RunCommand(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), 0.1);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), 0.1);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        this);
  }

  @Override
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    simulatedDrive.runSwerveStates(desiredStates);
  }

  @Override
  public ChassisSpeeds getMeasuredSpeeds() {
    return simulatedDrive.getMeasuredSpeedsFieldRelative(true);
  }

  @Override
  public Rotation2d getGyroYaw() {
    return simulatedDrive.getRawGyroAngle();
  }

  @Override
  public Pose2d getPose() {
    return simulatedDrive.getOdometryEstimatedPose();
  }

  @Override
  public void setPose(Pose2d pose) {
    simulatedDrive.setSimulationWorldPose(pose);
    simulatedDrive.resetOdometry(pose);
  }

  @Override
  public void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds) {
    simulatedDrive.addVisionEstimation(visionRobotPose, timeStampSeconds);
  }

  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
    simulatedDrive.addVisionEstimation(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  @Override
  public void periodic() {
    // update the odometry of the SimplifedSwerveSimulation instance
    simulatedDrive.periodic();

    // send simulation data to dashboard for testing
    field2d.setRobotPose(simulatedDrive.getActualPoseInSimulationWorld());
    field2d.getObject("odometry").setPose(getPose());
  }
}
