// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

// Subsystems and commands
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.commands.ClimbCommand;

import frc.robot.subsystems.ClawSubsystem;
import frc.robot.commands.ClawCommand;

import frc.robot.subsystems.ElavatorSubsystem;
import frc.robot.commands.ElavatorCommand;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.ArmCommand;

public class RobotContainer
{
  // -------------------------------
  // Controllers
  // -------------------------------
  final CommandXboxController driverXbox   = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);
  final CommandXboxController buttonBoard  = new CommandXboxController(2);

  // POV (D-pad) buttons for operator
  private final POVButton dpadUpButton    = new POVButton(operatorXbox.getHID(), 0);
  private final POVButton dpadRightButton = new POVButton(operatorXbox.getHID(), 90);
  private final POVButton dpadDownButton  = new POVButton(operatorXbox.getHID(), 180);
  //private final POVButton dpadLeftButton  = new POVButton(operatorXbox.getHID(), 270);

  // -------------------------------
  // Subsystems
  // -------------------------------
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve")
  );

  private final ClimbSubsystem climbSubsystem     = new ClimbSubsystem();
  private final ClawSubsystem  ClawSubsystem      = new ClawSubsystem();
  private final ElavatorSubsystem ElavatorSubsystem = new ElavatorSubsystem();
  private final ArmSubsystem   ArmSubsystem       = new ArmSubsystem();

  // -------------------------------
  // Drive commands
  // -------------------------------
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(
      drivebase,
      () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.DEADBAND),
      () -> -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
      driverXbox.getHID()::getYButtonPressed,
      driverXbox.getHID()::getAButtonPressed,
      driverXbox.getHID()::getXButtonPressed,
      driverXbox.getHID()::getBButtonPressed
  );

  // Convert driver input to field-relative controls
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1
  )
  .withControllerRotationAxis(driverXbox::getRightX)
  .deadband(OperatorConstants.DEADBAND)
  .scaleTranslation(0.8)
  .allianceRelativeControl(true)
  .headingOffset(true)
  .headingOffset(drivebase.getHeading());

  // Field-relative input stream
  SwerveInputStream driveDirectAngle =
      driveAngularVelocity.copy()
                          .withControllerHeadingAxis(
                              () -> driverXbox.getRightX() * -1,
                              () -> driverXbox.getRightY() * -1
                          )
                          .headingWhile(true);

  // Default drive commands
  Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  Command driveSetpointGen                  = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  // Sim-specific
  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX()
  )
  .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
  .deadband(OperatorConstants.DEADBAND)
  .scaleTranslation(0.8)
  .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngleSim =
      driveAngularVelocitySim.copy()
                             .withControllerHeadingAxis(
                                 () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                                 () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2)
                             )
                             .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);
  Command driveSetpointGenSim             = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  // ----------------------------------
  // Constructor
  // ----------------------------------
  public RobotContainer()
  {
    // Make sure we bind all the buttons
    configureBindings();

    // Zero the gyro to correct alliance if needed
    drivebase.zeroGyro();
    DriverStation.silenceJoystickConnectionWarning(true);

    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  // ----------------------------------
  // Button/POV bindings
  // ----------------------------------
  private void configureBindings()
  {
    // Set the default drive command, depending on whether we are in sim or real
    drivebase.setDefaultCommand(
        !RobotBase.isSimulation()
            ? driveFieldOrientedDirectAngle
            : driveFieldOrientedDirectAngleSim
    );

    ElavatorSubsystem.setDefaultCommand(new ElavatorCommand(ElavatorSubsystem, .1)); 
 
    ClawSubsystem.setDefaultCommand(new ClawCommand(ClawSubsystem, .2));

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> 
          drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d())))
      );
    }

    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

      driverXbox.x().whileTrue(
          Commands.runOnce(drivebase::lock, drivebase).repeatedly()
      );
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
    }
    else
    {
      // ------------------
      // Driver controls
      // ------------------
      driverXbox.a().onTrue(Commands.runOnce(drivebase::zeroGyro));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(6.161, 3.905), Rotation2d.fromDegrees(158.806))
          )
      );
      driverXbox.y().whileTrue(drivebase.aimAtSpeaker(2));

      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(
          Commands.runOnce(drivebase::lock, drivebase).repeatedly()
      );

      // ------------------
      // Operator controls
      // ------------------

      // Climb
      operatorXbox.rightBumper().whileTrue(new ClimbCommand(climbSubsystem, 0.5)); // UP
      operatorXbox.leftBumper().whileTrue(new ClimbCommand(climbSubsystem, -0.5)); // DOWN

      // Claw
      operatorXbox.a().onTrue(new ClawCommand(ClawSubsystem, .2));  // IN
      operatorXbox.b().whileTrue(new ClawCommand(ClawSubsystem, 0));  // OUT?
      // (If you really want "out" to be reversed, use -1 for the second line)

      // Elevator on D-pad
      dpadUpButton.whileTrue(new ElavatorCommand(ElavatorSubsystem, .5));    // SlideUP
      dpadDownButton.whileTrue(new ElavatorCommand(ElavatorSubsystem, -0.5)); // SlideDOWN
      dpadRightButton.onTrue(new ElavatorCommand(ElavatorSubsystem, 0)); 

      // Arm
      operatorXbox.x().whileTrue(new ArmCommand(ArmSubsystem, 0.35));  // ArmUP
      operatorXbox.y().whileTrue(new ArmCommand(ArmSubsystem, -0.35)); // ArmDOWN
    }
  }

  // ----------------------------------
  // Autonomous
  // ----------------------------------
  //public Command getAutonomousCommand()
  {
    // Example command used for autonomous
    //return drivebase.getAutonomousCommand("Blue Auto 1");
  }

  // ----------------------------------
  // Extra methods
  // ----------------------------------
  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
