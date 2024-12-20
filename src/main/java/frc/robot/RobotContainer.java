// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.comands.IntakeSuck;
import frc.robot.comands.WristManual;
import frc.robot.comands.ArmManual;
import frc.robot.comands.HighGoal;
import frc.robot.comands.IntakeSTOP;
import frc.robot.comands.IntakeSpit;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  private static final Command HighGoal = null;
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandJoystick joystick = new CommandJoystick(0); // My joystick
  private final CommandJoystick buttonbord = new CommandJoystick(1);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Intake intake=new Intake();
  private final Arm arm=new Arm();
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.RobotCentric robotCentricDrive=new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.1 ).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);// I want Robot-centric
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  //system based commands
  private final Command wristMidPoint= arm.PIDWristCommand(.25);
  private final Command armMidPoint= arm.PIDArmCommand(.7);

  private void configureBindings() {

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> (drive.withVelocityX(-joystick.getRawAxis(1) * MaxSpeed*(1-joystick.getRawAxis(3)*0.8))) // Drive forward with
            .withVelocityY(-joystick.getRawAxis(0) * MaxSpeed*(1-joystick.getRawAxis(3)*0.8)) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRawAxis(4) * MaxAngularRate*(1-joystick.getRawAxis(3)*0.8)) // Drive counterclockwise with negative X (left)
        ));

    joystick.button(8).toggleOnTrue( drivetrain.applyRequest(() -> (robotCentricDrive.withVelocityX(-joystick.getRawAxis(1) * MaxSpeed*(1-joystick.getRawAxis(3)*0.8))) // Drive forward with
            .withVelocityY(-joystick.getRawAxis(0) * MaxSpeed*(1-joystick.getRawAxis(3)*0.8)) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRawAxis(4) * MaxAngularRate*(1-joystick.getRawAxis(3)*0.8)) // Drive counterclockwise with negative X (left)
        ));
    intake.setDefaultCommand(new IntakeSTOP(intake));

    joystick.button(4).whileTrue(drivetrain.applyRequest(() -> brake));

    joystick.button(5).whileTrue(new IntakeSpit(intake));

    joystick.button(6).whileTrue(new IntakeSuck(intake));


    buttonbord.button(9).toggleOnTrue (arm.HighPosCommand());
    

    buttonbord.button(10).toggleOnTrue (armMidPoint);

    buttonbord.button(11).toggleOnTrue (Commands.sequence(arm.HighPosCommand(),new IntakeSpit(intake).withTimeout(5)));
    arm.setDefaultCommand(new ArmManual(arm,()->buttonbord.getRawAxis(1),()->buttonbord.button(3).getAsBoolean(),()->buttonbord.button(6).getAsBoolean()));
    


    
    //joystick.b().whileTrue(drivetrain
        //.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.button(2).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }
  

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("test");
  }
}
