// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.comands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
/** An example command that uses an example subsystem. */
public class ArmManual extends Command {

  private Arm arm;
  private DoubleSupplier armJoystick;
  private BooleanSupplier OutSupplier;
  private BooleanSupplier InSupplier;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param wristOut 
   * @param wristIn 
   */
  public ArmManual(Arm subsystem,DoubleSupplier armJoystick, BooleanSupplier wristOut, BooleanSupplier wristIn ) {
    arm = subsystem;
    OutSupplier = wristOut;
    InSupplier = wristIn;
   this.armJoystick=armJoystick;
    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setArmMotor(0);
    arm.setWristMotor(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
  arm.setArmMotor(armJoystick.getAsDouble()*.2); 
  
  if(OutSupplier.getAsBoolean()||InSupplier.getAsBoolean()){
    if(OutSupplier.getAsBoolean()){
     arm.setWristMotor(.1);
    } else{
      arm.setWristMotor(-.1);
    }
  } else{
    arm.setWristMotor(0);
  }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
