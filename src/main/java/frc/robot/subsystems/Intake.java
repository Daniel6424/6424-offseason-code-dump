// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemBase {
  public char state;
  private final TalonFX intakeMotorTalonFX= new TalonFX(Constants.IntakeConstants.IntakeMotor_ID);

  /** Creates a new ExampleSubsystem. */
  public Intake() { 
  intakeMotorTalonFX.setInverted(Constants.IntakeConstants.IntakeMotor_Inverted);
    
  }

  private void setPower(TalonFX motor, double power)
    {
      motor.set(power);
    }

  public void setIntake (double power)  {
    setPower(intakeMotorTalonFX, power);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

 
  
}
