// Copyright (c) FIRST and other WPILib contributors. 136
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private DigitalInput limitHI= new DigitalInput(Constants.ArmConstants.HILimitPin);
  private DigitalInput limitLO= new DigitalInput(Constants.ArmConstants.LOLimitPin);
  private final TalonFX wristMotorTalonFX= new TalonFX(Constants.ArmConstants.wristMotor_ID);
  private final CANcoder wristCaNcoder= new CANcoder(Constants.ArmConstants.WristCANcoder_ID);
  private final TalonFX armTalonFX= new TalonFX(Constants.ArmConstants.armMotor_ID);
  private final CANcoder armCancoder= new CANcoder(Constants.ArmConstants.armCANcoder_ID);

  
  private final PIDController wristController=new PIDController(2,0,0);
  private ShuffleboardTab Wristtab= Shuffleboard.getTab("Wrist");
  private GenericEntry Wristvalue=Wristtab.add("WristValue",0).getEntry();

  private final PIDController armController=new PIDController(4,0,0);

  public Arm() {
    wristMotorTalonFX.setNeutralMode(NeutralModeValue.Brake);
    armTalonFX.setNeutralMode(NeutralModeValue.Brake);
    //wrisController.setTolerance(0.1);
    armController.setTolerance(0.2);
  }  
  
  // Place Getter methods Here

  public double getWristValue(){
    return wristCaNcoder.getAbsolutePosition().getValueAsDouble();
  }
  public double getArmValue(){
    return armCancoder.getAbsolutePosition().getValueAsDouble();
  }
  public double getWristSetpoint(){
    return this.wristController.getSetpoint();
  }
  public boolean wristAtSetpoint(){
    return this.wristController.atSetpoint();
  }
   public boolean armAtSetpoint(){
    return this.armController.atSetpoint();
  }
  // Place Setter Methods Here
 private void setPower(TalonFX motor, double power){
    motor.set(power);
  }

  public void setWristMotor (double power){
    setPower(wristMotorTalonFX, wristSoftLimit(power));
  }

  public void setArmMotor (double power)  {
    setPower(armTalonFX, armSoftLimit(power));
    }
  
  //Place other Methods Here 
  /* Wrist limit thing */
  private double wristSoftLimit(double power)
  {
    double output=0;
    if((limitHI.get()&&power>0)||(limitLO.get()&&power<0))
    {
      output=0;
    }else{
      output=power;
    }
    return output;
  }
  
  /* Arm limit thing */
  
  private double armSoftLimit(double power){
    double output=0;
    if(armCancoder.getAbsolutePosition().getValueAsDouble()>.82||armCancoder.getAbsolutePosition().getValueAsDouble()<.62){
      if(armCancoder.getAbsolutePosition().getValueAsDouble()>.82){
        if(power>0){
        output=0;
        }
        else{
          output=power;
        }
      } else{
        if(power<0){
        output=0;
        }
        else{
          output=power;
        }
      }
    } else{
      output=power;
    }
    return output;
  }

 


  public void setWristPID(double setPoint){
    this.wristController.setSetpoint(setPoint);
  }

  private void executeWristPID(){
    setWristMotor(wristSoftLimit(this.wristController.calculate(this.getWristValue())
            )); 
  }


   public void setArmPID(double setPoint){
    this.armController.setSetpoint(setPoint);
  }

  private void executeArmPID(){
    setArmMotor(armSoftLimit(this.armController.calculate(this.getArmValue())
            )); 
  }

  
  // Place Commands hear
 /*  public Command PidWristCommand (double wristSetPoint) {
   return run(()->setWristPID(wristSetPoint));
  }


 public Command PidArmCommand (double armSetPoint) {
   return run(()->setArmPID(armSetPoint));
  }
*/
  

  public Command PIDWristCommand(double setPoint)
  {
    return new FunctionalCommand(
      ()->this.setWristPID(setPoint),

      ()-> this.executeWristPID(),
      interrupted->this.setWristMotor(0),
      
      ()->this.wristAtSetpoint(),
       this);
  }
  
  public Command PIDArmCommand(double setPoint)
  {
    return new FunctionalCommand(
      ()->this.setArmPID(setPoint),

      ()-> this.executeArmPID(),
      interrupted->this.setArmMotor(0),
      
      ()->this.armAtSetpoint(),
       this);
  }

  public Command HighPosCommand()
  {
    return new FunctionalCommand(
      ()->this.setAllSetPoints(.8,.4),

      ()-> this.executeArmPIDS(),
      interrupted->this.breakArm(),
      
      ()->this.armAtHigh(),
       this);
  }



  private void setAllSetPoints(double armSetPoint, double wristSetPoint){

    this.setArmPID(armSetPoint);
    this.setWristPID(wristSetPoint);
  }
  public void executeArmPIDS(){
   this.executeArmPID();
  this.executeWristPID();
  }
  public void breakArm(){
   this.setArmMotor(0);
   this.setWristMotor(0);
  }
  public boolean armAtHigh(){
   return (this.armAtSetpoint()&&this.wristAtSetpoint());
   
  }



  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }





   
 

  



}
