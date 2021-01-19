package frc.robot;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends TimedRobot {
  SpeedController m_frontLeft = new PWMVictorSPX(0);
  SpeedController m_rearLeft = new PWMVictorSPX(1);
  SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);

  SpeedController m_frontRight = new PWMVictorSPX(2);
  SpeedController m_rearRight = new PWMVictorSPX(3);
  SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);

  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
  Joystick m_stick= new Joystick(0);
  AnalogGyro m_gyro = new AnalogGyro(0);

  boolean verificacao,modelocomotion;
  double x,y,AnguloCarro,AnguloJoystick;

  @Override
  public void teleopPeriodic()
  {
    if(getMagnitude(m_stick.getRawAxis(0), m_stick.getRawAxis(1))!=0 && !modelocomotion)
    {
      x=m_stick.getRawAxis(0);
      y=-m_stick.getRawAxis(1);

      modelocomotion=true;
    }else
    {
      x=m_stick.getRawAxis(0);
      y=-m_stick.getRawAxis(1);

      modelocomotion=false;
    }

    AnguloJoystick = getDirectionRadiansJoystick(x, y);
    AnguloCarro = getDirectionRadiansRobot(x, y);

    if(modelocomotion)
    {
      driveCarro(x, y);
    }else
    {
      driveTradicional(x, y);
    }
  }

  public void drivePowerController(double x, double y)
  {
    m_drive.arcadeDrive(m_stick.getRawAxis(2), cosMagnitude(x, y));
  }

  public void driveMira()
  {
    m_drive.arcadeDrive(getControllerDirection(), getControllerPower());
  }

  public void driveCarro(double x, double y)
  {

    driveTradicional(Math.cos(getDirectionRadiansVetor())*getMagnitude(x, y), Math.sin(getDirectionRadiansVetor())*getMagnitude(x, y));
  }

  public void driveTradicional(double x, double y)
  {
    if(y>0){
      m_drive.arcadeDrive(getMagnitude(x, y), cosMagnitude(x, y));
    }else if(y<0){
      m_drive.arcadeDrive(-getMagnitude(x, y), -cosMagnitude(x, y));
    }else
    {
      m_drive.arcadeDrive(0, 0);
    }
  }

  public double getMagnitude(double x,double y)
  {
    return Math.sqrt(Math.pow(x,2)*Math.pow(y,2));
  }

  public double cosMagnitude(double x,double y)
  {
    return x/getMagnitude(x, y);
  }

  public double sinMagnitude(double x,double y)
  {
    return y/getMagnitude(x, y);
  }

  public double getDirectionRadiansJoystick(double x, double y)
  {
    double num1 = Math.acos(cosMagnitude(x, y)),num2 = Math.asin(sinMagnitude(x, y));
    if(num1==0)
    {
      return num2;
    }else if(num2==Math.PI/2)
    {
      if(num1==Math.PI/2)
      {
        return num1;
      }else
      {
        return 2*Math.PI+num1;
      }
    }else
    {
      if(num1>0)
      {
        return num2;
      }else
      {
        return 2*Math.PI-num2;
      }
    }
  }

  public double getDirectionRadiansRobot(double x, double y)
  {
    if(getMagnitude(x, y)==0)
    {
      verificacao=false;
      m_gyro.reset();
    }else if(!verificacao)
    {
      verificacao=true;
      m_gyro.initGyro();
    }

    return m_gyro.getAngle();
  }

  public double getDirectionRadiansVetor()
  {
    if(AnguloJoystick-AnguloCarro<Math.PI/2)
    {
      return AnguloJoystick-AnguloCarro+5*Math.PI/2;
    }else if(AnguloJoystick-AnguloCarro>5*Math.PI/2)
    {
      return AnguloJoystick-AnguloCarro-3*Math.PI/2;
    }else
    {
      return AnguloJoystick-AnguloCarro+Math.PI/2;
    }
  }

  public double getControllerDirection()
  {
    if(AnguloCarro<Math.PI)
    {
      if(AnguloJoystick==AnguloCarro || AnguloJoystick==AnguloCarro+Math.PI)
      return 0;
      if(AnguloJoystick>AnguloCarro && AnguloJoystick<AnguloCarro+Math.PI)
      return -1;
      return 1;
    }
    if(AnguloJoystick==AnguloCarro || AnguloJoystick==AnguloCarro-Math.PI)
    return 0;
    if(AnguloJoystick<AnguloCarro && AnguloJoystick>AnguloCarro-Math.PI)  
    return 1;
    return -1;
  }
  
  public double getControllerPower()
  {
    if(AnguloJoystick<Math.PI)
    {
      if(AnguloCarro==AnguloJoystick || AnguloCarro == AnguloJoystick+Math.PI)
      return 0;
      if(AnguloCarro>AnguloJoystick && AnguloCarro<AnguloJoystick+Math.PI)
      return Math.sqrt((AnguloCarro-AnguloJoystick)/Math.PI);
      if(AnguloCarro<AnguloJoystick)
      return Math.sqrt((AnguloJoystick-AnguloCarro)/Math.PI);
      return Math.sqrt((2*Math.PI-AnguloCarro+AnguloJoystick)/Math.PI);
    }
    if(AnguloCarro==AnguloJoystick || AnguloCarro == AnguloJoystick-Math.PI)
    return 0;
    if(AnguloCarro<AnguloJoystick && AnguloCarro>AnguloJoystick-Math.PI)
    return Math.sqrt((AnguloJoystick-AnguloCarro)/Math.PI);
    if(AnguloCarro<AnguloJoystick-Math.PI)
    return Math.sqrt((2*Math.PI+AnguloCarro-AnguloJoystick)/Math.PI);
    return Math.sqrt((AnguloCarro-AnguloJoystick)/Math.PI);
  }
}