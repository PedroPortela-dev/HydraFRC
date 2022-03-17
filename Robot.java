package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  //Motores
  VictorSPX m_right = new VictorSPX(1);
  VictorSPX m_right2 = new VictorSPX(2);
  VictorSPX m_left = new VictorSPX(3);
  VictorSPX m_left2 = new VictorSPX(4);
  //VictorSPX m_Coleta = new VictorSPX(8);
  VictorSPX m_armazem = new VictorSPX(7);
  VictorSPX m_lancamento = new VictorSPX(6);
  VictorSPX m_lancamento2 = new VictorSPX(8);

  //Joystick
  Joystick m_stick = new Joystick(0);
  Joystick m_stick2 = new Joystick(1);

  Timer mTimer = new Timer();

  double x1, y1, x2, y2, lt, rt, mag, motorE, motorD, sin1, bt=0.2, mag2, sin2, pl, pr, t1, t2,
  t3, t4, t5, pc, pa, ys2, motorC, tLanc, pLanc, pLanc2;
  int pov, i=0, j=0;
  boolean a, b, x, y, davi, daviA, daviX, daviB;
  double[][] vetpower = new double[700][2];

  public Robot(){
    super(0.05);
  }

  @Override
  public void robotInit(){
    //Motores Servos
    m_right2.follow(m_right);
    m_left2.follow(m_left);

    //Configura Modo Neutro para 4%
    m_left.configNeutralDeadband(0.04);
    m_left2.configNeutralDeadband(0.04);
    m_right.configNeutralDeadband(0.04);
    m_right2.configNeutralDeadband(0.04);

    //Travar motores caso força seja 0
    m_left.setNeutralMode(NeutralMode.Brake);
    m_left2.setNeutralMode(NeutralMode.Brake);
    m_right.setNeutralMode(NeutralMode.Brake);
    m_right2.setNeutralMode(NeutralMode.Brake);
    //m_Coleta.setNeutralMode(NeutralMode.Brake);
    m_armazem.setNeutralMode(NeutralMode.Brake);
    m_lancamento.setNeutralMode(NeutralMode.Brake);
    m_lancamento2.setNeutralMode(NeutralMode.Brake);
  }

   

  @Override
  public void teleopInit() {
    for(i=0;i<700;i++){
      for(j=0;j<2;j++){
        vetpower[i][j]=0;
      }
    }
    i=0;
    j=0;
    mTimer.reset();
    mTimer.start();
  }

 @Override
 public void teleopPeriodic(){
   
  x1=m_stick.getRawAxis(0);
  y1=-m_stick.getRawAxis(1);
  x2=m_stick.getRawAxis(4);
  y2=-m_stick.getRawAxis(5);
  rt=m_stick.getRawAxis(3);
  lt=-m_stick.getRawAxis(2);
  pov=m_stick.getPOV();
  a=m_stick.getRawButtonPressed(1);
  b=m_stick.getRawButtonPressed(2);
  x=m_stick.getRawButtonPressed(3);
  y=m_stick.getRawButtonPressed(4);
  mag = Math.sqrt(Math.pow(x1, 2) + Math.pow(y1, 2));
  mag2 = Math.sqrt(Math.pow(x2, 2) + Math.pow(y2, 2));

  daviA = m_stick2.getRawButton(1);
  daviB = m_stick2.getRawButton(2);
  daviX = m_stick2.getRawButton(3);

    if(daviA){
      SmartDashboard.putString("Condição","daviA");
      pLanc = 1;
      pLanc2 = 1;
    }else if(daviX){
      SmartDashboard.putString("Condicao","daviX");
      pLanc = 0.75;
      pLanc2 = 0.75;
    }else if(daviB){
      SmartDashboard.putString("Condicao","daviB");
      pLanc = 0.5;
      pLanc2 = 0.5; 
    }else{
      pLanc = 0;
      pLanc2 = 0;
    }
  
  t5=(m_stick2.getRawButton(4))?t5:mTimer.get();
  pa=(mTimer.get()-t5>0.45)?1:0;

  SmartDashboard.putNumber("TimerArmazenamento", mTimer.get()-t5);

  if(mag<0.1 && rt==0 && lt==0){
    t1=mTimer.get();
    mag=0;
    x1=0;
    y1=0;
  }
  if(mag2<0.1 && rt==0 && lt==0){
    t2=mTimer.get();
    mag2=0;
    x2=0;
    y2=0;
  }

  t3=(t3>=1)?1:(mTimer.get()-t1)*2;
  t4=(t4>=1)?1:(mTimer.get()-t2)*2;

  bt=(b)?0.2:(a)?0.4:(x)?0.6:bt;

  if(bt==0.25){
    SmartDashboard.putBoolean("Minimo", true);
    SmartDashboard.putBoolean("Medio", false);
    SmartDashboard.putBoolean("Maximo", false);
  }else if(bt==0.5){
    SmartDashboard.putBoolean("Minimo", false);
    SmartDashboard.putBoolean("Medio", true);
    SmartDashboard.putBoolean("Maximo", false);
  }else if(bt==1){
    SmartDashboard.putBoolean("Minimo", false);
    SmartDashboard.putBoolean("Medio", false);
    SmartDashboard.putBoolean("Maximo", true);
  }

  sin1=y1/mag;
  sin2=y2/mag2;

  if(rt!=0){
    if(x1>=0){
      if(x1<=0.04){
        x1=0;
     }
      SmartDashboard.putString("Condicao","rt!=0 && x1>=0");
      pr=(1-x1)*rt;
      pl=rt;
    }else{
      if(x1>-0.04){
        x1=0;
     }
      SmartDashboard.putString("Condicao","rt!=0 && x1<0");
      pr=rt;
      pl=(1+x1)*rt;
    }
  }
  else if(lt!=0){
    if(x1>=0){
      if(x1<0.04){
        x1=0;
     }
      SmartDashboard.putString("Condicao","lt!=0 && x1>=0");
      pr=lt;
      pl=(1-x1)*lt;
    }else{
      if(x1>-0.04){
        x1=0;
     }
      SmartDashboard.putString("Condicao","lt!=0 && x1<0");
      pr=(1+x1)*lt;
      pl=lt;
    }   
  }
  else if(mag!=0){
    if(x1>=0 && y1>=0){
      SmartDashboard.putString("Condicao","x1>=0 && y1>=0");
      pr=sin1*mag;
      pl=mag;
    }
    else if(x1<0 && y1>=0){
      SmartDashboard.putString("Condicao","x1<0 && y1>=0");
      pr=mag;
      pl=sin1*mag;
    }
    else if(x1<0 && y1<0){
      SmartDashboard.putString("Condicao","x1<0 && y1<0");
      pr=sin1*mag;
      pl=-mag;
    }
    else if(x1>=0 && y1<0){
      SmartDashboard.putString("Condicao","x1>=0 && y1<0");
      pr=-mag;
      pl=sin1*mag;
    }
  }
  else if(mag2!=0){
    if(x2>=0 && y2>=0){
      SmartDashboard.putString("Condicao","x2>=0 && y2>=0");
      pr=-mag2;
      pl=-sin2*mag2;
    }
    else if(x2<0 && y2>=0){
      SmartDashboard.putString("Condicao","x2<0 && y2>=0");
      pr=-sin2*mag2;
      pl=-mag2;
    }
    else if(x2<0 && y2<0){
      SmartDashboard.putString("Condicao","x2<0 && y2<0");
      pr=mag2;
      pl=-sin2*mag2;
    }
    else if(x2>=0 && y2<0){
      SmartDashboard.putString("Condicao","x2<0 && y2<0");
      pr=-sin2*mag2;
      pl=mag2;
    }
  }
  else if(pov!=-1){
    if(pov==0){
      SmartDashboard.putString("Condicao","pov==0");
      pr=1;
      pl=1;
    }
    else if(pov==45){
      SmartDashboard.putString("Condicao","pov==45");
      pr=0;
      pl=1;
    }
    else if(pov==90){
      SmartDashboard.putString("Condicao","pov==90");
      pr=-1;
      pl=1;
    }
    else if(pov==135){
      SmartDashboard.putString("Condicao","pov==135");
      pr=0;
      pl=-1;
    }
    else if(pov==180){
      SmartDashboard.putString("Condicao","pov==180");
      pr=-1;
      pl=-1;
    }
    else if(pov==225){
      SmartDashboard.putString("Condicao","pov==225");
      pr=-1;
      pl=0;
    }
    else if(pov==270){
      SmartDashboard.putString("Condicao","pov==270");
      pr=1;
      pl=-1;
    }
    else if(pov==315){
      SmartDashboard.putString("Condicao","pov==315");
      pr=1;
      pl=0;
    }
  }else{
    SmartDashboard.putString("Condicao","else");
    pr=0;
    pl=0;
  }

  SmartDashboard.putNumber("Power Right", pr);
  SmartDashboard.putNumber("Power Left", pl);

  pl=pl*((pov==-1)?(t3+t4):1)*bt;
  pr=pr*((pov==-1)?(t3+t4):1)*bt;

  pl = pl*((pov==-1)?(t3+t4):1)*bt;

  if(y){
    i++;
  }

  if(i==1){
      SmartDashboard.putBoolean("Gravador:", true);
      vetpower[j][0]=pl;
      vetpower[j][1]=pr;
      j++;
  }else{
    SmartDashboard.putBoolean("Gravador:", false);
  }
  if(i>1){
    SmartDashboard.putNumber("motor esquerdo", vetpower[i-2][0]);
    SmartDashboard.putNumber("motor direito", vetpower[i-2][1]);
  }

  ys2 = -m_stick2.getRawAxis(5);

  //m_Coleta.set(ControlMode.PercentOutput, -ys2);
  m_left.set(ControlMode.PercentOutput, pl);
  m_right.set(ControlMode.PercentOutput, -pr);
  m_armazem.set(ControlMode.PercentOutput, pa);
  m_lancamento.set(ControlMode.PercentOutput, -pLanc);
  m_lancamento2.set(ControlMode.PercentOutput, -pLanc2);
  SmartDashboard.putNumber("ForcaLancamento", pLanc);
  SmartDashboard.putNumber("ForçaLancamento2", pLanc2);
 }

 @Override
 public void autonomousInit() {
   i=0;
 }

 @Override
 public void autonomousPeriodic() {
  m_left.set(ControlMode.PercentOutput, vetpower[i][0]);
  m_right.set(ControlMode.PercentOutput, -vetpower[i][1]);
  SmartDashboard.putNumber("motor esquerdo", vetpower[i][0]);
  SmartDashboard.putNumber("motor direito", vetpower[i][1]);
  i++;
 }

}
