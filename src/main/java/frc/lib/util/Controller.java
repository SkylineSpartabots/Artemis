package frc.lib.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller {
    private XboxController xbox;
    // private static final byte DPAD_U_PORT = -1;
    // private static final byte DPAD_D_PORT = -3;
    // private static final byte DPAD_L_PORT = -4;
    // private static final byte DPAD_R_PORT = -2;
    private Trigger a, b, x, y, rb, lb, lstick, rstick, back, start;
    // private POVButton up, down, left, right;
    public Controller(XboxController xbox){
        this.xbox = xbox;
        a = new Trigger(xbox::getAButton);
        b = new Trigger(xbox::getBButton);
        x = new Trigger(xbox::getXButton);
        y = new Trigger(xbox::getYButton);
        lb = new Trigger(xbox::getLeftBumper);
        rb = new Trigger(xbox::getRightBumper);
        lstick = new Trigger(xbox::getLeftStickButton);
        rstick = new Trigger(xbox::getRightStickButton);
        back = new Trigger(xbox::getBackButton);
        start = new Trigger(xbox::getStartButton);
    }
    
    public boolean getDpadUp(){ return xbox.getPOV() == 0; }
    public boolean getDpadUpRight(){ return xbox.getPOV() > 15 && xbox.getPOV() < 75; }
    public boolean getDpadRight(){ return xbox.getPOV() == 90;}
    public boolean getDpadDown(){ return xbox.getPOV() == 180;}
    public boolean getDpadLeft(){ return xbox.getPOV() == 270;}

    public double getRightTriggerAxis(){ return xbox.getRightTriggerAxis();}
    public double getLeftTriggerAxis(){ return xbox.getLeftTriggerAxis();}

    public Trigger getAButton(){ return a;}
    public Trigger getBButton(){ return b;}
    public Trigger getXButton(){ return x;}
    public Trigger getYButton(){ return y;}
    public Trigger getLeftBumper(){ return lb;}
    public Trigger getRightBumper(){ return rb;}
    public Trigger getLeftStickButton(){ return lstick;}
    public Trigger getRightStickButton(){ return rstick;}
    public Trigger getBackButton(){ return back;}
    public Trigger getStartButton(){ return start;}

    public double getLeftX(){ return xbox.getLeftX();}
    public double getLeftY(){ return xbox.getLeftY();}
    public double getRightX(){ return xbox.getRightX();}
    public double getRightY(){ return xbox.getRightY();}
}