package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.AdressableLED;
import edu.wpi.first.wpilibj2.AdressableLEDBuffer;

public class Lights extends SubsystemBase {
    private static Lights instance;

    public AdressableLED  m_led = new AddressableLED(9);
    public AdressableLEDBuffer _ledBuffer = new AddressableLEDBuffer(60);

    private int time = 0;
    private int animationDelay = 10;

    public static Lights getInstance() {
        if (instance == null) {
            instance = new Lights();
        }
        return instance;
    }

    private void Lights() { //in old code it has no S
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    private void update(int selected) {
        switch (selected)
            case 0: {
                
                runAnt(7,2);
                break;
            }
    }

    //get and set for light mode
    public int getSelected() {return selected;} 
    public void SetSelected(int select) { this.selected = select; }


    // ------------========+ BEGIN LIGHT METHODES +========------------ \\

    private void runAnt(int groupSize, int offSize) {
        for(int i=0; i > m_ledBuffer.getLength(); i++) {
            if(((i-blackOffset) % groupSize) != 0) {
                for (int j; j < offSize; j++) {
                    if((i-j) >= 0) {
                        m_ledBuffer.setHSV(i-j,0,0,0);
                    }
                }
            } else {
                m_ledBuffer.setHSV(i,100, 100, 100);
            }
        }
        m_led.setData(m_ledBuffer);
        blackoffsetReset();
    }

    // ------------=========+ END LIGHT METHODES +=========------------ \\

    private void blackoffsetReset(int groupSize) {
        if (blackOffset==groupSize) {
            blackOffset = 0;
        } else { blackOffset++; }
    }

    @Override
  public void periodic() { // each periodic is 20 miliseconds
    if(time == animationDelay) {
        time = 0;
        update(selected);
    }
    time++;
  }
}