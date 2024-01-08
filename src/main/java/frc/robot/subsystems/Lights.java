package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;


public class Lights extends SubsystemBase {
    private static Lights instance;

    public AddressableLED  m_led = new AddressableLED(9);
    public AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);

    private int time = 0;
    private int animationDelay = 10;

    public static Lights getInstance() {
        if (instance == null) {
            instance = new Lights();
        }
        return instance;
    }

    private int selected = 0; // selected mode of LEDs
    private int blackOffset = 0; // k; turns off ever nth LED // is a global variable cause we want the ants to continue thier progress even when switching modes

    private void Lights() { //in old code it has no S
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    private void update(int selected) {
        switch (selected){
            case 0: {
                
                runAnt(7,2);
                break;
            }
        }
    }

    // Get and Set LED mode
    public int getSelected() {return selected;} 
    public void SetSelected(int select) { this.selected = select; }

    // ------------========+ BEGIN LIGHT METHODS +========------------ \\

    private void runAnt(int groupSize, int offSize) {
        for(int i=0; i > m_ledBuffer.getLength(); i++) {
            if(((i-blackOffset) % groupSize) != 0) {
                for (int j = 0; j < offSize; j++) {
                    if((i-j) >= 0) {
                        m_ledBuffer.setHSV(i-j,0,0,0);
                    }
                }
            } else {
                m_ledBuffer.setHSV(i,100, 100, 100);
            }
        }
        m_led.setData(m_ledBuffer);
        blackOffsetReset(groupSize);
    }

    // ------------=========+ END LIGHT METHODS +=========------------ \\

    private void blackOffsetReset(int groupSize) {
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


    // ------------========+ ANT MATH EXPLAINER +========------------ \\ 
/* so i dont forget how it works
Assuming a groupSize (g) of 5 and a blackOffset (k) of 0 and a blackSize/offsize (b) of 5
The goal is to turn every 5th (g) LED (i) off 
To find if i is the 5th LED we subtract k from i to get some number which if it is the 5th LED it should be evenly divisible by 5 (g)
If (i - k) % g == 0 then the LED at i should be set to black
If (i - k) % g != 0 then the LED at i should be set to the desired color, in this case red

We first progress through every LED in the strip, every value of i
The above formula will turn every 5th LED off
(0 - 0) % 5 == 0 and (5 - 0) % 5 == 0 but (2 - 0) % 5 != 0


But we want the black spaces to move to create the animation

To do this we must increment k after we have looped throught the whole LED strip
This makes it so that every 5th LED is still off but it is now one value of i farther down the strip
For example: (1 - 1) % 5 == 0 and  (6 - 1) % 5 == 0 mean that LEDs 1 & 6 are black
but the previously black LED with i value of 5 will be red because (5 - 1) % 5 != 0

**Remember that i of 1 is actually the second LED in the stip as 0 is the first**

We continue this looping through every i value and then incrementing k until k is equal to g
When k == g we can reset it to be 0 because (5 - 0) % 5 == 0 and (5 - 5) % 5 == 0
This helps to keep numbers small

Now one final thing, what if we want more than one LED to be turned off
Simple we create a b value which tells us how many LEDs behind the currently off LED should also be set to black
It looks like this: if (i - k) % g == 0 then we will set the LEDs from i - b to i as black

*/
