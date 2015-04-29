package orc;

public class DigitalPWM
{
    Orc orc;
    int port;
    
    public DigitalPWM(final Orc orc, final int port) {
        super();
        this.orc = orc;
        this.port = port;
    }
    
    public void setPWM(final int period_usec, final double dutyCycle) {
        assert dutyCycle >= 0.0 && dutyCycle <= 1.0;
        assert period_usec >= 1000 && period_usec <= 1000000;
        final int iduty = (int)(dutyCycle * 4095.0);
        final int command = (iduty << 20) + period_usec;
        this.orc.doCommand(28672, new byte[] { (byte)(this.port - 8), 2, (byte)(command >> 24 & 0xFF), (byte)(command >> 16 & 0xFF), (byte)(command >> 8 & 0xFF), (byte)(command >> 0 & 0xFF) });
    }
    
    public void setValue(final boolean v) {
    }

    /*
    public static void main(final String[] args) {
        final Orc orc = Orc.makeOrc();
        final DigitalOutput dout = new DigitalOutput(orc, 0);
    Label_0014_Outer:
        while (true) {
            while (true) {
                try {
                    while (true) {
                        dout.setValue(true);
                        System.out.println("true");
                        Thread.sleep(1000L);
                        dout.setValue(false);
                        System.out.println("false");
                        Thread.sleep(1000L);
                    }
                }
                catch (InterruptedException ex) {
                    continue Label_0014_Outer;
                }
                continue;
            }
        }
    }
    */
}
