package orc;

public class DigitalOutput
{
    Orc orc;
    int port;
    boolean invert;
    
    public DigitalOutput(final Orc orc, final int port) {
        this(orc, port, false);
    }
    
    public DigitalOutput(final Orc orc, final int port, final boolean invert) {
        super();
        this.orc = orc;
        this.port = port;
        this.invert = invert;
        if (port < 8) {
            orc.doCommand(24576, new byte[] { (byte)port, 0, 0 });
        }
        else {
            orc.doCommand(28672, new byte[] { (byte)(port - 8), 2, 0, 0, 0, 0 });
        }
    }
    
    public void setValue(final boolean v) {
        if (this.port < 8) {
            this.orc.doCommand(24577, new byte[] { (byte)this.port, (byte)((v ^ this.invert) ? 1 : 0) });
        }
        else {
            this.orc.doCommand(28672, new byte[] { (byte)(this.port - 8), 2, 0, 0, 0, (byte)((v ^ this.invert) ? 1 : 0) });
        }
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
