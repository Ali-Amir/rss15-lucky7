package orc;

public class DigitalInput
{
    Orc orc;
    int port;
    boolean invert;
    
    public DigitalInput(final Orc orc, final int port, final boolean pullup, final boolean invert) {
        super();
        this.orc = orc;
        this.port = port;
        this.invert = invert;
        if (port < 8) {
            orc.doCommand(24576, new byte[] { (byte)port, 1, (byte)(pullup ? 1 : 0) });
        }
        else {
            orc.doCommand(28672, new byte[] { (byte)(port - 8), 1, 0, 0, 0, 0 });
        }
    }
    
    public boolean getValue() {
        return getValue(this.orc.getStatus());
    }

    public boolean getValue(final OrcStatus status) {
        boolean v;
        if (this.port < 8) {
            v = ((status.simpleDigitalValues & 1 << this.port) != 0x0);
        }
        else {
            v = (status.fastDigitalConfig[this.port - 8] != 0);
        }
        return v ^ this.invert;
    }
}
