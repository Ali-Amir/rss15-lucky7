package orc;

public class Servo
{
    Orc orc;
    int port;
    double pos0;
    double pos1;
    int usec0;
    int usec1;
    
    public Servo(final Orc orc, final int port, final double pos0, final int usec0, final double pos1, final int usec1) {
        super();
        this.orc = orc;
        this.port = port;
        this.pos0 = pos0;
        this.usec0 = usec0;
        this.pos1 = pos1;
        this.usec1 = usec1;
    }
    
    public void setPulseWidth(final int usecs) {
        this.orc.doCommand(28672, new byte[] { (byte)this.port, 3, (byte)(usecs >> 24 & 0xFF), (byte)(usecs >> 16 & 0xFF), (byte)(usecs >> 8 & 0xFF), (byte)(usecs >> 0 & 0xFF) });
    }
    
    public void idle() {
        final int value = 0;
        this.orc.doCommand(28672, new byte[] { (byte)this.port, 2, (byte)(value >> 24 & 0xFF), (byte)(value >> 16 & 0xFF), (byte)(value >> 8 & 0xFF), (byte)(value >> 0 & 0xFF) });
    }
    
    public void setPosition(double pos) {
        if (pos < Math.min(this.pos0, this.pos1)) {
            pos = Math.min(this.pos0, this.pos1);
        }
        if (pos > Math.max(this.pos0, this.pos1)) {
            pos = Math.max(this.pos0, this.pos1);
        }
        this.setPulseWidth((int)(this.usec0 + (this.usec1 - this.usec0) * (pos - this.pos0) / (this.pos1 - this.pos0)));
    }
    
    public static Servo makeMPIMX400(final Orc orc, final int port) {
        return new Servo(orc, port, 0.0, 600, 3.141592653589793, 2500);
    }
}
