package orc;

import java.io.*;

public class Motor
{
    Orc orc;
    int port;
    boolean invert;
    
    public Motor(final Orc orc, final int port, final boolean invert) {
        super();
        this.orc = orc;
        this.port = port;
        this.invert = invert;
    }
    
    public void idle() {
        this.orc.doCommand(4096, new byte[] { (byte)this.port, 0, 0, 0 });
    }
    
    public void setPWM(final double v) {
        final int pwm = this.mapPWM(v);
        this.orc.doCommand(4096, new byte[] { (byte)this.port, 1, (byte)(pwm >> 8 & 0xFF), (byte)(pwm & 0xFF) });
    }
    
    public double getPWM(final OrcStatus status) {
        double v = status.motorPWMactual[this.port] / 255.0;
        if (this.invert) {
            v *= -1.0;
        }
        return v;
    }
    
    public double getPWM() {
        return this.getPWM(this.orc.getStatus());
    }
    
    protected int mapPWM(final double v) {
        assert v >= -1.0 && v <= 1.0;
        int pwm = (int)(v * 255.0);
        if (this.invert) {
            pwm *= -1;
        }
        return pwm;
    }
    
    public double getCurrent(final OrcStatus status) {
        final double voltage = status.analogInput[this.port + 8] / 65535.0 * 3.0;
        final double current = voltage * 375.0 / 200.0;
        return current;
    }
    
    public double getCurrent() {
        return this.getCurrent(this.orc.getStatus());
    }
    
    public double getCurrentFiltered(final OrcStatus status) {
        final double voltage = status.analogInputFiltered[this.port + 8] / 65535.0 * 3.0;
        final double current = voltage * 375.0 / 200.0;
        return current;
    }
    
    public double getCurrentFiltered() {
        return this.getCurrentFiltered(this.orc.getStatus());
    }
    
    public void setSlewSeconds(double seconds) {
        assert seconds >= 0.0 && seconds < 120.0;
        seconds = Math.max(seconds, 0.001);
        final double dv = 0.51 / seconds * 128.0;
        int iv = (int)dv;
        iv = Math.max(iv, 1);
        iv = Math.min(iv, 65535);
        this.orc.doCommand(4097, new byte[] { (byte)this.port, (byte)(iv >> 8 & 0xFF), (byte)(iv & 0xFF) });
    }
    
    public static void setMultiplePWM(final Motor[] ms, final double[] vs) {
        final ByteArrayOutputStream outs = new ByteArrayOutputStream();
        for (int i = 0; i < ms.length; ++i) {
            assert ms[i].orc == ms[0].orc;
            final Motor m = ms[i];
            outs.write((byte)m.port);
            outs.write(1);
            final int pwm = m.mapPWM(vs[i]);
            outs.write((byte)(pwm >> 8 & 0xFF));
            outs.write((byte)(pwm & 0xFF));
        }
        ms[0].orc.doCommand(4096, outs.toByteArray());
    }
    
    public boolean isFault() {
        return this.isFault(this.orc.getStatus());
    }
    
    public boolean isFault(final OrcStatus status) {
        assert status.orc == this.orc;
        return (status.simpleDigitalValues & 1 << 8 + this.port * 2) == 0x0;
    }
    
    public void clearFault() {
        this.idle();
    }
}
