package orc;

public class SRF02
{
    Orc orc;
    int i2caddr;
    static final int DEFAULT_I2C_ADDR = 112;
    static double SOUND_METERS_PER_SEC;
    
    public SRF02(final Orc orc, final int i2caddr) {
        super();
        this.orc = orc;
        this.i2caddr = i2caddr;
    }
    
    public SRF02(final Orc orc) {
        this(orc, 112);
    }
    
    public void ping() {
        final byte[] resp = this.orc.i2cTransaction(this.i2caddr, new byte[] { 0, 82 }, 0);
    }
    
    public double readTime() {
        final byte[] resp = this.orc.i2cTransaction(this.i2caddr, new byte[] { 2 }, 4);
        final int usecs = ((resp[0] & 0xFF) << 8) + (resp[1] & 0xFF);
        final int minusecs = ((resp[2] & 0xFF) << 8) + (resp[3] & 0xFF);
        return usecs / 1000000.0;
    }
    
    public double readRange() {
        return this.readTime() * SRF02.SOUND_METERS_PER_SEC / 2.0;
    }
    
    public double measure() {
        synchronized(orc) {
            this.ping();
        }
        try {
            Thread.sleep(70L);
        }
        catch (InterruptedException ex) {}
        synchronized(orc) {
            return this.readRange();
        }
    }
    
    public static void main(final String[] args) {
        final Orc orc = Orc.makeOrc();
        final SRF02 srf = new SRF02(orc);
        while (true) {
            System.out.printf("%15f m\n", srf.measure());
        }
    }
    
    static {
        SRF02.SOUND_METERS_PER_SEC = 343.0;
    }
}
