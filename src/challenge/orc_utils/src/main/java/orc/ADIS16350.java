package orc;

public class ADIS16350
{
    Orc orc;
    int maxclk;
    int spo;
    int sph;
    int nbits;
    
    public ADIS16350(final Orc orc) {
        super();
        this.maxclk = 100000;
        this.spo = 1;
        this.sph = 1;
        this.nbits = 16;
        this.orc = orc;
    }
    
    int readRegister(final int addr) {
        int[] v = this.orc.spiTransaction(this.maxclk, this.spo, this.sph, this.nbits, new int[] { addr << 8 });
        v = this.orc.spiTransaction(this.maxclk, this.spo, this.sph, this.nbits, new int[] { addr << 8 });
        return v[v.length - 1];
    }
    
    static int convert14(int v) {
        v &= 0x3FFF;
        if ((v & 0x2000) > 0) {
            v |= 0xFFFFC000;
        }
        return v;
    }
    
    static int convert12(int v) {
        v &= 0xFFF;
        if ((v & 0x800) > 0) {
            v |= 0xFFFFF000;
        }
        return v;
    }
    
    void writeRegister(final int addr) {
        final int[] v = this.orc.spiTransaction(this.maxclk, this.spo, this.sph, this.nbits, new int[] { (addr | 0x80) << 8, 0 });
    }
    
    public double[] readState() {
        final double[] x = { convert12(this.readRegister(2)) * 0.0018315, Math.toRadians(convert14(this.readRegister(4)) * 0.07326), Math.toRadians(convert14(this.readRegister(6)) * 0.07326), Math.toRadians(convert14(this.readRegister(8)) * 0.07326), convert14(this.readRegister(10)) * 0.002522, convert14(this.readRegister(12)) * 0.002522, convert14(this.readRegister(14)) * 0.002522, convert12(this.readRegister(16)) * 0.1453 + 25.0, convert12(this.readRegister(18)) * 0.1453 + 25.0, convert12(this.readRegister(20)) * 0.1453 + 25.0, convert12(this.readRegister(22)) * 6.105E-4 };
        return x;
    }
    
    public static void main(final String[] args) {
        final Orc orc = Orc.makeOrc();
        final ADIS16350 adis = new ADIS16350(orc);
        final double zpos = 0.0;
        final long lastTime = System.currentTimeMillis();
        while (true) {
            System.out.printf("%20.5f ", System.currentTimeMillis() / 1000.0);
            final double[] state = adis.readState();
            for (int i = 0; i < state.length; ++i) {
                System.out.printf("%15f", state[i]);
            }
            System.out.printf("\n", new Object[0]);
            try {
                Thread.sleep(5L);
            }
            catch (InterruptedException ex) {}
        }
    }
}
