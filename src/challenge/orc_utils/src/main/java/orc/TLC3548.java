package orc;

public class TLC3548
{
    Orc orc;
    int maxclk;
    int spo;
    int sph;
    int nbits;
    boolean shortSampling;
    boolean externalReference;
    
    public TLC3548(final Orc orc) {
        super();
        this.maxclk = 2500000;
        this.spo = 0;
        this.sph = 1;
        this.nbits = 16;
        this.shortSampling = false;
        this.externalReference = true;
        (this.orc = orc).spiTransaction(this.maxclk, this.spo, this.sph, this.nbits, new int[] { 40960 });
        int flag = 0;
        if (this.externalReference) {
            flag |= 0x800;
        }
        if (this.shortSampling) {
            flag |= 0x200;
        }
        orc.spiTransaction(this.maxclk, this.spo, this.sph, this.nbits, new int[] { 0xA000 | flag });
    }
    
    public int beginConversion(final int port) {
        final int[] rx = this.orc.spiTransaction(this.maxclk, this.spo, this.sph, this.nbits, new int[] { port << 12, 0, 0, 0, 0, 0, 0, 0, 0, 0 });
        return rx[0];
    }
    
    public static void main(final String[] args) {
        final Orc orc = Orc.makeOrc();
        final TLC3548 tlc = new TLC3548(orc);
        while (true) {
            for (int i = 0; i < 8; ++i) {
                System.out.printf("%04x  ", tlc.beginConversion(i));
            }
            System.out.printf("\n", new Object[0]);
            try {
                Thread.sleep(20L);
            }
            catch (InterruptedException ex) {}
        }
    }
}
