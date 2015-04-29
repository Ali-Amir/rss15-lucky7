package orc;

public class WiiMotionPlus
{
    Orc orc;
    static final int I2C_INIT_ADDRESS = 83;
    static final int I2C_ADDRESS = 82;
    
    public WiiMotionPlus(final Orc orc) {
        super();
        (this.orc = orc).i2cTransaction(83, new byte[] { -2, 4 }, 0);
        orc.i2cTransaction(82, new byte[] { 0 }, 0);
    }
    
    public int[] readAxes() {
        final byte[] resp = this.orc.i2cTransaction(82, new byte[] { 0 }, 6);
        final int[] data = { (resp[0] & 0xFF) + ((resp[3] & 0xFC) << 6), (resp[1] & 0xFF) + ((resp[4] & 0xFC) << 6), (resp[2] & 0xFF) + ((resp[5] & 0xFC) << 6) };
        return data;
    }
    
    public static void main(final String[] args) {
        final Orc orc = Orc.makeOrc();
        final WiiMotionPlus wmp = new WiiMotionPlus(orc);
        while (true) {
            final int[] axes = wmp.readAxes();
            System.out.printf("%10d %10d %10d\n", axes[0], axes[1], axes[2]);
            try {
                Thread.sleep(30L);
            }
            catch (InterruptedException ex) {}
        }
    }
}
