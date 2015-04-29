package orc;

public class ADXL345
{
    Orc orc;
    static final int I2C_ADDRESS = 29;
    
    public ADXL345(final Orc orc) {
        super();
        this.orc = orc;
        final int deviceId = this.getDeviceId();
        assert deviceId == 229;
        orc.i2cTransaction(29, new byte[] { 45, 8 }, 0);
    }
    
    public int getDeviceId() {
        final byte[] res = this.orc.i2cTransaction(29, new byte[] { 0 }, 1);
        return res[0] & 0xFF;
    }
    
    public int[] readAxes() {
        final byte[] resp = this.orc.i2cTransaction(29, new byte[] { 50 }, 6);
        final int[] v = { (resp[0] & 0xFF) + (resp[1] << 8), (resp[2] & 0xFF) + (resp[3] << 8), (resp[4] & 0xFF) + (resp[5] << 8) };
        return v;
    }
    
    public static void main(final String[] args) {
        final Orc orc = Orc.makeOrc();
        final ADXL345 accel = new ADXL345(orc);
        while (true) {
            final int[] axes = accel.readAxes();
            System.out.printf("%10d %10d %10d\n", axes[0], axes[1], axes[2]);
            try {
                Thread.sleep(30L);
            }
            catch (InterruptedException ex) {}
        }
    }
}
