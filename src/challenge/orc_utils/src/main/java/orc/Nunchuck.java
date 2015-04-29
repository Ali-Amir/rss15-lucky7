package orc;

public class Nunchuck
{
    Orc orc;
    byte[] writeme2;
    byte[] readme2;
    byte addrss;
    byte writeleng;
    byte readleng;
    static final int I2C_ADDRESS = -92;
    
    public Nunchuck(final Orc orc) {
        super();
        this.writeme2 = new byte[1];
        this.readme2 = new byte[20];
        this.addrss = 82;
        this.orc = orc;
        this.writeme2[0] = 0;
        orc.i2cTransaction(-92, new byte[] { 64, 0 }, 0);
    }
    
    public int[] readState() {
        this.orc.i2cTransaction(-92, new byte[] { 0 }, 0);
        final byte[] resp = this.orc.i2cTransaction(-92, null, 6);
        final int[] state = { ((resp[0] & 0xFF) ^ 0x17) + 23, ((resp[1] & 0xFF) ^ 0x17) + 23, ((resp[2] & 0xFF) ^ 0x17) + 23, ((resp[3] & 0xFF) ^ 0x17) + 23, ((resp[4] & 0xFF) ^ 0x17) + 23, ((resp[5] & 0xFF) ^ 0x17) + 23 & 0x3 };
        return state;
    }
    
    public int[] readJoystick() {
        final int[] v = this.readState();
        return new int[] { v[0], v[1] };
    }
    
    public int[] readAccelerometers() {
        final int[] v = this.readState();
        return new int[] { v[2], v[3], v[4] };
    }
    
    public int readButtons() {
        final int[] v = this.readState();
        return v[5];
    }
}
