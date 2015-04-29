package orc;

public class AX12Status
{
    public static final int ERROR_INSTRUCTION = 64;
    public static final int ERROR_OVERLOAD = 32;
    public static final int ERROR_CHECKSUM = 16;
    public static final int ERROR_RANGE = 8;
    public static final int ERROR_OVERHEAT = 4;
    public static final int ERROR_ANGLE_LIMIT = 2;
    public static final int ERROR_VOLTAGE = 1;
    public long utimeOrc;
    public long utimeHost;
    public double positionDegrees;
    public double speed;
    public double load;
    public double voltage;
    public double temperature;
    public int error_flags;
    
    public void print() {
        System.out.printf("position:    %f\n", this.positionDegrees);
        System.out.printf("speed:       %f\n", this.speed);
        System.out.printf("load:         %f\n", this.load);
        System.out.printf("voltage:     %f\n", this.voltage);
        System.out.printf("temperature: %f\n", this.temperature);
        System.out.printf("error flags:  %d\n", this.error_flags);
        System.out.printf("utime (orc):  %d\n", this.utimeOrc);
        System.out.printf("utime (host): %d\n", this.utimeHost);
    }
}
