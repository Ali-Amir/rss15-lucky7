package orc;

public class Gyro
{
    Orc orc;
    int port;
    double SAMPLE_HZ;
    double v0;
    double mvPerDegPerSec;
    double radPerSecPerLSB;
    long integratorOffset;
    int integratorCountOffset;
    boolean calibrated;
    double voltsPerLSB;
    double theta;
    
    public Gyro(final Orc orc, final int port) {
        this(orc, port, 0.005);
    }
    
    public Gyro(final Orc orc, final int port, final double voltsPerDegPerSec) {
        super();
        this.radPerSecPerLSB = 5.32638E-5;
        this.voltsPerLSB = 6.103515625E-5;
        this.theta = 0.0;
        assert port >= 0 && port <= 2;
        this.orc = orc;
        this.port = port;
        final double lsbPerDegPerSec = 1.0 / this.voltsPerLSB * voltsPerDegPerSec;
        final double degPerSecPerLSB = 1.0 / lsbPerDegPerSec;
        this.radPerSecPerLSB = Math.toRadians(degPerSecPerLSB);
        this.reset();
    }
    
    public void reset() {
        final OrcStatus status = this.orc.getStatus();
        this.integratorOffset = status.gyroIntegrator[this.port];
        this.integratorCountOffset = status.gyroIntegratorCount[this.port];
        this.theta = 0.0;
    }
    
    public synchronized double getTheta() {
        if (!this.calibrated) {
            System.out.println("orc.Gyro: Must calibrate before calling getTheta!()");
            this.calibrated = true;
            return this.theta;
        }
        final OrcStatus s = this.orc.getStatus();
        final double integrator = s.gyroIntegrator[this.port] - this.integratorOffset;
        final double integratorCount = s.gyroIntegratorCount[this.port] - this.integratorCountOffset;
        if (integratorCount == 0.0) {
            return this.theta;
        }
        final double dt = integratorCount / this.SAMPLE_HZ;
        final double averageIntegrator = integrator / integratorCount - this.v0;
        this.theta += averageIntegrator * dt * this.radPerSecPerLSB;
        this.integratorOffset = s.gyroIntegrator[this.port];
        this.integratorCountOffset = s.gyroIntegratorCount[this.port];
        return this.theta;
    }
    
    public void calibrate(final double seconds) {
        final OrcStatus s0 = this.orc.getStatus();
        try {
            Thread.sleep((int)(seconds * 1000.0));
        }
        catch (InterruptedException ex) {}
        final OrcStatus s = this.orc.getStatus();
        final double dt = (s.utimeOrc - s0.utimeOrc) / 1000000.0;
        final double dv = s.gyroIntegrator[this.port] - s0.gyroIntegrator[this.port];
        final double ds = s.gyroIntegratorCount[this.port] - s0.gyroIntegratorCount[this.port];
        this.SAMPLE_HZ = ds / dt;
        this.v0 = dv / ds;
        System.out.printf("Requested calib t: %15f seconds\n", seconds);
        System.out.printf("Actual calib t:    %15f seconds\n", dt);
        System.out.printf("Integrator change: %15.1f ADC LSBs\n", dv);
        System.out.printf("Integrator counts: %15.1f counts\n", ds);
        System.out.printf("Sample rate:       %15f Hz\n", this.SAMPLE_HZ);
        System.out.printf("calibrated at:     %15f ADC LSBs (about %f V)\n", this.v0, this.v0 / 65535.0 * 5.0);
        this.calibrated = true;
    }
    
    public static void main(final String[] args) {
        final int port = 0;
        final Orc orc = Orc.makeOrc();
        final Gyro gyro = new Gyro(orc, port);
        final AnalogInput ain = new AnalogInput(orc, port);
        final double calibrateTime = 3.0;
        System.out.println("Calibrating for " + calibrateTime + " seconds...");
        gyro.calibrate(calibrateTime);
        final double starttime = System.currentTimeMillis() / 1000.0;
        while (true) {
            final double rad = gyro.getTheta();
            final double dt = System.currentTimeMillis() / 1000.0 - starttime;
            System.out.printf("\r t=%15f V=%15f theta=%15f rad (%15f deg)", dt, ain.getVoltage(), rad, Math.toDegrees(rad));
            try {
                Thread.sleep(100L);
            }
            catch (InterruptedException ex) {}
        }
    }
}
