package orc;

public class IRRangeFinder
{
    AnalogInput ain;
    double Xd;
    double Xm;
    double Xb;
    double arcAngle;
    double voltageStdDev;
    
    public IRRangeFinder(final Orc orc, final int port) {
        super();
        this.arcAngle = 0.08;
        this.voltageStdDev = 0.025;
        this.ain = new AnalogInput(orc, port);
    }
    
    public double[] getRangeAndUncertainty() {
        final double v = this.ain.getVoltage();
        double range = this.Xm / (v - this.Xb) + this.Xd;
        if (range < 0.0) {
            range = 0.0;
        }
        if (range > 100.0) {
            range = 100.0;
        }
        return new double[] { range, this.getRangeUncertainty(v) };
    }
    
    public double getRange() {
        return this.getRangeAndUncertainty()[0];
    }
    
    double getRangeUncertainty(final double v) {
        final double dddV = Math.abs(-this.Xm / (v - this.Xb) / (v - this.Xb));
        return this.voltageStdDev * dddV;
    }
    
    public void setParameters(final double Xd, final double Xm, final double Xb, final double voltageStdDev) {
        this.Xd = Xd;
        this.Xm = Xm;
        this.Xb = Xb;
        this.voltageStdDev = voltageStdDev;
    }
    
    public static IRRangeFinder make2Y0A02(final Orc orc, final int port) {
        final IRRangeFinder s = new IRRangeFinder(orc, port);
        s.Xd = -0.0618;
        s.Xm = 0.7389;
        s.Xb = -0.1141;
        return s;
    }
    
    public static IRRangeFinder makeGP2D12(final Orc orc, final int port) {
        final IRRangeFinder s = new IRRangeFinder(orc, port);
        s.Xd = 0.0828;
        s.Xm = 0.1384;
        s.Xb = 0.2448;
        return s;
    }
}
