package orc;

public class QuadratureEncoder
{
    Orc orc;
    int port;
    boolean invert;
    static final int QEI_VELOCITY_SAMPLE_HZ = 40;
    
    public QuadratureEncoder(final Orc orc, final int port, final boolean invert) {
        super();
        assert port >= 0 && port <= 1;
        this.orc = orc;
        this.port = port;
        this.invert = invert;
    }
    
    public int getPosition() {
        return this.getPosition(this.orc.getStatus());
    }
    
    public int getPosition(final OrcStatus status) {
        assert status.orc == this.orc;
        return status.qeiPosition[this.port] * (this.invert ? -1 : 1);
    }
    
    public double getVelocity() {
        return this.getVelocity(this.orc.getStatus());
    }
    
    public double getVelocity(final OrcStatus status) {
        assert status.orc == this.orc;
        return status.qeiVelocity[this.port] * (this.invert ? -1 : 1) * 40;
    }
}
