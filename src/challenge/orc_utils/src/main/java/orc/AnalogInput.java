package orc;

public class AnalogInput
{
    Orc orc;
    int port;
    
    public AnalogInput(final Orc orc, final int port) {
        super();
        assert port >= 0 && port <= 7;
        this.orc = orc;
        this.port = port;
    }
    
    public double getVoltage() {
        return getVoltage(this.orc.getStatus());
    }

    public double getVoltage(final OrcStatus status) {
        return status.analogInputFiltered[this.port] / 65535.0 * 5.0;
    }
    
    public double getVoltageUnfiltered() {
        return getVoltageUnfiltered(this.orc.getStatus());
    }

    public double getVoltageUnfiltered(final OrcStatus status) {
        return status.analogInput[this.port] / 65535.0 * 5.0;
    }
    
    public void setLPF(final double alpha) {
        assert alpha >= 0.0 && alpha <= 1.0;
        final int v = (int)(alpha * 65536.0);
        this.orc.doCommand(12288, new byte[] { (byte)this.port, (byte)(v >> 8 & 0xFF), (byte)(v >> 0 & 0xFF) });
    }
}
