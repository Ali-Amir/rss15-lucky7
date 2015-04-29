package orc;

import java.io.*;

public class OrcStatus
{
    public Orc orc;
    public long utimeOrc;
    public long utimeHost;
    public int statusFlags;
    public int debugCharsWaiting;
    public int[] analogInput;
    public int[] analogInputFiltered;
    public int[] analogInputFilterAlpha;
    public int simpleDigitalValues;
    public int simpleDigitalDirections;
    public boolean[] motorEnable;
    public int[] motorPWMactual;
    public int[] motorPWMgoal;
    public int[] motorSlewRaw;
    public double[] motorSlewSeconds;
    public int[] qeiPosition;
    public int[] qeiVelocity;
    public int[] fastDigitalMode;
    public int[] fastDigitalConfig;
    public long[] gyroIntegrator;
    public int[] gyroIntegratorCount;
    
    static final int readU16(final DataInputStream ins) throws IOException {
        return ins.readShort() & 0xFFFF;
    }
    
    static final int readS16(final DataInputStream ins) throws IOException {
        return ins.readShort();
    }
    
    static final int readU8(final DataInputStream ins) throws IOException {
        return ins.readByte() & 0xFF;
    }
    
    public boolean getEstopState() {
        return (this.statusFlags & 0x1) > 0;
    }
    
    public boolean getMotorWatchdogState() {
        return (this.statusFlags & 0x2) > 0;
    }
    
    public double getBatteryVoltage() {
        final double voltage = this.analogInputFiltered[11] / 65535.0 * 3.0;
        final double batvoltage = voltage * 10.1;
        return batvoltage;
    }
    
    public OrcStatus(final Orc orc, final OrcResponse response) throws IOException {
        super();
        this.analogInput = new int[13];
        this.analogInputFiltered = new int[13];
        this.analogInputFilterAlpha = new int[13];
        this.motorEnable = new boolean[3];
        this.motorPWMactual = new int[3];
        this.motorPWMgoal = new int[3];
        this.motorSlewRaw = new int[3];
        this.motorSlewSeconds = new double[3];
        this.qeiPosition = new int[2];
        this.qeiVelocity = new int[2];
        this.fastDigitalMode = new int[8];
        this.fastDigitalConfig = new int[8];
        this.gyroIntegrator = new long[3];
        this.gyroIntegratorCount = new int[3];
        this.orc = orc;
        final DataInputStream ins = response.ins;
        this.utimeOrc = response.utimeOrc;
        this.utimeHost = response.utimeHost;
        this.statusFlags = ins.readInt();
        this.debugCharsWaiting = readU16(ins);
        for (int i = 0; i < 13; ++i) {
            this.analogInput[i] = readU16(ins);
            this.analogInputFiltered[i] = readU16(ins);
            this.analogInputFilterAlpha[i] = readU16(ins);
        }
        this.simpleDigitalValues = ins.readInt();
        this.simpleDigitalDirections = ins.readInt();
        for (int i = 0; i < 3; ++i) {
            this.motorEnable[i] = (readU8(ins) != 0);
            this.motorPWMactual[i] = readS16(ins);
            this.motorPWMgoal[i] = readS16(ins);
            this.motorSlewRaw[i] = readU16(ins);
            this.motorSlewSeconds[i] = 510.0 / this.motorSlewRaw[i] / 1000.0 * 128.0;
        }
        for (int i = 0; i < 2; ++i) {
            this.qeiPosition[i] = ins.readInt();
            this.qeiVelocity[i] = ins.readInt();
        }
        for (int i = 0; i < 8; ++i) {
            this.fastDigitalMode[i] = readU8(ins);
            this.fastDigitalConfig[i] = ins.readInt();
        }
        for (int i = 0; i < 3; ++i) {
            this.gyroIntegrator[i] = ins.readLong();
            this.gyroIntegratorCount[i] = ins.readInt();
        }
    }
}
