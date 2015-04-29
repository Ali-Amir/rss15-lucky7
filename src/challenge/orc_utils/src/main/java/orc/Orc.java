package orc;

import java.util.*;
import orc.util.*;
import java.net.*;
import java.io.*;

public class Orc
{
    DatagramSocket sock;
    ReaderThread reader;
    InetAddress orcAddr;
    int nextTransactionId;
    static final int ORC_BASE_PORT = 2378;
    static final double MIN_TIMEOUT = 0.002;
    static final double MAX_TIMEOUT = 0.01;
    public static final int FAST_DIGIO_MODE_IN = 1;
    public static final int FAST_DIGIO_MODE_OUT = 2;
    public static final int FAST_DIGIO_MODE_SERVO = 3;
    public static final int FAST_DIGIO_MODE_SLOW_PWM = 4;
    double meanRTT;
    public boolean verbose;
    HashMap<Integer, OrcResponse> transactionResponses;
    ArrayList<OrcListener> listeners;
    TimeSync ts;
    
    public static void main(final String[] args) {
        final Orc orc = makeOrc();
        System.out.println("Version: " + orc.getVersion());
        System.out.println("Benchmarking...");
        final long startmtime = System.currentTimeMillis();
        final int niters = 1000;
        for (int i = 0; i < niters; ++i) {
            orc.getVersion();
        }
        final long endmtime = System.currentTimeMillis();
        final double iterspersec = niters / ((endmtime - startmtime) / 1000.0);
        System.out.printf("Iterations per second: %.1f\n", iterspersec);
    }
    
    public InetAddress getAddress() {
        return this.orcAddr;
    }
    
    public static Orc makeOrc() {
        return makeOrc("192.168.237.7");
    }
    
    public long toHostUtime(final long uorcUtime) {
        return this.ts.getHostUtime(uorcUtime);
    }
    
    public String getVersion() {
        final OrcResponse resp = this.doCommand(2, null);
        final StringBuffer sb = new StringBuffer();
        try {
            while (resp.ins.available() > 0) {
                final byte b = (byte)resp.ins.read();
                if (b == 0) {
                    break;
                }
                sb.append((char)b);
            }
        }
        catch (IOException ex) {
            System.out.println("ex: " + ex);
        }
        return sb.toString();
    }
    
    public static Orc makeOrc(final String hostname) {
        // Loop until connected
        while (true) {
            try {
                return new Orc(InetAddress.getByName(hostname));
            }
            catch (IOException ex) {
                System.out.println("Exception creating Orc: " + ex);
                try {
                    Thread.sleep(500L);
                }
                catch (InterruptedException ex2) {}
            }
        }
    }
    
    public Orc(final InetAddress inetaddr) throws IOException {
        super();
        this.meanRTT = 0.002;
        this.verbose = false;
        this.transactionResponses = new HashMap<Integer, OrcResponse>();
        this.listeners = new ArrayList<OrcListener>();
        this.ts = new TimeSync(1000000.0, 0L, 0.001, 0.5);
        this.orcAddr = inetaddr;
        this.sock = new DatagramSocket();
        (this.reader = new ReaderThread()).setDaemon(true);
        this.reader.start();
        final String version = this.getVersion();
        System.out.println("Connected to uorc with firmware " + version);
        if (!version.startsWith("v")) {
            System.out.println("Unrecognized firmware signature.");
        }
        int vers = 0;
        for (int idx = 1; idx < version.length() && Character.isDigit(version.charAt(idx)); ++idx) {
            final char c = version.charAt(idx);
            if (c == '-') {
                break;
            }
            vers = vers * 10 + Character.digit(c, 10);
        }
        if (vers < 1) {
            System.out.println("Your firmware is too old.");
        }
    }
    
    public void addListener(final OrcListener ol) {
        this.listeners.add(ol);
    }
    
    public OrcResponse doCommand(final int commandId, final byte[] payload) {
        while (true) {
            try {
                return this.doCommandEx(commandId, payload);
            }
            catch (IOException ex) {
                System.out.println("ERR: Orc ex: " + ex);
            }
        }
    }
    
    public OrcResponse doCommandEx(final int commandId, final byte[] payload) throws IOException {
        final ByteArrayOutputStream bouts = new ByteArrayOutputStream();
        final DataOutputStream outs = new DataOutputStream(bouts);
        final OrcResponse response = new OrcResponse();
        outs.writeInt(216858626);
        final int transactionId;
        synchronized (this) {
            transactionId = this.nextTransactionId++;
            this.transactionResponses.put(transactionId, response);
        }
        outs.writeInt(transactionId);
        outs.writeLong(System.nanoTime() / 1000L);
        outs.writeInt(commandId);
        if (payload != null) {
            outs.write(payload);
        }
        final byte[] p = bouts.toByteArray();
        final DatagramPacket packet = new DatagramPacket(p, p.length, this.orcAddr, 2378 + (commandId >> 24 & 0xFF));
        try {
            boolean okay;
            do {
                final long starttime = System.nanoTime();
                this.sock.send(packet);
                okay = response.waitForResponse(50 + (int)(10.0 * this.meanRTT));
                if (!okay && this.verbose) {
                    System.out.printf("Transaction timeout: xid=%8d, command=%08x, timeout=%.4f\n", transactionId, commandId, this.meanRTT);
                }
                final long endtime = System.nanoTime();
                final double rtt = (endtime - starttime) / 1.0E9;
                final double alpha = 0.995;
                this.meanRTT = alpha * this.meanRTT + (1.0 - alpha) * rtt;
                this.meanRTT = Math.min(Math.max(this.meanRTT, 0.002), 0.01);
            } while (!okay);
            return response;
        }
        catch (IOException ex) {
            throw ex;
        }
        finally {
            this.transactionResponses.remove(transactionId);
        }
    }
    
    public OrcStatus getStatus() {
        while (true) {
            try {
                return new OrcStatus(this, this.doCommand(1, null));
            }
            catch (IOException ex) {}
        }
    }
    
    public byte[] i2cTransaction(final int addr, final Object... os) {
        final ByteArrayOutputStream bouts = new ByteArrayOutputStream();
        bouts.write((byte)addr);
        bouts.write(1);
        assert (os.length & 0x1) == 0x0;
        assert os.length >= 2;
        final int ntransactions = os.length / 2;
        for (int transaction = 0; transaction < ntransactions; ++transaction) {
            final byte[] writebuf = (byte[])os[2 * transaction + 0];
            final int writebuflen = (writebuf == null) ? 0 : writebuf.length;
            final int readlen = (Integer)os[2 * transaction + 1];
            bouts.write((byte)writebuflen);
            bouts.write((byte)readlen);
            for (int i = 0; i < writebuflen; ++i) {
                bouts.write(writebuf[i]);
            }
        }
        final OrcResponse resp = this.doCommand(20480, bouts.toByteArray());
        assert resp.responded;
        final ByteArrayOutputStream readData = new ByteArrayOutputStream();
        try {
            for (int transaction2 = 0; transaction2 < ntransactions; ++transaction2) {
                final int error = resp.ins.readByte() & 0xFF;
                if (error != 0) {
                    System.out.printf("Orc I2C error: code = %d\n", error);
                }
                for (int actualreadlen = resp.ins.readByte() & 0xFF, j = 0; j < actualreadlen; ++j) {
                    readData.write(resp.ins.readByte());
                }
            }
            return readData.toByteArray();
        }
        catch (IOException ex) {
            return null;
        }
    }
    
    public int[] spiTransaction(int slaveClk, final int spo, final int sph, final int nbits, final int[] writebuf) {
        slaveClk /= 1000;
        assert nbits <= 16;
        assert spo == 1;
        assert sph == 1;
        assert writebuf.length <= 16;
        final ByteArrayOutputStream bouts = new ByteArrayOutputStream();
        bouts.write(slaveClk >> 8 & 0xFF);
        bouts.write(slaveClk & 0xFF);
        bouts.write(nbits | spo << 6 | sph << 7);
        bouts.write(writebuf.length);
        for (int i = 0; i < writebuf.length; ++i) {
            bouts.write(writebuf[i] >> 8 & 0xFF);
            bouts.write(writebuf[i] & 0xFF);
        }
        final OrcResponse resp = this.doCommand(16384, bouts.toByteArray());
        assert resp.responded;
        int[] rx = null;
        try {
            final int status = resp.ins.readByte();
            assert status == 0;
            final int nwords = resp.ins.readByte() & 0xFF;
            rx = new int[nwords];
            for (int j = 0; j < nwords; ++j) {
                rx[j] = (resp.ins.readShort() & 0xFFFF);
            }
        }
        catch (IOException ex) {
            return null;
        }
        return rx;
    }
    
    class ReaderThread extends Thread
    {
        public void run() {
            while (true) {
                final byte[] packetBuffer = new byte[1600];
                final DatagramPacket packet = new DatagramPacket(packetBuffer, packetBuffer.length);
                try {
                    Orc.this.sock.receive(packet);
                    final DataInputStream ins = new DataInputStream(new ByteArrayInputStream(packetBuffer, 0, packet.getLength()));
                    final int signature = ins.readInt();
                    if (signature != 216858625) {
                        System.out.println("bad signature");
                    }
                    else {
                        final int transId = ins.readInt();
                        final long utimeOrc = ins.readLong();
                        final int responseId = ins.readInt();
                        Orc.this.ts.update(System.currentTimeMillis() * 1000L, utimeOrc);
                        final long utimeHost = Orc.this.toHostUtime(utimeOrc);
                        final OrcResponse sig;
                        synchronized (Orc.this) {
                            sig = Orc.this.transactionResponses.remove(transId);
                        }
                        if (sig != null) {
                            sig.ins = ins;
                            sig.responseBuffer = packetBuffer;
                            sig.responseBufferOffset = 20;
                            sig.responseBufferLength = packet.getLength();
                            sig.utimeOrc = utimeOrc;
                            sig.utimeHost = utimeHost;
                            sig.responseId = responseId;
                            sig.gotResponse();
                        }
                        else {
                            if (!Orc.this.verbose) {
                                continue;
                            }
                            System.out.println("Unexpected reply for transId: " + transId + " (last issued: " + (Orc.this.nextTransactionId - 1) + ")");
                        }
                    }
                }
                catch (IOException ex) {
                    System.out.println("Orc.ReaderThread Ex: " + ex);
                    try {
                        Thread.sleep(100L);
                    }
                    catch (InterruptedException ex2) {}
                }
            }
        }
    }

    public boolean isSim() {
        return false;
    }

    public void setNode(Object obj) {}
}
