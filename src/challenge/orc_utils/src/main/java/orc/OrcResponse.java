package orc;

import java.io.*;

public final class OrcResponse
{
    public boolean responded;
    public int transactionId;
    public long utimeOrc;
    public long utimeHost;
    public int responseId;
    public DataInputStream ins;
    public byte[] responseBuffer;
    public int responseBufferOffset;
    public int responseBufferLength;
    
    public OrcResponse() {
        super();
        this.responded = false;
    }
    
    public synchronized void gotResponse() {
        this.responded = true;
        this.notifyAll();
    }
    
    public synchronized boolean waitForResponse(final int timeoutms) {
        if (this.responded) {
            return true;
        }
        try {
            this.wait(timeoutms);
        }
        catch (InterruptedException ex) {}
        return this.responded;
    }
    
    public void print() {
        for (int i = 0; i < this.responseBufferLength; ++i) {
            if (i % 16 == 0) {
                System.out.printf("%04x: ", i);
            }
            System.out.printf("%02x ", this.responseBuffer[this.responseBufferOffset + i]);
            if (i % 16 == 15 || i == this.responseBufferLength - 1) {
                System.out.printf("\n", new Object[0]);
            }
        }
    }
}
