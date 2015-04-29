package uORCInterface;

public interface OrcControlInterface
{
    void motorSlewWrite(int p0, int p1);
    
    void motorSet(int p0, int p1);
    
    void servoWrite(int p0, int p1);
    
    long clockReadSlave();
    
    int readEncoder(int p0);
    
    int readVelocity(int p0);
    
    double analogRead(int p0);
    
    void digitalSet(int p0, boolean p1);
    
    boolean digitalRead(int p0);
}
