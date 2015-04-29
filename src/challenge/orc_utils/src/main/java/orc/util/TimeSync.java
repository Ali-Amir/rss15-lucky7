package orc.util;

public class TimeSync
{
    long last_device_ticks_wrapping;
    long device_ticks_offset;
    long device_ticks_wrap;
    double device_ticks_per_second;
    double rate_error;
    double reset_time;
    long p_ticks;
    long q_ticks;
    public double last_sync_error;
    public int resync_count;
    
    public TimeSync(final double device_ticks_per_second, final long device_ticks_wrap, final double rate_error, final double reset_time) {
        super();
        this.p_ticks = -1L;
        this.device_ticks_per_second = device_ticks_per_second;
        this.device_ticks_wrap = device_ticks_wrap;
        this.rate_error = rate_error;
        this.reset_time = reset_time;
    }
    
    public void update(final long host_utime, final long device_ticks_wrapping) {
        assert device_ticks_wrapping >= 0L;
        if (device_ticks_wrapping < this.last_device_ticks_wrapping) {
            this.device_ticks_offset += this.device_ticks_wrap;
        }
        this.last_device_ticks_wrapping = device_ticks_wrapping;
        final long pi_ticks;
        final long device_ticks = pi_ticks = this.device_ticks_offset + device_ticks_wrapping;
        final double dp = (pi_ticks - this.p_ticks) / this.device_ticks_per_second;
        final double dq = (host_utime - this.q_ticks) / 1000000.0;
        this.last_sync_error = Math.abs(dp - dq);
        if (this.p_ticks == -1L || this.last_sync_error >= this.reset_time) {
            this.p_ticks = pi_ticks;
            this.q_ticks = host_utime;
            ++this.resync_count;
            return;
        }
        if (dp >= dq - Math.abs(this.rate_error * dp)) {
            this.p_ticks = pi_ticks;
            this.q_ticks = host_utime;
        }
    }
    
    public long getHostUtime(final long device_ticks_wrapping) {
        assert this.p_ticks != -1L;
        assert device_ticks_wrapping >= 0L;
        long device_ticks;
        if (device_ticks_wrapping <= this.last_device_ticks_wrapping) {
            device_ticks = this.device_ticks_offset + device_ticks_wrapping;
        }
        else {
            device_ticks = this.device_ticks_offset + device_ticks_wrapping - this.device_ticks_wrap;
        }
        final long pi_ticks = device_ticks;
        final double dp = (pi_ticks - this.p_ticks) / this.device_ticks_per_second;
        return (long)(dp * 1000000.0) + this.q_ticks + (long)(1000000.0 * Math.abs(this.rate_error * dp));
    }
}
