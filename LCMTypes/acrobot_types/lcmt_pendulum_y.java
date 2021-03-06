/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package acrobot_types;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class lcmt_pendulum_y implements lcm.lcm.LCMEncodable
{
    public long timestamp;
    public double theta;
    public double tau;
 
    public lcmt_pendulum_y()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x82916ff66ab3b5beL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(acrobot_types.lcmt_pendulum_y.class))
            return 0L;
 
        classes.add(acrobot_types.lcmt_pendulum_y.class);
        long hash = LCM_FINGERPRINT_BASE
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        outs.writeLong(this.timestamp); 
 
        outs.writeDouble(this.theta); 
 
        outs.writeDouble(this.tau); 
 
    }
 
    public lcmt_pendulum_y(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public lcmt_pendulum_y(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static acrobot_types.lcmt_pendulum_y _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        acrobot_types.lcmt_pendulum_y o = new acrobot_types.lcmt_pendulum_y();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.timestamp = ins.readLong();
 
        this.theta = ins.readDouble();
 
        this.tau = ins.readDouble();
 
    }
 
    public acrobot_types.lcmt_pendulum_y copy()
    {
        acrobot_types.lcmt_pendulum_y outobj = new acrobot_types.lcmt_pendulum_y();
        outobj.timestamp = this.timestamp;
 
        outobj.theta = this.theta;
 
        outobj.tau = this.tau;
 
        return outobj;
    }
 
}

