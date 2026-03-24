package frc.robot;

import java.nio.ByteBuffer;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public class FuelStruct implements StructSerializable {
    public float x;
    public float y;

    public static final FuelStructDef struct = new FuelStructDef();

    public FuelStruct(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public FuelStruct() {}

    public static class FuelStructDef implements Struct<FuelStruct> {
        @Override public Class<FuelStruct> getTypeClass() { return FuelStruct.class; }
        @Override public String getTypeString() { return "struct:Fuel"; }
        @Override public int getSize()    { return Float.BYTES * 2; }
        @Override public String getSchema() { return "float x;float y;"; }
        @Override public String getTypeName() { return "Fuel"; }

        @Override
        public FuelStruct unpack(ByteBuffer bb) {
            FuelStruct f = new FuelStruct();
            f.x = bb.getFloat();
            f.y = bb.getFloat();
            return f;
        }

        @Override
        public void pack(ByteBuffer bb, FuelStruct val) {
            bb.putFloat(val.x);
            bb.putFloat(val.y);
        }
    }
}