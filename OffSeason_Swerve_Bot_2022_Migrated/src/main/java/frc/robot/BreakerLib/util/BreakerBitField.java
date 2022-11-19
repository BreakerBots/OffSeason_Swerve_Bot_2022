// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util;

import java.util.ArrayList;
import java.util.Arrays;

/** A class that represents a modifiable bit field for user convienance */
public class BreakerBitField {
    public boolean[] bools;

    public BreakerBitField(boolean... bools) {
        this.bools = Arrays.copyOf(bools, bools.length);
    }

    public BreakerBitField(Boolean... bools) {
        this.bools = new boolean[bools.length];
        for (int i = 0; i < bools.length; i++) {
            this.bools[i] = bools[i];
        }
    }

    public BreakerBitField(long bitField) {
        bools = new boolean[64];
        long bitMask = 1;
        for (int i = 0; i < bools.length; i++) {
            bools[i] = (bitField & bitMask) != 0;
            bitMask <<= 1;
        }
    }

    public BreakerBitField(int bitField) {
        bools = new boolean[32];
        int bitMask = 1;
        for (int i = 0; i < bools.length; i++) {
            bools[i] = (bitField & bitMask) != 0;
            bitMask <<= 1;
        }
    }

    public BreakerBitField(short bitField) {
        bools = new boolean[16];
        int bitMask = 1;
        for (int i = 0; i < bools.length; i++) {
            bools[i] = (bitField & bitMask) != 0;
            bitMask <<= 1;
        }
    }

    public BreakerBitField(byte bitField) {
        bools = new boolean[8];
        int bitMask = 1;
        for (int i = 0; i < bools.length; i++) {
            bools[i] = (bitField & bitMask) != 0;
            bitMask <<= 1;
        }
    }

    public BreakerBitField(boolean bit) {
        bools = new boolean[]{bit};
    }

    public BreakerBitField and(BreakerBitField outher) {
        int minLen = Math.min(outher.bools.length, bools.length);
        int maxLen = Math.max(outher.bools.length, bools.length);
        boolean[] newBools = new boolean[maxLen];
        for (int i = 0; i < maxLen; i++) {
            if (i < minLen) {
                newBools[i] = (bools[i] & outher.bools[i]);
            } else {
                newBools[i] = false;
            }  
        }
        return new BreakerBitField(newBools);
    }

    public BreakerBitField or(BreakerBitField outher) {
        int minLen = Math.min(outher.bools.length, bools.length);
        int maxLen = Math.max(outher.bools.length, bools.length);
        boolean[] newBools = new boolean[maxLen];
        for (int i = 0; i < maxLen; i++) {
            if (i < minLen) {
                newBools[i] = (bools[i] | outher.bools[i]);
            } else {
                if (bools.length > outher.bools.length) {
                    newBools[i] = bools[i];
                } else {
                    newBools[i] = outher.bools[i];
                }
            }  
        }
        return new BreakerBitField(newBools);
    }

    public BreakerBitField xor(BreakerBitField outher) {
        int minLen = Math.min(outher.bools.length, bools.length);
        int maxLen = Math.max(outher.bools.length, bools.length);
        boolean[] newBools = new boolean[maxLen];
        for (int i = 0; i < maxLen; i++) {
            if (i < minLen) {
                newBools[i] = (bools[i] ^ outher.bools[i]);
            } else {
                if (bools.length > outher.bools.length) {
                    newBools[i] = bools[i];
                } else {
                    newBools[i] = outher.bools[i];
                }
            }  
        }
        return new BreakerBitField(newBools);
    }

    public boolean get(int index) {
        return bools[index];
    }

    public BreakerBitField truncate(int startIndex, int endIndex) {
        ArrayList<Boolean> list = new ArrayList<>();
        for (int i = startIndex; i <= endIndex && i < bools.length; i++) {
            list.add(bools[i]);
        }
        return new BreakerBitField(list.toArray(new Boolean[list.size()]));
    }

    public boolean[] getBaseBooleanArray() {
        return Arrays.copyOf(bools, bools.length);
    }
}
