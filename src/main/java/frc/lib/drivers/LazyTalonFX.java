/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.drivers;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

public class LazyTalonFX extends TalonFX {

    protected double mLastSet = Double.NaN;
    // Control mode no longer uses enums.
    //protected TalonFXControlMode mLastControlMode = null;

    private String mName;

    protected LazyTalonFX mLeader = null;
    protected Slot0Configs mConfigs;

    public LazyTalonFX(String name, int deviceID) {
        super(deviceID);
        mName = name;
        mConfigs = new Slot0Configs();
        super.getConfigurator().apply(mConfigs);
    }

    public double getLastSet() {
        return mLastSet;
    }

    public void setName(String name) {
        mName = name;
    }

    public String getName() {
        return mName;
    }

    public LazyTalonFX getLeader() {
        return mLeader;
    }

    public void setLeader(final LazyTalonFX leader) {
        mLeader = leader;
        super.setControl(new Follower(leader.getDeviceID(), false));
    }

    public void config_kP(double p, double timeout) {
        mConfigs.kP = p;

        super.getConfigurator().refresh(mConfigs);
    }

    public void config_kI(double i, double timeout) {
        mConfigs.kI = i;

        super.getConfigurator().refresh(mConfigs);
    }

    public void config_kD(double d, double timeout) {
        mConfigs.kD = d;

        super.getConfigurator().refresh(mConfigs);
    }

    public void config_kV(double v, double timeout) {
        mConfigs.kV = v;

        super.getConfigurator().refresh(mConfigs);
    }

    public void clearLeader() {
        mLeader = null;
        super.setControl(_emptyControl);
    }

    @Override
    public void set(double value) {
        if(value != mLastSet) {
            mLastSet = value;
            super.set(value);
        }
    }

    @Override
    public String toString() {
        return getName() + " -> Output Power: " + mLastSet; 
    }

}