package frc.robot.util;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.hardware.TalonFX;

public class KrakenLogger {
    private TalonFX motor;
    private int index;
    // private KrakenStats stats = new KrakenStats();

    // public class KrakenStats {
    //     private double motorVoltage = 0.0;
    //     private double supplyVoltage = 0.0;
    //     private double motorCurrent = 0.0;
    //     private double supplyCurrent = 0.0;
    //     private double torqueCurrent = 0.0;
    //     private double motorVelocity = 0.0;
    //     private double motorPosition = 0.0;
    //     private double acceleration = 0.0;
    //     private double temp = 0.0;
    // }

    public KrakenLogger(TalonFX motor, int index) {
        this.motor = motor;
        this.index = index;
    }

    @AutoLogOutput(key = "CTRE/Kraken{index}/motorVoltage")
    private double motorVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    @AutoLogOutput(key = "CTRE/Kraken{index}/supplyVoltage")
    private double supplyVoltage() {
        return motor.getSupplyVoltage().getValueAsDouble();
    }

    @AutoLogOutput(key = "CTRE/Kraken{index}/statorCurrent")
    private double motorCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    @AutoLogOutput(key = "CTRE/Kraken{index}/supplyCurrent")
    private double supplyCurrent() {
        return motor.getSupplyCurrent().getValueAsDouble();
    }

    // public void update() {
    //     stats.motorVoltage = motor.getMotorVoltage().getValueAsDouble();
    //     stats.supplyVoltage = motor.getSupplyVoltage().getValueAsDouble();
    //     stats.motorCurrent = motor.getStatorCurrent().getValueAsDouble();
    //     stats.supplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
    //     stats.acceleration = motor.getAcceleration().getValueAsDouble();
    //     stats.motorVelocity = motor.getVelocity().getValueAsDouble();
    //     stats.motorPosition = motor.getPosition().getValueAsDouble();
    //     stats.temp = motor.getDeviceTemp().getValueAsDouble();
    //     stats.torqueCurrent = motor.getTorqueCurrent().getValueAsDouble();
    // } 
}
