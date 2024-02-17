package frc.robot.util;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.hardware.TalonFX;

public class KrakenLogger {
    private TalonFX motor;
    private String name;

    public KrakenLogger(TalonFX motor, String name) {
        this.motor = motor;
        this.name = name;
    }

    @AutoLogOutput(key = "Krakens/{name}/motorVoltage")
    private double motorVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    @AutoLogOutput(key = "Krakens/{name}/supplyVoltage")
    private double supplyVoltage() {
        return motor.getSupplyVoltage().getValueAsDouble();
    }

    @AutoLogOutput(key = "Krakens/{name}/statorCurrent")
    private double motorCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    @AutoLogOutput(key = "Krakens/{name}/supplyCurrent")
    private double supplyCurrent() {
        return motor.getSupplyCurrent().getValueAsDouble();
    }

    @AutoLogOutput(key = "Krakens/{name}/temperature")
    private double temperature() {
        return motor.getDeviceTemp().getValueAsDouble();
    }

    @AutoLogOutput(key = "Krakens/{name}/torqueCurrent")
    private double torqueCurrent() {
        return motor.getTorqueCurrent().getValueAsDouble();
    }

    @AutoLogOutput(key = "Krakens/{name}/velocity")
    private double velocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    @AutoLogOutput(key = "Krakens/{name}/acceleration")
    private double acceleration() {
        return motor.getAcceleration().getValueAsDouble();
    }
}
