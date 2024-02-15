package frc.robot;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.TalonFX;

@AutoLog
public class KrakenLogger {
    private TalonFX motor;
    private int index;
    private KrakenStats stats = new KrakenStatsAutoLogged();

    @AutoLogOutput(key = "CTRE/Kraken{index}")
    public class KrakenStats {
        private double motorVoltage = 0.0;
        private double supplyVoltage = 0.0;
        private double motorCurrent = 0.0;
        private double supplyCurrent = 0.0;
        private double torqueCurrent = 0.0;
        private double motorVelocity = 0.0;
        private double motorPosition = 0.0;
        private double acceleration = 0.0;
        private double temp = 0.0;
    }

    public KrakenLogger(TalonFX motor, int index) {
        this.motor = motor;
        this.index = index;
    }

    public void update() {
        stats.motorVoltage = motor.getMotorVoltage().getValueAsDouble();
        stats.supplyVoltage = motor.getSupplyVoltage().getValueAsDouble();
        stats.motorCurrent = motor.getStatorCurrent().getValueAsDouble();
        stats.supplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
        stats.acceleration = motor.getAcceleration().getValueAsDouble();
        stats.motorVelocity = motor.getVelocity().getValueAsDouble();
        stats.motorPosition = motor.getPosition().getValueAsDouble();
        stats.temp = motor.getDeviceTemp().getValueAsDouble();
        stats.torqueCurrent = motor.getTorqueCurrent().getValueAsDouble();
    } 
}
