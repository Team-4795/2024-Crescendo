package frc.robot.util;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;

public class CANSpark {
    private CANSparkBase motor;
    private AbsoluteEncoder absEncoder;
    private RelativeEncoder relativeEncoder;

    public static enum Controller {
        FLEX,
        MAX
    }

    public static class Motor {    
        private final Controller motorController;
        private final int canID;

        private int currentLimit = 30;
        private boolean compensate = false;
        private double compensationVolts = 12.0;
        private boolean inverted = false;
        private IdleMode mode = IdleMode.kBrake;
        private CANSpark leader;
        private boolean hasAbsoluteEncoder = false;
        private double relativePosConversion = 1.0;
        private double relativeVelConversion = 1.0;
        private double absPosConversion = 1.0;
        private double absVelConversion = 1.0;
        
        public Motor(Controller type, int canID){
            motorController = type;
            this.canID = canID;
        }

        public Motor currentLimit(int limit){
            currentLimit = limit;
            return this;
        }

        public Motor compensateVoltage(boolean compensate){
            this.compensate = compensate;
            return this;
        }

        public Motor compensateVoltage(boolean compensate, double value){
            this.compensate = compensate;
            compensationVolts = value;
            return this;
        }

        public Motor inverted(boolean inverted){
            this.inverted = inverted;
            return this;
        }

        public Motor idleMode(IdleMode mode){
            this.mode = mode;
            return this;
        }

        public Motor follows(CANSpark leader){
            this.leader = leader;
            return this;
        }

        public Motor hasAbsoluteEncoder(boolean absEncoder){
            hasAbsoluteEncoder = absEncoder;
            return this;
        }

        public Motor setRelativeConversionFactors(double position, double velocity){
            relativePosConversion = position;
            relativeVelConversion = velocity;
            return this;
        }

        public Motor setAbsoluteConversionFactors(double position, double velocity){
            absPosConversion = position;
            absVelConversion = velocity;
            return this;
        }

        public CANSpark configure(){
            return new CANSpark(this);
        }
    }

    private CANSpark(Motor build){
        switch(build.motorController){
            case FLEX:
                motor = new CANSparkFlex(build.canID, MotorType.kBrushless);
                break;
            case MAX:
                motor = new CANSparkMax(build.canID, MotorType.kBrushless);
                break;
            default:
                throw new IllegalArgumentException("Not a Recognized Motor Controller Type");
        }

        motor.restoreFactoryDefaults();

        motor.setSmartCurrentLimit(build.currentLimit);
        motor.setIdleMode(build.mode);
        if(build.compensate){
            motor.enableVoltageCompensation(build.compensationVolts);
        }
        motor.setInverted(build.inverted);

        if(build.leader != null){
            motor.follow(build.leader.motor);
        }

        relativeEncoder = motor.getEncoder();
        relativeEncoder.setPosition(0);
        if(build.hasAbsoluteEncoder){
            absEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
        } else {
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
        }

        //set conversion factors
        relativeEncoder.setPositionConversionFactor(build.relativePosConversion);
        relativeEncoder.setVelocityConversionFactor(build.relativeVelConversion);
        absEncoder.setPositionConversionFactor(build.absPosConversion);
        absEncoder.setVelocityConversionFactor(build.absVelConversion);

        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);

        motor.burnFlash();
    }

    public void set(double speed){
        motor.set(speed);
    }

    public void setVoltage(double voltage){
        motor.setVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    public double getRelativePosition(){
        return relativeEncoder.getPosition();
    }

    public double getRelativeVelocity(){
        return relativeEncoder.getVelocity();
    }

    public double getAbsolutePosition(){
        return absEncoder.getPosition();
    }

    public double getAbsoluteVelocity(){
        return absEncoder.getVelocity();
    }

    public void resetEncoders(){
        relativeEncoder.setPosition(0);
    }
}
