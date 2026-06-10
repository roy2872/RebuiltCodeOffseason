package frc.lib.util;

public class ControlGains {
    

    public static class PidGains {
        public final double kP;
        public final double kI;
        public final double kD;
        public PidGains(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
        public double kP() {
            return kP;
        }
        public double kI() {
            return kI;
        }
        public double kD() {
            return kD;
        }
    }

    public static class SimpleFFConstants {
        public final double kS;
        public final double kV;
        public final double kA;
        public SimpleFFConstants(double kS, double kV, double kA) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
        }
        public double kS() {
            return kS;
        }
        public double kV() {
            return kV;
        }
        public double kA() {
            return kA;
        }
    }

    public static class FFConstants {
        public final double kS;
        public final double kV;
        public final double kA;
        public final double kg;
        public FFConstants(double kS, double kV, double kA, double kg) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.kg = kg;
        }
        public double kS() {
            return kS;
        }
        public double kV() {
            return kV;
        }
        public double kA() {
            return kA;
        }
        public double kg() {
            return kg;
        }
    }

    public static class TrapezoidalProfileGains {
        public final double kP;
        public final double kI;
        public final double kD;
        public final double maxVelocity;
        public final double maxAcceleration;
        public TrapezoidalProfileGains(double kP, double kI, double kD, double maxVelocity, double maxAcceleration) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
        }
        public double kP() {
            return kP;
        }
        public double kI() {
            return kI;
        }
        public double kD() {
            return kD;
        }
        public double maxVelocity() {
            return maxVelocity;
        }
        public double maxAcceleration() {
            return maxAcceleration;
        }
    }
}
