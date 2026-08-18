package frc.robot;

import com.ctre.phoenix6.CANBus;

public enum Ports {
	INTAKE_ROLLERS_MAIN(51, Constants.rio),
	INTAKE_ROLLERS_FOLLOWER(52, Constants.rio),
	INTAKE_DEPLOY(53, Constants.rio),
	FEEDER_ROLLERS_MAIN(54, Constants.rio),
	FEEDER_ROLLERS_FOLLOWER1(55, Constants.rio),
	FEEDER_ROLLERS_FOLLOWER2(56, Constants.rio),
	SHOOTER_MAIN(57, Constants.rio),
	SHOOTER_FOLLOWER_1(58, Constants.rio),
	SHOOTER_FOLLOWER_2(59, Constants.rio),
	SHOOTER_FOLLOWER_3(60, Constants.rio),
	HOOD(61, Constants.rio);

	public final int id;
	public final CANBus bus;

	private Ports(int id, CANBus bus) {
		this.id = id;
		this.bus = bus;
	}
}