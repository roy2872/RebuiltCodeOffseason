package frc.robot;

import com.ctre.phoenix6.CANBus;

public enum Ports {
	INTAKE_ROLLERS_MAIN(8, Constants.rio),
	INTAKE_ROLLERS_FOLLOWER(9, Constants.rio),
	INTAKE_DEPLOY(10, Constants.rio),
	HOPPER_ROLLERS(11, Constants.rio),
	CLIMBER(12, Constants.rio),
	FEEDER_ROLLERS(13, Constants.rio),
	TUNNEL_ROLLERS_MAIN(14, Constants.rio),
	TUNNEL_ROLLERS_FOLLOWER(15, Constants.rio),
	SHOOTER_MAIN(16, Constants.rio),
	SHOOTER_FOLLOWER_1(17, Constants.rio),
	SHOOTER_FOLLOWER_2(18, Constants.rio),
	SHOOTER_FOLLOWER_3(19, Constants.rio),
	HOOD(20, Constants.rio);

	public final int id;
	public final CANBus bus;

	private Ports(int id, CANBus bus) {
		this.id = id;
		this.bus = bus;
	}
}