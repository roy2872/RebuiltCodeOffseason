package frc.lib.util.power;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class PowerManagerTest {
	private static class FakeConsumer implements PowerConsumer {
		final String name;
		double drawAmps = 0.0;
		double limitAmps = Double.NaN;

		FakeConsumer(String name) {
			this.name = name;
		}

		@Override
		public String getPowerName() {
			return name;
		}

		@Override
		public double getSupplyCurrentAmps() {
			return drawAmps;
		}

		@Override
		public void setSupplyCurrentLimitAmps(double totalAmps) {
			limitAmps = totalAmps;
		}
	}

	private static final FakeConsumer drive = new FakeConsumer("TestDrive");
	private static final FakeConsumer shooter = new FakeConsumer("TestShooter");
	private static final FakeConsumer intake = new FakeConsumer("TestIntake");

	@BeforeAll
	static void setup() {
		assertTrue(HAL.initialize(500, 0));
		PowerManager.mInstance.register(drive, new PowerConfig("TestDrive", PowerPriority.CRITICAL, 100.0, 280.0));
		PowerManager.mInstance.register(shooter, new PowerConfig("TestShooter", PowerPriority.HIGH, 40.0, 120.0));
		PowerManager.mInstance.register(intake, new PowerConfig("TestIntake", PowerPriority.LOW, 10.0, 40.0));
	}

	private static void runLoops(int loops) {
		for (int i = 0; i < loops; i++) {
			PowerManager.mInstance.periodic();
		}
	}

	@Test
	void shedsLowPriorityFirst() {
		// Idle robot: full battery, everyone keeps their max
		runLoops(10);
		assertEquals(280.0, drive.limitAmps, 1e-6);
		assertEquals(120.0, shooter.limitAmps, 1e-6);
		assertEquals(40.0, intake.limitAmps, 1e-6);
		assertEquals(PowerManager.PowerState.NORMAL, PowerManager.mInstance.getState());

		// 260A draw sags the simulated battery (12V - 260A * 0.02ohm = 6.8V) under the 8V target.
		// Budget = 260 + (6.8 - 8) / 0.02 = 200A, minimums use 150A, the 50A left goes to the drive.
		drive.drawAmps = 180.0;
		shooter.drawAmps = 60.0;
		intake.drawAmps = 20.0;
		runLoops(10);
		assertEquals(10.0, intake.limitAmps, 1e-6);
		assertEquals(40.0, shooter.limitAmps, 1e-6);
		assertEquals(150.0, drive.limitAmps, 1.0);
		assertEquals(PowerManager.PowerState.CRITICAL, PowerManager.mInstance.getState());

		// Load gone: limits recover gradually instead of jumping back to max
		drive.drawAmps = 0.0;
		shooter.drawAmps = 0.0;
		intake.drawAmps = 0.0;
		runLoops(10);
		assertTrue(intake.limitAmps < 40.0);
		assertTrue(drive.limitAmps < 280.0);
	}
}
