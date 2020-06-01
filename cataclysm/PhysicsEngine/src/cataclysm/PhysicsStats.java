package cataclysm;

/**
 * Cette classe regroupe des infos concernant le nombre d'objet dans la
 * simulation et le temps de calcul de la dernière frame.
 * 
 * @author Briac
 *
 */
public class PhysicsStats {

	public static enum TimeUnit {
		SEC("sec", 1E-9f), MILLISEC("ms", 1E-6f), MICROSEC("µs", 1E-3f), NANOSEC("ns", 1E0f);

		private String repr;
		private float nanosToUnit;

		TimeUnit(String repr, float nanosToUnit) {
			this.repr = repr;
			this.nanosToUnit = nanosToUnit;
		}

		public String getUnitRepr() {
			return repr;
		}

		public double toUnit(double nanosValue) {
			return nanosValue * nanosToUnit;
		}
	}

	public static class TimeAverage {
		private long updateCount = 0;
		private long start;
		private long accumulatedDelta;
		private long stop;

		private final TimeUnit unit;
		private final String name;
		private final long[] history;
		private double average = 0.0;
		private double std = 0.0;

		public TimeAverage(TimeUnit unit, String name, int length) {
			this.unit = unit;
			this.name = name;
			this.history = new long[length];
		}

		public TimeAverage(TimeUnit unit, String name) {
			this.unit = unit;
			this.name = name;
			this.history = null;
		}

		public void start() {
			start = System.nanoTime();
		}

		public void pause() {
			stop = System.nanoTime();
			accumulatedDelta += stop - start;
		}

		public void stop() {
			stop = System.nanoTime();
			accumulatedDelta += stop - start;

			if (history != null) {
				update(accumulatedDelta);
			} else {
				average = Math.min(average, accumulatedDelta);
			}
			accumulatedDelta = 0;
		}

		private void update(long deltaNanoSec) {
			updateCount++;

			average = 0;
			for (int i = 0; i < history.length - 1; i++) {
				history[i] = history[i + 1];
				average += history[i];
			}
			average += deltaNanoSec;
			history[history.length - 1] = deltaNanoSec;

			long length = updateCount < history.length ? updateCount : history.length;
			average /= length;

			std = 0;
			for (int i = history.length - 1; i >= history.length - length; i--) {
				std += (history[i] - average) * (history[i] - average);
			}
			std = Math.sqrt(std / length);
		}

		public double getAverageNanos() {
			return average;
		}

		public double getDeltaNanos() {
			return history[history.length - 1];
		}

		public String display(double totalNanoSec, boolean percentage) {
			if (percentage) {
				return name + ": " + String.format("%4.1f", average / totalNanoSec * 100.0) + "%" + " +/-"
						+ String.format("%4.1f", 100.0 * std / totalNanoSec) + "%";
			} else {
				return name + ": " + String.format("%4.1f", unit.toUnit(average)) + unit.getUnitRepr() + " +/-"
						+ String.format("%4.1f", unit.toUnit(std)) + unit.getUnitRepr();
			}
		}

		@Override
		public String toString() {
			return name + ": " + String.format("%4.1f", unit.toUnit(average)) + unit.getUnitRepr();
		}
	}

	/**
	 * Le nombre de frame simulées depuis le lancement de la simulation.
	 */
	private long frame_count = 0;
	private double elapsedTime = 0;

	private int rigidBodies;
	private int staticMeshes;
	private int constraints;
	public int threads;

	public int bodyToBodyContacts;
	public int bodyToBodyActiveContacts;

	public int bodyToMeshContacts;
	public int bodyToMeshActiveContacts;

	private final int smooth = 5;

	public final TimeAverage globalUpdate = new TimeAverage(TimeUnit.MILLISEC, "Global update", smooth);
	public final TimeAverage broadAndNarrowphase = new TimeAverage(TimeUnit.MILLISEC, "Broad & Narrow phase", smooth);
	public final TimeAverage constraintSolver = new TimeAverage(TimeUnit.MILLISEC, "Constraint Solver", smooth);
	public final TimeAverage velocityIntegration = new TimeAverage(TimeUnit.MILLISEC, "Velocity integration", smooth);

	public final TimeAverage physicsRecorder = new TimeAverage(TimeUnit.MILLISEC, "Simulation recording", smooth);
	public final TimeAverage physicsPlayers = new TimeAverage(TimeUnit.MILLISEC, "Simulation replaying", smooth);

	public void step(float timeStep) {
		this.frame_count++;
		this.elapsedTime += timeStep;
	}

	public void reset(int rigidBodies, int staticMeshes, int constraints, int threads) {
		this.rigidBodies = rigidBodies;
		this.staticMeshes = staticMeshes;
		this.constraints = constraints;
		this.threads = threads;
	}

	@Override
	public String toString() {
		boolean percentage = false;

		StringBuilder sb = new StringBuilder(
				"PhysicsWorld " + globalUpdate + " (" + threads + " thread(s)) [ \n");
		if (physicsRecorder.getDeltaNanos() != 0)
			sb.append("\t" + physicsRecorder.display(globalUpdate.average, percentage) + "\n");
		if (physicsPlayers.getDeltaNanos() != 0)
			sb.append("\t" + physicsPlayers.display(globalUpdate.average, percentage) + "\n");
		sb.append("\tTotal " + broadAndNarrowphase.display(globalUpdate.average, percentage) + "\n");
		sb.append("\t" + constraintSolver.display(globalUpdate.average, percentage) + "\n");
		sb.append("\t" + velocityIntegration.display(globalUpdate.average, percentage) + "\n");
		sb.append("\tTotal frames simulated: " + frame_count);
		sb.append(
				"\n\tRigidBodies: " + rigidBodies + " StaticMeshes: " + staticMeshes + " Constraints: " + constraints);
		sb.append("\n\tBody to Body contacts: " + bodyToBodyContacts + " (" + bodyToBodyActiveContacts + " active)");
		sb.append("\n\tBody to Mesh contacts: " + bodyToMeshContacts + " (" + bodyToMeshActiveContacts + " active)");
		sb.append("\n] frame " + frame_count);
		return sb.toString();
	}

	public long getFrameCount() {
		return frame_count;
	}

	public double getElapsedTime() {
		return elapsedTime;
	}

}
