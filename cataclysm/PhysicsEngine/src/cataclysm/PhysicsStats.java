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
		private long stop;

		private final TimeUnit unit;
		private final String name;
		private final long[] history;
		private double average = Double.POSITIVE_INFINITY;
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

		public void stop() {
			stop = System.nanoTime();
			
			if(history!=null) {
				update(stop - start);
			}else {
				average = Math.min(average, stop-start);
			}
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
			for (int i = history.length-1; i >= history.length - length; i--) {
				std += (history[i]-average)*(history[i]-average);
			}
			std = Math.sqrt(std/length);
		}

		public double getAverageNanos() {
			return average;
		}
		
		public double getDeltaNanos() {
			return history[history.length - 1];
		}

		public String asPercentage(double totalNanoSec) {
			return name + ": " + String.format("%4.2f", average / totalNanoSec * 100.0) + "%" + " +/-" + String.format("%4.1f", 100.0*std/totalNanoSec) + "%";
		}

		@Override
		public String toString() {
			return name + ": " + String.format("%4.3f", unit.toUnit(average)) + unit.getUnitRepr();
		}
	}

	/**
	 * Le nombre de frame simulées depuis le lancement de la simulation.
	 */
	public long frame_count = 0;

	private int rigidBodies;
	private int staticMeshes;
	private int constraints;

	public int bodyToBodyContacts;
	public int bodyToBodyActiveContacts;

	public int bodyToMeshContacts;
	public int bodyToMeshActiveContacts;
	
	private final int smooth = 10;

	public final TimeAverage globalUpdate = new TimeAverage(TimeUnit.MILLISEC, "Global update", smooth);
	public final TimeAverage broadAndNarrowphase = new TimeAverage(TimeUnit.MILLISEC, "Broad & Narrow phase", smooth);
	public final TimeAverage constraintSolver = new TimeAverage(TimeUnit.MILLISEC, "Constraint Solver", smooth);
	public final TimeAverage velocityIntegration = new TimeAverage(TimeUnit.MILLISEC, "Velocity integration", smooth);

	public void reset(int rigidBodies, int staticMeshes, int constraints) {
		this.frame_count++;

		this.rigidBodies = rigidBodies;
		this.staticMeshes = staticMeshes;
		this.constraints = constraints;
	}

	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder("\nPhysicsWorld " + globalUpdate + " [\n");
		sb.append("\t Total " + broadAndNarrowphase.asPercentage(globalUpdate.average) + "\n");
		sb.append("\t" + constraintSolver.asPercentage(globalUpdate.average) + "\n");
		sb.append("\t" + velocityIntegration.asPercentage(globalUpdate.average) + "\n");
		sb.append("\tTotal frames simulated: " + frame_count);
		sb.append(
				"\n\tRigidBodies: " + rigidBodies + " StaticMeshes: " + staticMeshes + " Constraints: " + constraints);
		sb.append("\n\tBody to Body contacts: " + bodyToBodyContacts + " (" + bodyToBodyActiveContacts + " active)");
		sb.append("\n\tBody to Mesh contacts: " + bodyToMeshContacts + " (" + bodyToMeshActiveContacts + " active)");
		sb.append("\n] frame " + frame_count);
		return sb.toString();
	}

}
