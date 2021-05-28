package cataclysm.parallel;

/**
 * Represents a general task to be run by a {@link PhysicsWorkerThread}
 * 
 * @author Briac
 *
 */
public abstract class PhysicsWork {

	String name;

	/**
	 * The task to be run.
	 * 
	 * @param worker The thread running this task
	 */
	public abstract void run(PhysicsWorkerThread worker);

	public String getName() {
		return name;
	}

	@Override
	public String toString() {
		return name == null ? super.toString() : name;
	}

}
