package cataclysm.parallel;

/**
 * Represents a general task to be run by a {@link PhysicsWorkerThread}
 * 
 * @author Briac
 *
 */
@FunctionalInterface
public interface PhysicsWork {

	/**
	 * The task to be run.
	 * 
	 * @param worker The thread running this task
	 */
	public void run(PhysicsWorkerThread worker);

}
