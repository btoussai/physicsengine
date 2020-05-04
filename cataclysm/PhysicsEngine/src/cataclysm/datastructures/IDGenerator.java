package cataclysm.datastructures;

/**
 * This class generates unique IDs.
 * 
 * @author Briac
 *
 */
public final class IDGenerator {

	private long maxID = 0;

	public IDGenerator() {
	}

	public long nextID() {
		maxID++;
		return maxID;
	}

	public void reset() {
		maxID = 0;
	}

}
