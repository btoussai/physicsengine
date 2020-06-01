package cataclysm.datastructures;

import cataclysm.parallel.PhysicsWorkerPool;

/**
 * This class represents an abstract data structure managing identifiers.
 * 
 * @see Identifier
 * 
 * @author Briac
 *
 * @param <T>
 */
public interface IdentifierManager<T extends Identifier> extends Iterable<T> {

	/**
	 * Update all the elements
	 */
	public void update();

	/**
	 * Updates all the elements in a parallel fashion
	 * 
	 * @param workers A group of thread
	 */
	public void parallelUpdate(PhysicsWorkerPool workers);

	/**
	 * @return The number of elements contained in the data structure
	 */
	public int size();

	/**
	 * Adds an element to the data structure. The element's ID must have been
	 * generated through #nextID()
	 * 
	 * @param element
	 */
	public void addElement(T element);

	/**
	 * Deletes the unique element such that {@code e.getID() == ID}
	 * 
	 * @param ID
	 * @return true if such an an element has been deleted
	 */
	public default boolean removeElement(long ID) {
		return removeAndGet(ID) != null;
	}

	/**
	 * Deletes the unique element such that {@code e.getID() == ID}
	 * 
	 * @param ID
	 * @return The removed element or null
	 */
	public T removeAndGet(long ID);

	/**
	 * 
	 * @param ID
	 * @return The unique element such that {@code e.getID()==ID} or null
	 */
	public T get(long ID);

	/**
	 * 
	 * @param ID
	 * @return true if there is an element such that {@code e.getID()==ID}
	 */
	public boolean contains(long ID);

	/**
	 * Deletes all the elements
	 */
	public void cleanUp();
}
