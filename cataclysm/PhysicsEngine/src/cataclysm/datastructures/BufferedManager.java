package cataclysm.datastructures;

import java.util.ArrayList;
import java.util.List;

/**
 * This class provides buffers to store added and removed elements in order to
 * apply a processing step in the update method before actually adding or
 * removing them.
 * 
 * @author Briac
 *
 * @param <T>
 */
public abstract class BufferedManager<T extends Identifier> extends Manager<T> {

	/**
	 * A buffer of added elements
	 */
	private List<T> added = new ArrayList<T>();

	/**
	 * A buffer of removed elements
	 */
	private List<T> removed = new ArrayList<T>();

	public BufferedManager() {

	}

	/**
	 * Applies some processing step to the buffered added and removed elements. When
	 * the function returns, the elements are effectively added or removed. The
	 * buffers of added and removed elements are then cleared. Finally,
	 * {@link #internalUpdate()} is called.
	 * 
	 * @param added   The elements to be added
	 * @param removed The elements to be removed
	 */
	protected abstract void processAddedAndRemovedElements(List<T> added, List<T> removed);

	@Override
	public void update() {
		processAddedAndRemovedElements(added, removed);
		removed.forEach(e -> {
			super.removeElement(e.getID());
		});
		added.forEach(e -> super.addElement(e));

		added.clear();
		removed.clear();

		super.update();
	}

	@Override
	public void addElement(T element) {
		added.add(element);
	}

	@Override
	public boolean removeElement(long ID) {
		T e = super.get(ID);
		if (e != null && !removed.contains(e)) {
			removed.add(e);
			return true;
		}
		return false;
	}

	@Override
	public boolean contains(long ID) {
		T e = super.get(ID);
		return e != null && !removed.contains(e);
	}

	@Override
	public void cleanUp() {
		super.cleanUp();
		added.clear();
		removed.clear();
	}

}
