package cataclysm.datastructures;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

/**
 * This class is an implementation of {@link IdentifierManager} based on an
 * ArrayList for fast iteration and a hashmap for fast lookup methods
 * 
 * @author Briac
 *
 * @param <T>
 */
public abstract class Manager<T extends Identifier> implements IdentifierManager<T> {

	protected IDGenerator generator = new IDGenerator();
	protected ArrayList<T> elements = new ArrayList<T>();
	private Map<Long, T> map = new HashMap<Long, T>();

	public Manager() {

	}
	/**
	 * Update all the elements
	 */
	@Override
	public void update() {
		internalUpdate();
	}

	/**
	 * Custom update function
	 */
	protected abstract void internalUpdate();

	@Override
	public int size() {
		return elements.size();
	}

	@Override
	public void addElement(T element) {
		elements.add(element);
		map.put(element.getID(), element);
	}

	@Override
	public boolean contains(long ID) {
		return map.containsKey(ID);
	}
	
	@Override
	public T get(long ID) {
		return map.get(ID);
	}
	
	@Override
	public T removeAndGet(long ID) {
		T e = map.remove(ID);
		if(e != null) {
			for(int i=0; i<elements.size(); i++) {
				if(elements.get(i).getID() == ID) {
					elements.remove(i);
					break;
				}
			}
		}
		return e;
	}
	
	/**
	 * @return Reserves a unique ID for a newly allocated element
	 */
	public long nextID() {
		return generator.nextID();
	}

	
	@Override
	public void cleanUp() {
		elements.clear();
		map.clear();
		generator.reset();
	}

	@Override
	public Iterator<T> iterator() {
		return elements.iterator();
	}

}
