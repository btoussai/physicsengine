package cataclysm.datastructures;

import java.util.ArrayList;
import java.util.List;

/**
 * Représente un tas binaire.
 * 
 * @author Briac
 *
 * @param <T>
 */
public abstract class BinaryHeap<T extends Identifier> extends IdentifierManager<T> {

	private final List<T> elements = new ArrayList<T>();

	@Override
	public int size() {
		return elements.size();
	}

	@Override
	public void addElement(T element) {
		int position = elements.size();
		elements.add(element);

		while (position != 0) {
			int parentPos = (position - 1) / 2;
			T parent = elements.get(parentPos);
			if (parent.getID() < element.getID()) {
				break;
			}
			elements.set(position, parent);
			position = parentPos;
		}
		elements.set(position, element);

	}

	@Override
	public boolean removeElement(long ID) {
		return removeAndGet(ID) != null;
	}

	@Override
	public T removeAndGet(long ID) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public boolean contains(long ID) {

		int position = elements.size();

		while (position != 0) {
			int parentPos = (position - 1) / 2;
			T parent = elements.get(parentPos);
			if (parent.getID() < ID) {
				break;
			}
			elements.set(position, parent);
			position = parentPos;
		}
		
		
		return false;
	}

}
