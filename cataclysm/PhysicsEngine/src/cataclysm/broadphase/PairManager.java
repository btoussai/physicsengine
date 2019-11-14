package cataclysm.broadphase;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import cataclysm.CollisionFilter;
import cataclysm.contact_creation.DoubleBodyContact;
import cataclysm.datastructures.Manager;
import cataclysm.wrappers.Wrapper;

/**
 * Représente un ensemble de {@link Pair}.
 * 
 * @author Briac
 *
 */
public class PairManager implements Iterable<Pair> {

	private static final boolean POOLING_ENABLED = true;

	private final Manager<Pair> pairs = new Manager<Pair>() {
		@Override
		protected void internalUpdate() {
		}
	};

	private final CollisionFilter filter;

	/**
	 * Représente un buffer permettant de réutiliser les contacts.
	 */
	private final Map<Integer, List<DoubleBodyContact>> contactPool = new HashMap<Integer, List<DoubleBodyContact>>();

	public PairManager(CollisionFilter collisionFilter) {
		this.filter = collisionFilter;
	}

	public void addPair(Wrapper wrapperA, Wrapper wrapperB) {

		if (!filter.canCollide(wrapperA.getBody(), wrapperB.getBody())) {
			return;
		}

		Pair pair = new Pair(wrapperA, wrapperB);
		pair.validate(retreiveContact(wrapperA.getType().maxContacts));

		pairs.addElement(pair);
	}

	public void removePair(long ID) {
		Pair pair = pairs.removeAndGet(ID);
		if (pair != null) {
			if (POOLING_ENABLED)
				storeContact(pair.getContact());
			pair.getWrapperA().removeCollision();
			pair.getWrapperB().removeCollision();
		}
	}

	public boolean contains(long ID) {
		return pairs.contains(ID);
	}

	private void storeContact(DoubleBodyContact contact) {
		List<DoubleBodyContact> contacts = contactPool.get(contact.getMaxContacts());
		if (contacts == null) {
			contacts = new ArrayList<DoubleBodyContact>();
			contactPool.put(contact.getMaxContacts(), contacts);
		}
		contacts.add(contact);
	}

	private DoubleBodyContact retreiveContact(int maxContacts) {
		List<DoubleBodyContact> contacts = contactPool.get(maxContacts);
		if (contacts == null) {
			contacts = new ArrayList<DoubleBodyContact>();
			contactPool.put(maxContacts, contacts);
		}

		if (!contacts.isEmpty()) {
			return contacts.remove(contacts.size() - 1);
		}

		return null;
	}

	@Override
	public Iterator<Pair> iterator() {
		return pairs.iterator();
	}

	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder("Pairs:\n");
		for (Pair p : this) {
			sb.append(p);
			sb.append("\t");
		}
		return sb.toString();
	}

	public void cleanUp() {
		pairs.cleanUp();
		contactPool.clear();
	}

	/**
	 * @return Le nombre de paires d'objets.
	 */
	public int size() {
		return pairs.size();
	}

}
