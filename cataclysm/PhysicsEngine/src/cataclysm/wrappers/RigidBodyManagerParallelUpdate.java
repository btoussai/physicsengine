package cataclysm.wrappers;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;

import cataclysm.CataclysmCallbacks;
import cataclysm.CollisionFilter;
import cataclysm.Epsilons;
import cataclysm.PhysicsStats;
import cataclysm.RayTest;
import cataclysm.broadphase.AABB;
import cataclysm.broadphase.BroadPhaseNode;
import cataclysm.broadphase.BroadPhaseTree;
import cataclysm.broadphase.ParallelBroadPhaseTree;
import cataclysm.broadphase.staticmeshes.StaticMeshManager;
import cataclysm.broadphase.staticmeshes.Triangle;
import cataclysm.contact_creation.AbstractContact;
import cataclysm.contact_creation.AbstractDoubleBodyContact;
import cataclysm.contact_creation.AbstractSingleBodyContact;
import cataclysm.contact_creation.CollisionTest;
import cataclysm.datastructures.Identifier;
import cataclysm.parallel.PhysicsWork;
import cataclysm.parallel.PhysicsWorkerPool;
import cataclysm.parallel.PhysicsWorkerThread;
import math.vector.Vector3f;

/**
 * A parallel implementation of RigidBodyManagerUpdate
 * 
 * @author Briac
 *
 */
class RigidBodyManagerParallelUpdate extends BodyUpdator {

	private static final boolean DEBUG = true;

	/**
	 * An internal class used as proxy to fill contact lists instead of a real
	 * contact. The proxy is replaced in a later step.
	 * 
	 * @author Briac
	 *
	 */
	private static class ContactProxy extends AbstractDoubleBodyContact {
		/**
		 * Index of the proxy in the contact list it is placed in
		 */
		private int index;

		public ContactProxy(Wrapper wrapperA, Wrapper wrapperB) {
			super(wrapperA, wrapperB);
		}

		@Override
		public void velocityStart() {
		}

		@Override
		public void warmStart() {
		}

		@Override
		public void resetImpulses() {
		}

		@Override
		public void solveVelocity() {
		}

		@Override
		public void positionStart(float timeStep) {
		}

		@Override
		public void solvePosition() {
		}

		public void setIndex(int index) {
			this.index = index;
		}

		public int getIndex() {
			return index;
		}

	}

	/**
	 * Updates the contact lists of the wrappers in parallel, but not concurrently.
	 * 
	 * When a contact is created, there are two cases: <br>
	 * -both bodies are updated: one of them will create the contact while the other
	 * will insert a proxy. The proxy is replaced in another pass by the genuine
	 * contact.<br>
	 * -only one is updated: the contact is created and inserted in its contact list
	 * and the contact is marked as 'to be inserted in another pass'.
	 * 
	 * When a contact is deleted, there are two cases: <br>
	 * -both bodies are updated: they both remove the contact from their contact
	 * list but only one of them stores the old contact in a pool <br>
	 * -only one is updated: the contact is removed from its contact list and stored
	 * in a pool. The other wrapper is marked as 'must clear deleted contacts in
	 * another pass'.
	 * 
	 * @author Briac
	 *
	 */
	private class InternalUpdator extends BodyUpdator {
		private final SortedSet<Wrapper> intersectedWrappers = new TreeSet<>((w1, w2) -> {
			return (int) (w1.getID() - w2.getID());
		});
		private final HashSet<Triangle> intersectedTriangles = new HashSet<Triangle>();
		private final CollisionTest collisionTest = new CollisionTest();

		/**
		 * This contact pool is used to store old contacts. The contact pool of the base
		 * class is used when a new contact is needed. The contact pools are swapped at
		 * the begining of each update. The idea is to avoid a contact being removed and
		 * reused immediately, before both wrapper have removed it from their contact
		 * lists.
		 */
		@SuppressWarnings("unchecked")
		protected final ArrayList<AbstractDoubleBodyContact>[] storeContactPool = new ArrayList[Epsilons.MAX_CONTACTS
				+ 1];

		/**
		 * A pool of proxy contacts to avoid unecessary allocations
		 */
		private final List<ContactProxy> proxyContactsPool = new ArrayList<>();

		/**
		 * Contains a list of proxy contacts having been added by this updator
		 */
		private final List<ContactProxy> addedProxyContacts = new ArrayList<>();

		/**
		 * A list of wrappers needing their contact lists to be updated because they
		 * moved outside their AABB
		 */
		private final List<Wrapper> movedWrappers = new ArrayList<>();

		/**
		 * A list of wrappers whose contacts list contains null contacts. <br>
		 * When a contact has to be deleted, it will be removed from one wrapper's
		 * contact list but not necessarily from the other.
		 */
		private final List<Wrapper> nullsToRemove = new ArrayList<>();

		/**
		 * A list of contacts that have been added in one wrapper's contact list but not
		 * in the other.
		 */
		private final List<AbstractDoubleBodyContact> contactsToInsert = new ArrayList<>();

		/**
		 * A list of the active mesh contacts identified during
		 * {@link #udpdateContacts(List, CataclysmCallbacks)}
		 */
		private final List<AbstractSingleBodyContact> meshContacts = new ArrayList<>();

		/**
		 * A list of the active body contacts identified during
		 * {@link #udpdateContacts(List, CataclysmCallbacks)}
		 */
		private final List<AbstractDoubleBodyContact> bodyContacts = new ArrayList<>();

		private final int updatorIndex;

		public InternalUpdator(CollisionFilter filter, int updatorIndex) {
			super(filter);
			this.updatorIndex = updatorIndex;
			for (int i = 1; i <= Epsilons.MAX_CONTACTS; i++) {
				storeContactPool[i] = new ArrayList<AbstractDoubleBodyContact>();
			}
		}

		public void swapContactPools() {
			for (int i = 1; i < storeContactPool.length; i++) {
				ArrayList<AbstractDoubleBodyContact> temp = storeContactPool[i];
				storeContactPool[i] = super.bodyContactPool[i];
				super.bodyContactPool[i] = temp;
			}
		}

		@Override
		void processAddedAndRemovedElements(List<RigidBody> added, List<RigidBody> removed, StaticMeshManager meshes) {
			throw new IllegalStateException("Not applicable");
		}

		@Override
		void updateBodies(RigidBodyManager bodies, StaticMeshManager meshes, CataclysmCallbacks callbacks,
				PhysicsStats stats, List<AbstractSingleBodyContact> meshContacts,
				List<AbstractDoubleBodyContact> bodyContacts) {
			throw new IllegalStateException("Not applicable");
		}

		/**
		 * We remove null contacts from contact lists whose wrapper hasn't been updated
		 */
		private void removeNullContacts() {
			for (Wrapper w : nullsToRemove) {
				w.bodyContacts.removeIf(c -> c.getWrapperA() == null || c.getWrapperB() == null);
			}
			nullsToRemove.clear();
		}

		/**
		 * We add new contacts to contact lists whose wrapper hasn't been updated
		 */
		private void insertNewContacts() {
			for (AbstractDoubleBodyContact c : contactsToInsert) {
				Wrapper a = c.getWrapperA();
				Wrapper b = c.getWrapperB();

				final Wrapper updated;
				final Wrapper toInsert;
				if (allMovedWrappers.contains(a)) {
					updated = a;
					toInsert = b;
				} else {
					updated = b;
					toInsert = a;
				}

				ArrayList<AbstractDoubleBodyContact> l = toInsert.getBodyContacts();
				int k = 0;
				while (k < l.size()) {
					if (l.get(k).getOther(toInsert).getID() < updated.getID()) {
						k++;
					} else {
						break;
					}
				}
				l.add(k, c);
			}
			contactsToInsert.clear();
		}

		/**
		 * Replaces the ContactProxy objects in the contact lists by the genuine
		 * contacts found in the other wrappers' contact list.
		 */
		private void subtituteContactProxys() {
			while (!addedProxyContacts.isEmpty()) {
				ContactProxy c = addedProxyContacts.remove(addedProxyContacts.size() - 1);

				Wrapper A = c.getWrapperA();
				Wrapper B = c.getWrapperB();

				List<AbstractDoubleBodyContact> l;
				AbstractDoubleBodyContact r;
				int i;
				if (A.getID() > B.getID()) {
					// A contains the true contact
					l = A.getBodyContacts();
					// iterate until we find the genuine contact
					for (i = 0; (r = l.get(i)).getOther(A) != B; i++)
						;
					l = B.getBodyContacts();
				} else {
					// B contains the true contact
					l = B.getBodyContacts();
					// iterate until we find the genuine contact
					for (i = 0; (r = l.get(i)).getOther(B) != A; i++)
						;
					l = A.getBodyContacts();
				}

				if (DEBUG) {
					if (A != r.getWrapperA() && B != r.getWrapperA() || A != r.getWrapperB() && B != r.getWrapperB()) {
						throw new IllegalStateException("Incompatible proxy & contact, mismatched wrappers");
					}
				}

				l.set(c.getIndex(), r);// swap the proxy with the true contact
				c.refresh(null, null);
				c.setIndex(-1);
				proxyContactsPool.add(c);

			}
		}

		/**
		 * Creates a new contact. The contact can be eiter a ProxyContact or a genuine
		 * contact.
		 * 
		 * @param wrapper
		 * @param other
		 * @param i       The index at which the contact is to be inserted in the
		 *                contact list
		 * @return
		 */
		private AbstractDoubleBodyContact createContact(Wrapper wrapper, Wrapper other, int i) {
			final AbstractDoubleBodyContact c;

			if (!allMovedWrappers.contains(other)) {
				c = super.createBodyContact(wrapper, other);
				// the other contact will not be updated during the frame
				// we mark the contact as 'to be inserted'
				contactsToInsert.add(c);
			} else if (wrapper.getID() > other.getID()) {
				// we consider the current wrapper owns the contact
				// and we create a genuine contact
				c = super.createBodyContact(wrapper, other);
			} else {
				// we create a proxy otherwise.
				// the proxy will be replaced later
				final ContactProxy proxyContact;
				if (proxyContactsPool.isEmpty()) {
					proxyContact = new ContactProxy(wrapper, other);
				} else {
					proxyContact = proxyContactsPool.remove(proxyContactsPool.size() - 1);
					proxyContact.refresh(wrapper, other);
				}
				proxyContact.setIndex(i);
				addedProxyContacts.add(proxyContact);
				c = proxyContact;
			}
			return c;
		}

		private void rebuildContactList(Wrapper wrapper) {

			if (DEBUG) {
				checkContactList(wrapper, true, true, true, false);
			}

			// The list is sorted by growing ID
			List<AbstractDoubleBodyContact> contacts = wrapper.getBodyContacts();

			int i = 0;
			Iterator<Wrapper> it = intersectedWrappers.iterator();
			// we iterate over the detected contact in the set
			setLoop: while (it.hasNext()) {
				// the set is also sorted by growing ID and does not contain
				// the current wrapper
				Wrapper w = it.next();

				// we remove all contacts not contained in the set
				// and we insert new contacts
				while (i < contacts.size()) {
					final AbstractDoubleBodyContact currentContact = contacts.get(i);
					// get a local reference that won't change between lookups
					final Wrapper wA = currentContact.getWrapperA();
					final Wrapper wB = currentContact.getWrapperB();

					if (wA == null || wB == null) {
						// The contact has to be removed since it has been reset
						// in another call of the function.
						// We check with || because the reset may not
						// have been propagated to both variables
						contacts.remove(i);
						continue;
					}

					// the contact has not been reset yet (according to the local copy of wA and
					// wB).
					// we get the ID of the other wrapper
					Wrapper other = (wrapper == wA ? wB : wA);
					long ID = other.getID();

					if (ID < w.getID()) {
						// the contact is not in the set, we remove it
						contacts.remove(i);
						if (!allMovedWrappers.contains(other)) {
							// the other wrapper won't be updated so we indicate that its contact list will
							// contain a null contact.
							nullsToRemove.add(other);
							// and we store the deleted contact in all cases
							currentContact.refresh(null, null);
							storeContactPool[currentContact.getMaxContacts()].add(currentContact);
						} else if (wrapper.getID() > ID) {
							// we store the contact only if the wrapper owns it
							currentContact.refresh(null, null);
							storeContactPool[currentContact.getMaxContacts()].add(currentContact);
						}
						// else we do nothing more
						continue;// next contact
					} else if (ID > w.getID()) {
						// we found a new contact
						if (getFilter().canCollide(wrapper.getBody(), w.getBody())) {
							contacts.add(i, createContact(wrapper, w, i));
						}
					}
					// go to next wrapper and next contact (or the size has increased and we stay on
					// that contact for the next loop)
					i++;
					continue setLoop;
				}

				// we add a new contact at the end of the contact list
				if (getFilter().canCollide(wrapper.getBody(), w.getBody())) {
					contacts.add(createContact(wrapper, w, i));
					i++;// the size has increased
				}
			}

			// we remove the remaining old contacts, starting from last
			while (i < contacts.size()) {
				final AbstractDoubleBodyContact currentContact = contacts.remove(contacts.size() - 1);
				// get a local reference that won't change between lookups
				final Wrapper wA = currentContact.getWrapperA();
				final Wrapper wB = currentContact.getWrapperB();

				if (wA == null || wB == null) {
					// The contact has to be removed since it has been reset
					// in another call of the function (not necessarily by another thread).
					// We check with || because the reset may not
					// have been propagated to both variables
					continue;
				}

				// the contact has not been reset yet (according to the local copy of wA and
				// wB).
				Wrapper other = wrapper == wA ? wB : wA;
				long ID = other.getID();

				if (!allMovedWrappers.contains(other)) {
					// the other wrapper won't be updated so we indicate that its contact list will
					// contain a null contact.
					nullsToRemove.add(other);
					// and we store the deleted contact in all cases
					currentContact.refresh(null, null);
					storeContactPool[currentContact.getMaxContacts()].add(currentContact);
				} else if (wrapper.getID() > ID) {
					// we store the contact only if the wrapper owns it
					currentContact.refresh(null, null);
					storeContactPool[currentContact.getMaxContacts()].add(currentContact);
				}
				// else we do nothing more
			}

			if (DEBUG) {
				if (it.hasNext() || i != contacts.size()) {
					throw new IllegalStateException("Unfinished iteration detected");
				}

				checkContactList(wrapper, false, true, true, false);
			}
		}

		/**
		 * Asks the bvh for new pairs of contacts, asks the meshes for new contacts with
		 * triangles
		 */
		private void recomputeContactLists(StaticMeshManager meshes) {

			if (DEBUG) {
				for (Wrapper wrapper : movedWrappers) {
					checkContactList(wrapper, true, true, true, true);
				}
			}

			for (Wrapper wrapper : movedWrappers) {

				intersectedWrappers.clear();

				BroadPhaseNode<Wrapper> node = wrapper.getNode();
				AABB box = node.getBox();

				bvh.boxTest(box, intersectedWrappers);
				intersectedWrappers.remove(wrapper);

				if (DEBUG && frame > 1) {
					HashSet<Wrapper> temp = new HashSet<>();
					for (Wrapper w : intersectedWrappers) {
						temp.clear();
						bvh.boxTest(w.getNode().getBox(), temp);
						if (!temp.contains(wrapper)) {
							System.out.println(bvh.getTree());
							throw new IllegalStateException("Boxtest isn't symmetric");
						}
					}
				}

				rebuildContactList(wrapper);

				// check if it can collide with meshes
				if (wrapper.getBody().getInvMass() == 0) {
					continue;
				}

				// update intersected triangles
				intersectedTriangles.clear();
				meshes.boxTriangleQuery(box, intersectedTriangles);

				wrapper.getMeshContacts().removeIf(contact -> {
					if (!intersectedTriangles.remove(contact.getTriangle())) {
						contact.getTriangle().mesh.getBodyContacts().remove(contact);
						contact.refresh(null, null);
						meshContactPool[contact.getMaxContacts()].add(contact);
						return true;
					}
					return false;
				});
				intersectedTriangles.forEach(triangle -> this.createMeshContact(wrapper, triangle));
			}

			if (DEBUG) {
				for (Wrapper wrapper : movedWrappers) {
					checkContactList(wrapper, false, true, true, false);
				}
			}

			// clear the list here
			// because it will be filled when updating the bvh at the next update
			this.movedWrappers.clear();
		}

		/**
		 * Updates the contacts.
		 */
		private void udpdateContacts(List<Wrapper> wrappers, CataclysmCallbacks callbacks) {
			meshContacts.clear();
			bodyContacts.clear();

			if (DEBUG) {
				for (Wrapper wrapper : wrappers) {
					checkContactList(wrapper, false, false, true, false);
				}
			}

			subtituteContactProxys();
			// we have to remove the null contacts after subtituting the proxies because the
			// proxies rely on their index in the list
			// removing contacts will mess up the indices so we do that later
			removeNullContacts();
			insertNewContacts();

			if (DEBUG) {
				for (Wrapper wrapper : wrappers) {
					checkContactList(wrapper, true, true, true, true);
				}
			}

			for (Wrapper wrapper : wrappers) {

				if (wrapper.getBody().isSleeping())
					continue;
				if (!wrapper.getBody().isKinematic())
					collisionTest.meshContacts(wrapper, callbacks, meshContacts);

				ArrayList<AbstractDoubleBodyContact> contacts = wrapper.getBodyContacts();
				for (int j = 0; j < contacts.size(); j++) {
					AbstractDoubleBodyContact contact = contacts.get(j);
					if (contact.getWrapperA() == wrapper || contact.getOther(wrapper).getBody().isSleeping()) {
						// we make sure only one wrapper in the pair effectively
						// calls the update
						collisionTest.bodyContacts(contact, callbacks, bodyContacts);
					}
				}
			}

		}

		@Override
		public void rayTest(RayTest test) {
			throw new IllegalStateException("Not applicable");
		}

		@Override
		public void boxTriangleQuery(AABB box, Set<Triangle> set) {
			throw new IllegalStateException("Not applicable");
		}

		@Override
		public void boxWrapperQuery(AABB box, Set<Wrapper> set) {
			throw new IllegalStateException("Not applicable");
		}

		@Override
		public BroadPhaseTree<Wrapper> getBVH() {
			throw new IllegalStateException("Not applicable");
		}

	}

	private int frame = 0;

	private final List<InternalUpdator> updators = new ArrayList<>();
	private final PhysicsWorkerPool workers;
	private final ParallelBroadPhaseTree<Wrapper> bvh = new ParallelBroadPhaseTree<>();

	private final float PADDING;
	private final float PADDING_SQUARED;

	private final HashSet<Wrapper> allMovedWrappers = new HashSet<>();// kick contain lookup
	private final HashSet<Wrapper> allNullWrappers = new HashSet<>();// we filter doubles
	private final List<AbstractDoubleBodyContact> allContactsToInsert = new ArrayList<>();

	RigidBodyManagerParallelUpdate(PhysicsWorkerPool workers, CollisionFilter filter, float padding) {
		super(filter);
		this.workers = workers;
		this.PADDING = padding;
		this.PADDING_SQUARED = padding * padding;
		for (int i = 0; i < workers.getThreadCount(); i++) {
			updators.add(new InternalUpdator(filter, i));
		}
	}

	private List<List<Wrapper>> splitList(List<RigidBody> l) {
		List<List<Wrapper>> tab = new ArrayList<>(updators.size());
		for (int i = 0; i < updators.size(); i++) {
			tab.add(new ArrayList<>(l.size() / updators.size() + 1));
		}
		for (RigidBody b : l) {
			for (Wrapper w : b.getWrappers()) {
				tab.get(dispatchToUpdator(w)).add(w);
			}
		}
		return tab;
	}

	@Override
	public void processAddedAndRemovedElements(List<RigidBody> added, List<RigidBody> removed,
			StaticMeshManager meshes) {

		// remove bodies from the bvh
		for (RigidBody body : removed) {
			for (Wrapper wrapper : body.getWrappers()) {
				bvh.remove(wrapper.getNode());
				InternalUpdator updator = updators.get(dispatchToUpdator(wrapper));

				updator.deleteMeshContacts(wrapper);// we delete and store the old mesh contacts
				for (int i = 0; i < wrapper.bodyContacts.size(); i++) {
					AbstractDoubleBodyContact c = wrapper.bodyContacts.get(i);
					if (c.getWrapperA() == null || c.getWrapperB() == null) {
						continue;// if the contact has already been removed, we skip
					}
					// we set the contact as 'removed'
					c.refresh(null, null);
					// we store the old body contact
					updator.storeContactPool[c.getMaxContacts()].add(c);
					// and we indicate that there will be one or more null contacts in the other's
					// contact list
					allNullWrappers.add(c.getOther(wrapper));
				}
				wrapper.bodyContacts.clear();
			}
		}

		for (Wrapper w : allNullWrappers) {
			updators.get(dispatchToUpdator(w)).nullsToRemove.add(w);
		}
		allNullWrappers.clear();
		for (InternalUpdator updator : updators) {
			updator.removeNullContacts();
		}

		// add bodies to the bvh
		allMovedWrappers.clear();
		for (RigidBody body : added) {
			for (Wrapper wrapper : body.getWrappers()) {
				BroadPhaseNode<Wrapper> node = wrapper.getNode();
				wrapper.placeBox(PADDING);
				bvh.add(node);
				// schedule the wrapper for an update
				allMovedWrappers.add(wrapper);
			}
		}
	}

	@Override
	public void updateBodies(RigidBodyManager bodies, StaticMeshManager meshes, CataclysmCallbacks callbacks,
			PhysicsStats stats, List<AbstractSingleBodyContact> meshContacts,
			List<AbstractDoubleBodyContact> bodyContacts) {

		frame++;

		meshContacts.clear();
		bodyContacts.clear();
		stats.bodyToMeshContacts = 0;
		stats.bodyToBodyContacts = 0;

		bvh.updateLeafs(workers, this::shouldMoveAABB);

		// careful, takes forever
		// if(DEBUG) {
		// checkBVHSymmetry();
		// }

		for (int i = 0; i < workers.getThreadCount(); i++) {
			updators.get(i).swapContactPools();// we swap the contacts used when inserting and removing contacts
			List<Wrapper> movedWrappers = updators.get(i).movedWrappers;
			this.allMovedWrappers.addAll(movedWrappers);
			movedWrappers.clear();
		}
		for (Wrapper w : allMovedWrappers) {
			updators.get(dispatchToUpdator(w)).movedWrappers.add(w);
		}

		List<PhysicsWork> contactListsUpdate = new ArrayList<>(workers.getThreadCount());
		List<PhysicsWork> contactsUpdate = new ArrayList<>(workers.getThreadCount());

		List<List<Wrapper>> slices = splitList(bodies.getElements());
		for (int i = 0; i < workers.getThreadCount(); i++) {
			List<Wrapper> slice = slices.get(i);
			InternalUpdator updator = updators.get(i);

			contactListsUpdate.add(new PhysicsWork() {

				@Override
				public void run(PhysicsWorkerThread worker) {
					updator.recomputeContactLists(meshes);
					worker.waitForGroup();
				}

			});

			contactsUpdate.add(new PhysicsWork() {

				@Override
				public void run(PhysicsWorkerThread worker) {
					updator.udpdateContacts(slice, callbacks);
					worker.waitForTermination();
				}

			});

		}
		workers.scheduleWork(contactListsUpdate, "recomputeContactLists", this::prepareUpdate, 1);
		workers.scheduleWork(contactsUpdate, "udpdateContacts", 1);

		// we wait for the end of recomputeContactLists and udpdateContacts
		workers.waitForTaskTermination();

		// we can aggregate all active contacts
		for (InternalUpdator updator : updators) {
			bodyContacts.addAll(updator.bodyContacts);
			meshContacts.addAll(updator.meshContacts);
		}

		stats.bodyToBodyContacts /= 2;
		stats.bodyToBodyActiveContacts = bodyContacts.size();
		stats.bodyToMeshActiveContacts = meshContacts.size();
	}

	private void prepareUpdate() {
		for (int i = 0; i < workers.getThreadCount(); i++) {
			List<Wrapper> nulls = updators.get(i).nullsToRemove;
			allNullWrappers.addAll(nulls);
			nulls.clear();
			List<AbstractDoubleBodyContact> toInsert = updators.get(i).contactsToInsert;
			allContactsToInsert.addAll(toInsert);
			toInsert.clear();
		}
		for (Wrapper w : allNullWrappers) {
			updators.get(dispatchToUpdator(w)).nullsToRemove.add(w);
		}
		allNullWrappers.clear();

		for (AbstractDoubleBodyContact c : allContactsToInsert) {
			final Wrapper w;// the wrapper in which the contact has to be inserted
			if (allMovedWrappers.contains(c.getWrapperA())) {
				w = c.getWrapperB();
			} else {
				w = c.getWrapperA();
			}
			updators.get(dispatchToUpdator(w)).contactsToInsert.add(c);
		}
		allContactsToInsert.clear();
	}

	/**
	 * Tests if an AABB should be updated inside the bvh.
	 * 
	 * @param threadIndex The thread calling the function or -1 if it is the main
	 *                    thread
	 * @param wrapper
	 * @return
	 */
	private boolean shouldMoveAABB(Wrapper wrapper) {
		AABB box = wrapper.getNode().getBox();
		Vector3f centroid = wrapper.getCentroid();

		float sx = 0.5f * (box.minX + box.maxX) - centroid.x;
		float sy = 0.5f * (box.minY + box.maxY) - centroid.y;
		float sz = 0.5f * (box.minZ + box.maxZ) - centroid.z;

		float d2 = sx * sx + sy * sy + sz * sz;

		if (d2 > PADDING_SQUARED) {
			wrapper.placeBox(PADDING);
			updators.get(dispatchToUpdator(wrapper)).movedWrappers.add(wrapper);
			return true;
		}
		return false;
	}

	/**
	 * Dispatches an object to a specific updator.
	 * 
	 * @param o
	 * @return the index of the updator in charge of the body
	 */
	private int dispatchToUpdator(Identifier o) {
		long ID = o.getID();
		return (int) (ID % updators.size());
	}

	@Override
	public BroadPhaseTree<Wrapper> getBVH() {
		return bvh.getTree();
	}

	@Override
	public void rayTest(RayTest test) {
		bvh.rayTest(test);
	}

	@Override
	public void boxTriangleQuery(AABB box, Set<Triangle> set) {
		throw new IllegalStateException("Not applicable");
	}

	@Override
	public void boxWrapperQuery(AABB box, Set<Wrapper> set) {
		bvh.boxTest(box, set);
	}

	private static void checkContactList(Wrapper wrapper, boolean proxyCheck, boolean pertainCheck, boolean sortCheck,
			boolean symmetricTest) {
		ArrayList<AbstractDoubleBodyContact> l = wrapper.getBodyContacts();
		long ID = -1;
		for (int i = 0; i < l.size(); i++) {
			AbstractDoubleBodyContact c = l.get(i);
			if (proxyCheck && c instanceof ContactProxy) {
				throw new IllegalStateException("Found an unreplaced ContactProxy");
			}

			Wrapper wA = c.getWrapperA();
			Wrapper wB = c.getWrapperB();

			if (pertainCheck && (wA == null || wB == null)) {
				throw new IllegalStateException("Found a contact with null wrappers");
			}

			if (pertainCheck && (wA != wrapper && wB != wrapper)) {
				throw new IllegalStateException("Found a contact not pertaining to this wrapper");
			}

			if (pertainCheck && sortCheck) {
				long id_c = c.getOther(wrapper).getID();
				if (id_c <= ID) {
					throw new IllegalStateException("The contact list isn't sorted");
				}
				ID = id_c;
			}

			if (symmetricTest && !c.getOther(wrapper).bodyContacts.contains(c)) {
				throw new IllegalStateException("Error, contact lists aren't symmetric");
			}
		}
	}

	private void checkBVHSymmetry() {
		List<Wrapper> leaves = new ArrayList<>();
		bvh.getTree().getLeaves(null, leaves);

		HashSet<Wrapper> temp = new HashSet<>();
		HashSet<Wrapper> temp2 = new HashSet<>();
		for (Wrapper wrapper : leaves) {
			temp.clear();
			bvh.boxTest(wrapper.getNode().getBox(), temp);
			for (Wrapper w : temp) {
				temp2.clear();
				bvh.boxTest(w.getNode().getBox(), temp2);
				if (!temp2.contains(wrapper)) {
					throw new IllegalStateException("Boxtest isn't symmetric");
				}
			}
		}
	}

}
