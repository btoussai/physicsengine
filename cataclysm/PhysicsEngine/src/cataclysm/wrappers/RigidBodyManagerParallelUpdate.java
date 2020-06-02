package cataclysm.wrappers;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentHashMap.KeySetView;

import cataclysm.CataclysmCallbacks;
import cataclysm.CollisionFilter;
import cataclysm.PhysicsStats;
import cataclysm.RayTest;
import cataclysm.broadphase.AABB;
import cataclysm.broadphase.BroadPhaseNode;
import cataclysm.broadphase.BroadPhaseTree;
import cataclysm.broadphase.staticmeshes.StaticMeshManager;
import cataclysm.broadphase.staticmeshes.Triangle;
import cataclysm.contact_creation.AbstractDoubleBodyContact;
import cataclysm.contact_creation.AbstractSingleBodyContact;
import cataclysm.contact_creation.CollisionTest;
import cataclysm.parallel.PhysicsWork;
import cataclysm.parallel.PhysicsWorkerPool;
import math.vector.Vector3f;

/**
 * A parallel implementation of RigidBodyManagerUpdate
 * 
 * @author Briac
 *
 */
class RigidBodyManagerParallelUpdate extends BodyUpdator {

	/**
	 * A set of contacts for a wrapper
	 * 
	 * @author Briac
	 *
	 */
	private static class CollisionSet {
		private final Wrapper wrapper;
		private final KeySetView<AbstractDoubleBodyContact, Boolean> set = ConcurrentHashMap.newKeySet();

		public CollisionSet(Wrapper wrapper) {
			this.wrapper = wrapper;
		}

		void aggregate(HashSet<Wrapper> updatedSet, InternalUpdator updator,
				ConcurrentHashMap<Wrapper, CollisionSet> allBodyContacts) {
			// remove all contacts that aren't in the updatedSet
			set.removeIf(c -> {

				Wrapper other = wrapper == c.getWrapperA() ? c.getWrapperB() : c.getWrapperA();
				if (other == null) {
					return true;
				}
				if (!updatedSet.remove(other)) {
					// this contact isn't in the updated set anymore, it is an old contact

					if (wrapper.getID() > other.getID()) {
						// we add it to the updator owning the contact
						c.refresh(null, null);
						updator.bodyContactPool[c.getMaxContacts()].add(c);
					}

					return true;
				}

				return false;
			});

			for (Wrapper other : updatedSet) {
				// we check that the updator owns that contact
				if (wrapper.getID() > other.getID()) {
					AbstractDoubleBodyContact contact = updator.createBodyContact(wrapper, other);
					if (contact != null) {
						set.add(contact);
						allBodyContacts.get(other).set.add(contact);
					}
				}
			}

		}

		void transferToWrapper() {
			List<AbstractDoubleBodyContact> contacts = wrapper.getBodyContacts();
			contacts.clear();
			contacts.addAll(set);
		}

		void delete(InternalUpdator updator, ConcurrentHashMap<Wrapper, CollisionSet> allBodyContacts) {
			for (AbstractDoubleBodyContact c : set) {
				Wrapper other = c.getOther(wrapper);
				CollisionSet otherSet = allBodyContacts.get(other);

				if (otherSet != null) {// could be null if another thread is deleting other
					otherSet.set.remove(c);// we remove the contact in the other's set

					// reuse the contact only if it is owned by the current updator
					if (wrapper.getID() > other.getID()) {
						c.refresh(null, null);
						updator.bodyContactPool[c.getMaxContacts()].add(c);
					}
					// don't reuse the contact otherwise, even though it will be inaccessible and
					// garbage collected.
					// This is done to prevent the case of two threads calling this
					// method on wrappers sharing a contact and both trying to reuse it at the same
					// time.
				}
			}
			// finally clear the list, not useful but signifies to the user that the wrapper
			// has been
			// effectively removed
			wrapper.getBodyContacts().clear();
		}
	}

	private static class InternalUpdator extends BodyUpdator {
		private final HashSet<Wrapper> intersectedWrappers = new HashSet<Wrapper>();
		private final HashSet<Triangle> intersectedTriangles = new HashSet<Triangle>();
		private final CollisionTest collisionTest = new CollisionTest();
		private final BroadPhaseTree<Wrapper> bvh;
		private final float PADDING;
		private final float PADDING_SQUARED;

		/**
		 * A list of wrappers needing their contact lists to be updated because they
		 * moved outside their AABB
		 */
		private final List<Wrapper> movedWrappers = new ArrayList<>();

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

		public InternalUpdator(CollisionFilter filter, float padding) {
			super(filter);
			this.PADDING = padding;
			this.PADDING_SQUARED = padding * padding;
			this.bvh = new BroadPhaseTree<Wrapper>();

		}

		void processAddedAndRemovedElements(List<RigidBody> added, List<RigidBody> removed, StaticMeshManager meshes,
				ConcurrentHashMap<Wrapper, CollisionSet> allBodyContacts) {
			for (RigidBody body : removed) {
				for (Wrapper wrapper : body.getWrappers()) {
					// delete the mapping and its associated contacts
					allBodyContacts.remove(wrapper).delete(this, allBodyContacts);
					deleteMeshContacts(wrapper);
					bvh.remove(wrapper.getNode());
				}
			}

			for (RigidBody body : added) {
				for (Wrapper wrapper : body.getWrappers()) {
					// create a new mapping for this wrapper
					allBodyContacts.put(wrapper, new CollisionSet(wrapper));
					// place all the wrappers in the bvh and schedule them for an update
					BroadPhaseNode<Wrapper> node = wrapper.getNode();
					wrapper.placeBox(PADDING);
					bvh.update(node);
					movedWrappers.add(wrapper);
				}
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
		 * Updates the bvh of this thread.
		 */
		private void moveAABBs(List<RigidBody> bodies) {
			for (RigidBody body : bodies) {
				if (body.isSleeping())
					continue;
				ArrayList<Wrapper> wrappers = body.getWrappers();
				for (int i = 0; i < wrappers.size(); i++) {
					Wrapper wrapper = wrappers.get(i);
					AABB box = wrapper.getNode().getBox();
					Vector3f centroid = wrapper.getCentroid();

					float sx = 0.5f * (box.minX + box.maxX) - centroid.x;
					float sy = 0.5f * (box.minY + box.maxY) - centroid.y;
					float sz = 0.5f * (box.minZ + box.maxZ) - centroid.z;

					float d2 = sx * sx + sy * sy + sz * sz;

					if (d2 > PADDING_SQUARED) {
						BroadPhaseNode<Wrapper> node = wrapper.getNode();
						wrapper.placeBox(PADDING);
						bvh.update(node);
						movedWrappers.add(wrapper);
					}
				}
			}

		}

		/**
		 * Asks the bvhs for new pairs of contacts, asks the meshes for new contacts
		 * with triangles
		 */
		private void recomputeContactLists(List<InternalUpdator> updators, StaticMeshManager meshes,
				ConcurrentHashMap<Wrapper, CollisionSet> allBodyContacts) {

			for (Wrapper wrapper : movedWrappers) {
				intersectedWrappers.clear();

				BroadPhaseNode<Wrapper> node = wrapper.getNode();
				AABB box = node.getBox();
				// performs a box test in each bvh
				for (int i = 0; i < updators.size(); i++)
					updators.get(i).bvh.boxTest(box, intersectedWrappers);
				intersectedWrappers.remove(wrapper);

				allBodyContacts.get(wrapper).aggregate(intersectedWrappers, this, allBodyContacts);

				// check if it can collide with meshes
				if (wrapper.getBody().getInvMass() == 0) {
					continue;
				}

				// update intersected triangles
				intersectedTriangles.clear();
				meshes.boxTriangleQuery(box, intersectedTriangles);

				wrapper.getMeshContacts().removeIf(contact -> {
					if (!intersectedTriangles.remove(contact.getTriangle())) {
						contact.refresh(null, null);
						meshContactPool[contact.getMaxContacts()].add(contact);
						return true;
					}
					return false;
				});
				intersectedTriangles.forEach(triangle -> createMeshContact(wrapper, triangle));
			}

			// we clear the list at the end of the update because
			// processAddedAndRemovedElements() adds new wrappers in it
			movedWrappers.clear();
		}

		/**
		 * Updates the contacts.
		 */
		private void udpdateContacts(List<RigidBody> bodies, CataclysmCallbacks callbacks,
				ConcurrentHashMap<Wrapper, CollisionSet> allBodyContacts) {
			meshContacts.clear();
			bodyContacts.clear();

			for (RigidBody body : bodies) {

				if (body.isSleeping())
					continue;
				ArrayList<Wrapper> wrappers = body.getWrappers();
				for (int i = 0; i < wrappers.size(); i++) {
					Wrapper wrapper = wrappers.get(i);
					if (!body.isKinematic())
						collisionTest.meshContacts(wrapper, callbacks, meshContacts);

					allBodyContacts.get(wrapper).transferToWrapper();
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

		}

		@Override
		public BroadPhaseTree<Wrapper> getBVH(int i) {
			return bvh;
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

	}

	private final List<InternalUpdator> updators = new ArrayList<>();
	private final PhysicsWorkerPool workers;
	private final ConcurrentHashMap<Wrapper, CollisionSet> allBodyContacts = new ConcurrentHashMap<Wrapper, RigidBodyManagerParallelUpdate.CollisionSet>();

	RigidBodyManagerParallelUpdate(PhysicsWorkerPool workers, CollisionFilter filter, float padding) {
		super(filter);
		this.workers = workers;

		for (int i = 0; i < workers.getThreadCount(); i++) {
			updators.add(new InternalUpdator(filter, padding));
		}
	}

	private List<List<RigidBody>> splitList(List<RigidBody> l) {
		List<List<RigidBody>> tab = new ArrayList<>(updators.size());
		for (int i = 0; i < updators.size(); i++) {
			tab.add(new ArrayList<>(l.size() / updators.size() + 1));
		}
		for (RigidBody b : l) {
			tab.get(dispatchToUpdator(b)).add(b);
		}
		return tab;
	}

	@Override
	public void processAddedAndRemovedElements(List<RigidBody> added, List<RigidBody> removed,
			StaticMeshManager meshes) {
		List<PhysicsWork> tasks = new ArrayList<>();
		List<List<RigidBody>> addedSplit = splitList(added);
		List<List<RigidBody>> removedSplit = splitList(removed);

		for (int i = 0; i < workers.getThreadCount(); i++) {
			final int threadIndex = i;
			tasks.add(worker -> {
				updators.get(threadIndex).processAddedAndRemovedElements(addedSplit.get(threadIndex),
						removedSplit.get(threadIndex), meshes, allBodyContacts);
				worker.waitForTermination();
			});

		}
		workers.scheduleWork(tasks);
	}

	@Override
	public void updateBodies(RigidBodyManager bodies, StaticMeshManager meshes, CataclysmCallbacks callbacks,
			PhysicsStats stats, List<AbstractSingleBodyContact> meshContacts,
			List<AbstractDoubleBodyContact> bodyContacts) {
		meshContacts.clear();
		bodyContacts.clear();
		stats.bodyToMeshContacts = 0;
		stats.bodyToBodyContacts = 0;

		List<PhysicsWork> bvhUpdate = new ArrayList<>(workers.getThreadCount());
		List<PhysicsWork> contactListsUpdate = new ArrayList<>(workers.getThreadCount());
		List<PhysicsWork> contactsUpdate = new ArrayList<>(workers.getThreadCount());

		List<List<RigidBody>> slices = splitList(bodies.getElements());
		for (int i = 0; i < workers.getThreadCount(); i++) {
			List<RigidBody> slice = slices.get(i);
			InternalUpdator updator = updators.get(i);

			bvhUpdate.add(worker -> {
				updator.moveAABBs(slice);
				worker.waitForGroup();
			});

			contactListsUpdate.add(worker -> {
				updator.recomputeContactLists(updators, meshes, allBodyContacts);
				worker.waitForGroup();
			});

			contactsUpdate.add(worker -> {
				updator.udpdateContacts(slice, callbacks, allBodyContacts);
				worker.waitForTermination();
			});

		}
		workers.scheduleWork(bvhUpdate);
		workers.scheduleWork(contactListsUpdate);
		workers.scheduleWork(contactsUpdate);

		// we have to wait for processAddedAndRemovedElements to finish as well
		workers.waitForTaskTermination();
		// we wait for the end of udpdateContacts
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

	/**
	 * Dispatches a body to a specific updator.
	 * 
	 * @param b
	 * @return the index of the updator in charge of the body
	 */
	private int dispatchToUpdator(RigidBody b) {
		long ID = b.getID();
		return (int) (ID % updators.size());
	}

	@Override
	public BroadPhaseTree<Wrapper> getBVH(int i) {
		if (i < 0 || i > updators.size()) {
			throw new IllegalArgumentException(
					"Error, there are " + updators.size() + " threads, got asked for bvh n°" + i);
		}
		return updators.get(i).getBVH(0);
	}

	@Override
	public void rayTest(RayTest test) {
		int threads = updators.size();
		for(int i=0; i<threads; i++) {
			getBVH(i).rayTest(test);
		}
	}

	@Override
	public void boxTriangleQuery(AABB box, Set<Triangle> set) {
		throw new IllegalStateException("Not applicable");
	}

	@Override
	public void boxWrapperQuery(AABB box, Set<Wrapper> set) {
		int threads = updators.size();
		for(int i=0; i<threads; i++) {
			getBVH(i).boxTest(box, set);
		}
	}

}
