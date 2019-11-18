package cataclysm.wrappers;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import org.lwjgl.util.vector.Vector3f;

import cataclysm.CataclysmCallbacks;
import cataclysm.CollisionFilter;
import cataclysm.Epsilons;
import cataclysm.PhysicsStats;
import cataclysm.broadphase.AABB;
import cataclysm.broadphase.BroadPhaseNode;
import cataclysm.broadphase.BroadPhaseTree;
import cataclysm.broadphase.staticmeshes.StaticMeshManager;
import cataclysm.broadphase.staticmeshes.Triangle;
import cataclysm.contact_creation.CollisionTest;
import cataclysm.contact_creation.DoubleBodyContact;
import cataclysm.contact_creation.SingleBodyContact;

/**
 * Cette classe permet d'effectuer la broadphase et la narrow phase en une même
 * passe.
 * 
 * @author Briac
 *
 */
class RigidBodyManagerUpdate {

	/**
	 * Représente un buffer permettant de réutiliser les contacts body vs body.
	 */
	@SuppressWarnings("unchecked")
	private final ArrayList<DoubleBodyContact>[] bodyContactPool = new ArrayList[Epsilons.MAX_CONTACTS + 1];

	/**
	 * Représente un buffer permettant de réutiliser les contacts body vs triangle.
	 */
	@SuppressWarnings("unchecked")
	private final ArrayList<SingleBodyContact>[] meshContactPool = new ArrayList[Epsilons.MAX_CONTACTS + 1];

	private final HashSet<Wrapper> intersectedWrappers = new HashSet<Wrapper>();
	private final HashSet<Triangle> intersectedTriangles = new HashSet<Triangle>();

	private final CollisionFilter filter;

	private final BroadPhaseTree<Wrapper> bvh = new BroadPhaseTree<Wrapper>();

	private final float PADDING;
	private final float PADDING_SQUARED;

	private final CollisionTest collisionTest = new CollisionTest();

	public RigidBodyManagerUpdate(CollisionFilter filter, float padding) {
		this.filter = filter;
		this.PADDING = padding;
		this.PADDING_SQUARED = padding * padding;

		for (int i = 1; i <= Epsilons.MAX_CONTACTS; i++) {
			bodyContactPool[i] = new ArrayList<DoubleBodyContact>();
			meshContactPool[i] = new ArrayList<SingleBodyContact>();
		}
	}

	void processAddedAndRemovedElements(List<RigidBody> added, List<RigidBody> removed, StaticMeshManager meshes) {

		for (RigidBody body : removed) {
			for (Wrapper wrapper : body.getWrappers()) {
				deleteMeshContacts(wrapper);
				deleteBodyContacts(wrapper);
				bvh.remove(wrapper.getNode());
			}
		}

		for (RigidBody body : added) {
			for (Wrapper wrapper : body.getWrappers()) {
				recomputeDoubleBodyContactList(wrapper);
				recomputeSingleBodyContactList(wrapper, meshes);
			}
		}

	}

	void updateBodies(RigidBodyManager bodies, StaticMeshManager meshes, CataclysmCallbacks callbacks,
			PhysicsStats stats, List<SingleBodyContact> meshContacts, List<DoubleBodyContact> bodyContacts) {
		meshContacts.clear();
		bodyContacts.clear();
		stats.bodyToMeshContacts = 0;
		stats.bodyToBodyContacts = 0;

		for (RigidBody body : bodies) {
			if (body.isSleeping())
				continue;
			ArrayList<Wrapper> wrappers = body.getWrappers();
			if (wrappers.size() == 1) {
				updateWrapper(body.getInvMass() == 0, wrappers.get(0), meshes, callbacks, stats, meshContacts,
						bodyContacts);
			} else {
				for (int i = 0; i < wrappers.size(); i++) {
					Wrapper wrapper = wrappers.get(i);
					updateWrapper(body.getInvMass() == 0, wrapper, meshes, callbacks, stats, meshContacts,
							bodyContacts);
				}
			}
		}

		stats.bodyToBodyContacts /= 2;
		stats.bodyToBodyActiveContacts = bodyContacts.size();
		stats.bodyToMeshActiveContacts = meshContacts.size();
	}

	private void updateWrapper(boolean isKinematic, Wrapper wrapper, StaticMeshManager meshes,
			CataclysmCallbacks callbacks, PhysicsStats stats, List<SingleBodyContact> meshContacts,
			List<DoubleBodyContact> bodyContacts) {
		AABB box = wrapper.getNode().getBox();
		Vector3f centroid = wrapper.getCentroid();

		float sx = 0.5f * (box.min.x + box.max.x) - centroid.x;
		float sy = 0.5f * (box.min.y + box.max.y) - centroid.y;
		float sz = 0.5f * (box.min.z + box.max.z) - centroid.z;

		float d2 = sx * sx + sy * sy + sz * sz;

		if (d2 > PADDING_SQUARED) {
			recomputeDoubleBodyContactList(wrapper);
			recomputeSingleBodyContactList(wrapper, meshes);
		}

		stats.bodyToMeshContacts += wrapper.getMeshContacts().size();

		if (!isKinematic)
			collisionTest.meshContacts(wrapper, callbacks, meshContacts);

		ArrayList<DoubleBodyContact> contacts = wrapper.getBodyContacts();
		if (contacts.isEmpty()) {
			return;
		}
		stats.bodyToBodyContacts += contacts.size();

		for (int j = 0; j < contacts.size(); j++) {
			DoubleBodyContact contact = contacts.get(j);
			//boolean wrapperA_sleeping = contact.getWrapperA().getBody().isSleeping();
			//boolean wrapperB_sleeping = contact.getWrapperB().getBody().isSleeping();
			//boolean update = (wrapperA_sleeping || wrapperB_sleeping) || contact.getUpdateFlag();
			
			boolean update = contact.getUpdateFlag();
			if (update) {
				collisionTest.bodyContacts(contact, callbacks, bodyContacts);
				contact.setUpdateFlag(false);
			} else {
				contact.setUpdateFlag(true);
			}

		}
	}

	void recomputeDoubleBodyContactList(Wrapper wrapper) {
		intersectedWrappers.clear();

		BroadPhaseNode<Wrapper> node = wrapper.getNode();
		wrapper.placeBox(PADDING);

		//bvh.remove(node);
		//bvh.add(node);
		bvh.update(node);
		bvh.boxTest(node.getBox(), intersectedWrappers);
		intersectedWrappers.remove(wrapper);

		ArrayList<DoubleBodyContact> contacts = wrapper.getBodyContacts();

		contacts.removeIf(contact -> {
			Wrapper other = wrapper.getID() == contact.getWrapperA().getID() ? contact.getWrapperB()
					: contact.getWrapperA();
			if (!intersectedWrappers.remove(other)) {
				other.getBodyContacts().remove(contact);
				contact.refresh(null, null);
				bodyContactPool[contact.getMaxContacts()].add(contact);
				return true;
			}
			return false;
		});
		intersectedWrappers.forEach(other -> createBodyContact(wrapper, other));
	}

	void recomputeSingleBodyContactList(Wrapper wrapper, StaticMeshManager meshes) {

		if (wrapper.getBody().getInvMass() == 0) {
			return;
		}

		intersectedTriangles.clear();
		meshes.boxTest(wrapper.getNode().getBox(), intersectedTriangles);

		ArrayList<SingleBodyContact> contacts = wrapper.getMeshContacts();

		contacts.removeIf(contact -> {
			if (!intersectedTriangles.remove(contact.getTriangle())) {
				contact.refresh(null, null);
				meshContactPool[contact.getMaxContacts()].add(contact);
				return true;
			}
			return false;
		});
		intersectedTriangles.forEach(triangle -> createMeshContact(wrapper, triangle));
	}

	private void deleteMeshContacts(Wrapper wrapper) {
		for (SingleBodyContact contact : wrapper.getMeshContacts()) {
			contact.refresh(null, null);
			meshContactPool[contact.getMaxContacts()].add(contact);
		}
		wrapper.getMeshContacts().clear();
	}

	private void deleteBodyContacts(Wrapper wrapper) {
		for (DoubleBodyContact contact : wrapper.getBodyContacts()) {
			Wrapper other = contact.getWrapperA().getID() == wrapper.getID() ? contact.getWrapperB()
					: contact.getWrapperA();
			other.getBodyContacts().remove(contact);
			contact.refresh(null, null);
			bodyContactPool[contact.getMaxContacts()].add(contact);
		}
		wrapper.getBodyContacts().clear();
	}

	private void createBodyContact(Wrapper wrapperA, Wrapper wrapperB) {
		if (!filter.canCollide(wrapperA.getBody(), wrapperB.getBody())) {
			return;
		}

		if (wrapperA.getType().maxContacts > wrapperB.getType().maxContacts) {
			Wrapper temp = wrapperA;
			wrapperA = wrapperB;
			wrapperB = temp;
		}

		int maxContacts = wrapperA.getType().maxContacts;
		ArrayList<DoubleBodyContact> pool = bodyContactPool[maxContacts];
		DoubleBodyContact contact = null;
		if (pool.isEmpty()) {
			contact = new DoubleBodyContact(maxContacts, wrapperA, wrapperB);
		} else {
			contact = pool.remove(pool.size() - 1);
			contact.refresh(wrapperA, wrapperB);
		}
		wrapperA.getBodyContacts().add(contact);
		wrapperB.getBodyContacts().add(contact);

	}

	private void createMeshContact(Wrapper wrapper, Triangle triangle) {
		int maxContacts = wrapper.getType().maxContacts;
		ArrayList<SingleBodyContact> pool = meshContactPool[maxContacts];
		SingleBodyContact contact = null;
		if (pool.isEmpty()) {
			contact = new SingleBodyContact(maxContacts, wrapper, triangle);
		} else {
			contact = pool.remove(pool.size() - 1);
			contact.refresh(wrapper, triangle);
		}
		wrapper.getMeshContacts().add(contact);
	}

}
