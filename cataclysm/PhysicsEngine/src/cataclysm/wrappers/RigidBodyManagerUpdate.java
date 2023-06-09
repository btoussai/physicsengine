package cataclysm.wrappers;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import cataclysm.CataclysmCallbacks;
import cataclysm.CollisionFilter;
import cataclysm.PhysicsStats;
import cataclysm.RayTest;
import cataclysm.broadphase.AABB;
import cataclysm.broadphase.ArrayBasedBroadPhaseTree;
import cataclysm.broadphase.BroadPhaseNode;
import cataclysm.broadphase.BroadPhaseTree;
import cataclysm.broadphase.staticmeshes.StaticMeshManager;
import cataclysm.broadphase.staticmeshes.Triangle;
import cataclysm.contact_creation.AbstractDoubleBodyContact;
import cataclysm.contact_creation.AbstractSingleBodyContact;
import cataclysm.contact_creation.CollisionTest;
import math.vector.Vector3f;

/**
 * Cette classe permet d'effectuer la broadphase et la narrow phase en une même
 * passe.
 * 
 * @author Briac
 *
 */
class RigidBodyManagerUpdate extends BodyUpdator {

	private final HashSet<Wrapper> intersectedWrappers = new HashSet<Wrapper>();
	private final HashSet<Triangle> intersectedTriangles = new HashSet<Triangle>();

	private final BroadPhaseTree<Wrapper> bvh = new BroadPhaseTree<Wrapper>();
//	private final ArrayBasedBroadPhaseTree<Wrapper> bvh = new ArrayBasedBroadPhaseTree<Wrapper>((i) -> new Wrapper[i]);

	private final float PADDING;
	private final float PADDING_SQUARED;

	private final CollisionTest collisionTest = new CollisionTest();
	
	private boolean firstUpdate = true;

	RigidBodyManagerUpdate(CollisionFilter filter, float padding) {
		super(filter);
		this.PADDING = padding;
		this.PADDING_SQUARED = padding * padding;
	}

	@Override
	public void processAddedAndRemovedElements(List<RigidBody> added, List<RigidBody> removed,
			StaticMeshManager meshes) {

		for (RigidBody body : removed) {
			for (Wrapper wrapper : body.getWrappers()) {
				deleteMeshContacts(wrapper);
				deleteBodyContacts(wrapper);
				bvh.remove(wrapper.getNode());
			}
		}

		for (RigidBody body : added) {
			for (Wrapper wrapper : body.getWrappers()) {
				
				wrapper.placeBox(PADDING);
				bvh.add(wrapper.getNode());
//				AABB box = wrapper.getBox();
//				wrapper.setNode(bvh.add(box.minX, box.minY, box.minZ, box.maxX, box.maxY, box.maxZ, wrapper));
				
				
				recomputeDoubleBodyContactList(wrapper);
				recomputeSingleBodyContactList(wrapper, meshes);
			}
		}

	}

	@Override
	public void updateBodies(RigidBodyManager bodies, StaticMeshManager meshes, CataclysmCallbacks callbacks,
			PhysicsStats stats, List<AbstractSingleBodyContact> meshContacts,
			List<AbstractDoubleBodyContact> bodyContacts) {
		meshContacts.clear();
		bodyContacts.clear();
		stats.bodyToMeshContacts = 0;
		stats.bodyToBodyContacts = 0;
		
		if(firstUpdate) {
			firstUpdate = false;
			for (RigidBody body : bodies) {
				ArrayList<Wrapper> wrappers = body.getWrappers();
				for (int i = 0; i < wrappers.size(); i++) {
					Wrapper wrapper = wrappers.get(i);
					updateWrapper(body.getInvMass() == 0, wrapper, meshes, callbacks, stats, meshContacts, bodyContacts);
				}
			}
		}else {
			for (RigidBody body : bodies) {
				if (body.isSleeping())
					continue;
				ArrayList<Wrapper> wrappers = body.getWrappers();
				for (int i = 0; i < wrappers.size(); i++) {
					Wrapper wrapper = wrappers.get(i);
					updateWrapper(body.getInvMass() == 0, wrapper, meshes, callbacks, stats, meshContacts, bodyContacts);
				}
			}
		}

		stats.bodyToBodyContacts /= 2;
		stats.bodyToBodyActiveContacts = bodyContacts.size();
		stats.bodyToMeshActiveContacts = meshContacts.size();
	}

	private void updateWrapper(boolean isKinematic, Wrapper wrapper, StaticMeshManager meshes,
			CataclysmCallbacks callbacks, PhysicsStats stats, List<AbstractSingleBodyContact> meshContacts,
			List<AbstractDoubleBodyContact> bodyContacts) {
//		AABB box = wrapper.getBox();
		AABB box = wrapper.getNode().getBox();
		Vector3f centroid = wrapper.getCentroid();

		float sx = 0.5f * (box.minX + box.maxX) - centroid.x;
		float sy = 0.5f * (box.minY + box.maxY) - centroid.y;
		float sz = 0.5f * (box.minZ + box.maxZ) - centroid.z;

		float d2 = sx * sx + sy * sy + sz * sz;

		if (d2 > PADDING_SQUARED) {
			recomputeDoubleBodyContactList(wrapper);
			recomputeSingleBodyContactList(wrapper, meshes);
		}

		stats.bodyToMeshContacts += wrapper.getMeshContacts().size();

		if (!isKinematic)
			collisionTest.meshContacts(wrapper, callbacks, meshContacts);

		ArrayList<AbstractDoubleBodyContact> contacts = wrapper.getBodyContacts();
		if (contacts.isEmpty()) {
			return;
		}
		stats.bodyToBodyContacts += contacts.size();

		for (int j = 0; j < contacts.size(); j++) {
			AbstractDoubleBodyContact contact = contacts.get(j);

			if(contact.getOther(wrapper).getBody().isSleeping()) {
				//we update the contact when one body is sleeping
				collisionTest.bodyContacts(contact, callbacks, bodyContacts);
			}else {
				// we update the contact if both bodies are updated, which means the flag
				// will be true the second time the function is called
				if (contact.getUpdateFlagAndFlip()) {
				collisionTest.bodyContacts(contact, callbacks, bodyContacts);
				}
			}
		}
	}

	private void recomputeDoubleBodyContactList(Wrapper wrapper) {
		intersectedWrappers.clear();

		BroadPhaseNode<Wrapper> node = wrapper.getNode();
		wrapper.placeBox(PADDING);
		bvh.update(node);
		bvh.boxTest(node.getBox(), intersectedWrappers);
		
//		final int node = wrapper.getNode();
//		wrapper.placeBox(PADDING);
//		bvh.update(node, wrapper.getBox());
//		bvh.boxTest(wrapper.getBox(), intersectedWrappers);
		
		intersectedWrappers.remove(wrapper);

		ArrayList<AbstractDoubleBodyContact> contacts = wrapper.getBodyContacts();

		contacts.removeIf(contact -> {
			Wrapper other = contact.getOther(wrapper);
			if (!intersectedWrappers.remove(other)) {
				other.getBodyContacts().remove(contact);
				contact.refresh(null, null);
				bodyContactPool[contact.getMaxContacts()].add(contact);
				return true;
			}
			return false;
		});

		for (Wrapper other : intersectedWrappers) {
			AbstractDoubleBodyContact contact = createBodyContact(wrapper, other);
			if (contact != null) {
				wrapper.getBodyContacts().add(contact);
				other.getBodyContacts().add(contact);
			}
		}
	}

	private void recomputeSingleBodyContactList(Wrapper wrapper, StaticMeshManager meshes) {

		if (wrapper.getBody().getInvMass() == 0) {
			return;
		}

		intersectedTriangles.clear();
//		meshes.boxTriangleQuery(wrapper.getBox(), intersectedTriangles);
		meshes.boxTriangleQuery(wrapper.getNode().getBox(), intersectedTriangles);

		ArrayList<AbstractSingleBodyContact> contacts = wrapper.getMeshContacts();

		contacts.removeIf(contact -> {
			if (!intersectedTriangles.remove(contact.getTriangle())) {
				contact.getTriangle().mesh.getBodyContacts().remove(contact);
				contact.refresh(null, null);
				meshContactPool[contact.getMaxContacts()].add(contact);
				return true;
			}
			return false;
		});
		intersectedTriangles.forEach(triangle -> createMeshContact(wrapper, triangle));
	}

//	@Override
//	public ArrayBasedBroadPhaseTree<Wrapper> getBVH() {
//		return bvh;
//	}
	
	@Override
	public BroadPhaseTree<Wrapper> getBVH() {
		return bvh;
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

}
