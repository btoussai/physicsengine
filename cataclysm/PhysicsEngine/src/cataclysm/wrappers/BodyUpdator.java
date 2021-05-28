package cataclysm.wrappers;

import java.util.ArrayList;
import java.util.List;

import cataclysm.CataclysmCallbacks;
import cataclysm.CollisionFilter;
import cataclysm.Epsilons;
import cataclysm.GeometryQuery;
import cataclysm.PhysicsStats;
import cataclysm.broadphase.ArrayBasedBroadPhaseTree;
import cataclysm.broadphase.BroadPhaseTree;
import cataclysm.broadphase.staticmeshes.StaticMeshManager;
import cataclysm.broadphase.staticmeshes.Triangle;
import cataclysm.contact_creation.AbstractDoubleBodyContact;
import cataclysm.contact_creation.AbstractSingleBodyContact;
import cataclysm.contact_creation.DoubleBodyContact;
import cataclysm.contact_creation.DoubleBodyContactArrayBased;
import cataclysm.contact_creation.SingleBodyContact;
import cataclysm.contact_creation.SingleBodyContactArrayBased;

@SuppressWarnings("unchecked")
abstract class BodyUpdator implements GeometryQuery {

	protected final ArrayList<AbstractDoubleBodyContact>[] bodyContactPool = new ArrayList[Epsilons.MAX_CONTACTS + 1];

	protected final ArrayList<AbstractSingleBodyContact>[] meshContactPool = new ArrayList[Epsilons.MAX_CONTACTS + 1];

	private final CollisionFilter filter;

	public BodyUpdator(CollisionFilter filter) {
		this.filter = filter;
		for (int i = 1; i <= Epsilons.MAX_CONTACTS; i++) {
			bodyContactPool[i] = new ArrayList<AbstractDoubleBodyContact>();
			meshContactPool[i] = new ArrayList<AbstractSingleBodyContact>();
		}
	}

	abstract void processAddedAndRemovedElements(List<RigidBody> added, List<RigidBody> removed,
			StaticMeshManager meshes);

	abstract void updateBodies(RigidBodyManager bodies, StaticMeshManager meshes, CataclysmCallbacks callbacks,
			PhysicsStats stats, List<AbstractSingleBodyContact> meshContacts,
			List<AbstractDoubleBodyContact> bodyContacts);

	protected void deleteMeshContacts(Wrapper wrapper) {
		for (AbstractSingleBodyContact contact : wrapper.getMeshContacts()) {
			contact.refresh(null, null);
			meshContactPool[contact.getMaxContacts()].add(contact);
		}
		wrapper.getMeshContacts().clear();
	}

	protected void deleteBodyContacts(Wrapper wrapper) {
		for (AbstractDoubleBodyContact contact : wrapper.getBodyContacts()) {
			Wrapper other = contact.getOther(wrapper);
			other.getBodyContacts().remove(contact);
			contact.refresh(null, null);
			bodyContactPool[contact.getMaxContacts()].add(contact);
		}
		wrapper.getBodyContacts().clear();
	}

	protected AbstractDoubleBodyContact createBodyContact(Wrapper wrapperA, Wrapper wrapperB) {
		if (!filter.canCollide(wrapperA.getBody(), wrapperB.getBody())) {
			return null;
		}

		if (wrapperA.getType().maxContacts > wrapperB.getType().maxContacts) {
			Wrapper temp = wrapperA;
			wrapperA = wrapperB;
			wrapperB = temp;
		}

		int maxContacts = wrapperA.getType().maxContacts;
		ArrayList<AbstractDoubleBodyContact> pool = bodyContactPool[maxContacts];
		AbstractDoubleBodyContact contact = null;
		if (pool.isEmpty()) {
			switch (Epsilons.contactType) {
			case ARRAY_BASED:
				contact = new DoubleBodyContactArrayBased(maxContacts, wrapperA, wrapperB);
				break;
			case SIMPLE:
				contact = new DoubleBodyContact(maxContacts, wrapperA, wrapperB);
				break;
			default:
				throw new IllegalStateException("Invalid enum value: " + Epsilons.contactType);
			}
		} else {
			contact = pool.remove(pool.size() - 1);
			contact.refresh(wrapperA, wrapperB);
		}

		return contact;
	}

	protected void createMeshContact(Wrapper wrapper, Triangle triangle) {
		int maxContacts = wrapper.getType().maxContacts;
		ArrayList<AbstractSingleBodyContact> pool = meshContactPool[maxContacts];
		AbstractSingleBodyContact contact = null;
		if (pool.isEmpty()) {
			switch (Epsilons.contactType) {
			case ARRAY_BASED:
				contact = new SingleBodyContactArrayBased(maxContacts, wrapper, triangle);
				break;
			case SIMPLE:
				contact = new SingleBodyContact(maxContacts, wrapper, triangle);
				break;
			default:
				throw new IllegalStateException("Invalid enum value: " + Epsilons.contactType);
			}
		} else {
			contact = pool.remove(pool.size() - 1);
			contact.refresh(wrapper, triangle);
		}
		wrapper.getMeshContacts().add(contact);
	}

//	public abstract ArrayBasedBroadPhaseTree<Wrapper> getBVH();
	public abstract BroadPhaseTree<Wrapper> getBVH();

	public CollisionFilter getFilter() {
		return filter;
	}
}
