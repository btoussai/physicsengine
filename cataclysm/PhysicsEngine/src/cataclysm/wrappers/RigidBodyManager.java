package cataclysm.wrappers;

import java.util.ArrayList;
import java.util.List;

import cataclysm.PhysicsStats;
import cataclysm.PhysicsWorld;
import cataclysm.broadphase.staticmeshes.StaticMeshManager;
import cataclysm.constraints.AbstractConstraint;
import cataclysm.constraints.AnchorPoint;
import cataclysm.contact_creation.AbstractDoubleBodyContact;
import cataclysm.contact_creation.AbstractSingleBodyContact;
import cataclysm.datastructures.BufferedManager;
import cataclysm.record.RigidBodyRepr;
import math.vector.Matrix4f;

/**
 * Contient l'ensemble des objets de la simulation.
 * 
 * @author Briac
 *
 */
public class RigidBodyManager extends BufferedManager<RigidBody> {

	private final RigidBodyManagerUpdate updator;

	private final PhysicsWorld world;

	private final StaticMeshManager meshes;

	private final PhysicsStats stats;

	private final PolyhedralMassProperties poly = new PolyhedralMassProperties();

	/**
	 * La liste des contacts Wrapper vs Triangle donnant lieu à une pénétration des
	 * solides.
	 */
	private final List<AbstractSingleBodyContact> meshContacts = new ArrayList<AbstractSingleBodyContact>();

	/**
	 * La liste des contacts Wrapper vs Wrapper donnant lieu à une pénétration des
	 * solides.
	 */
	private final List<AbstractDoubleBodyContact> bodyContacts = new ArrayList<AbstractDoubleBodyContact>();

	public RigidBodyManager(PhysicsWorld world, StaticMeshManager meshes, PhysicsStats stats) {
		this.world = world;
		this.meshes = meshes;
		this.stats = stats;
		this.updator = new RigidBodyManagerUpdate(world.getParameters().getCollisionFilter(),
				world.getParameters().getPadding());
	}

	/**
	 * Ajoute un corps rigide dans la simulation.
	 * 
	 * @param transform La position et la rotation de l'objet en world-space.
	 * @param builders  Les enveloppes de l'objet.
	 * 
	 * @return L'objet nouvellement créé.
	 */
	public RigidBody newBody(Matrix4f transform, WrapperBuilder... builders) {
		RigidBody body = new RigidBody(transform, world.getParameters(), this.generator, poly, builders);
		addElement(body);
		return body;
	}

	public RigidBody newBody(RigidBodyRepr repr) {
		RigidBody body = new RigidBody(world.getParameters(), this.generator, repr);
		addElement(body);
		return body;
	}

	@Override
	protected void processAddedAndRemovedElements(List<RigidBody> added, List<RigidBody> removed) {
		updator.processAddedAndRemovedElements(added, removed, meshes);

		if (world.getActiveRecord() != null) {
			world.getActiveRecord().getCurrentFrame().fillBodies(added, removed);
		}

		List<AbstractConstraint> contraintsToDelete = new ArrayList<AbstractConstraint>();
		for (RigidBody body : removed) {
			for (AnchorPoint point : body.getAnchorPoints()) {
				contraintsToDelete.add(point.getConstraint());
			}
		}
		contraintsToDelete.forEach(constraint -> world.deleteConstraint(constraint));
	}

	@Override
	protected void internalUpdate() {
		updator.updateBodies(this, meshes, world.getParameters().getCallbacks(), stats, meshContacts, bodyContacts);
	}

	/**
	 * Ajoute les points d'ancrage de la contrainte aux rigidbody reliés par la
	 * constrainte.
	 * 
	 * @param constraint
	 */
	public void addConstraint(AbstractConstraint constraint) {
		AnchorPoint pointA = constraint.getPointA();
		if (!pointA.isStatic()) {
			pointA.getBody().addAnchorPoint(pointA);
		}
		AnchorPoint pointB = constraint.getPointB();
		if (!pointB.isStatic()) {
			pointB.getBody().addAnchorPoint(pointB);
		}
	}

	/**
	 * Retire les points d'ancrage de la contrainte des rigidbody reliés par la
	 * constrainte.
	 * 
	 * @param constraint
	 */
	public void removeConstraint(AbstractConstraint constraint) {
		AnchorPoint pointA = constraint.getPointA();
		if (!pointA.isStatic()) {
			pointA.getBody().removeAnchorPoint(pointA);
		}
		AnchorPoint pointB = constraint.getPointA();
		if (!pointB.isStatic()) {
			pointB.getBody().removeAnchorPoint(pointB);
		}
	}

	public List<AbstractSingleBodyContact> getMeshContacts() {
		return meshContacts;
	}

	public List<AbstractDoubleBodyContact> getBodyContacts() {
		return bodyContacts;
	}

	@Override
	public void cleanUp() {
		super.cleanUp();
	}

}
