package cataclysm.wrappers;

import java.util.ArrayList;
import java.util.List;

import org.lwjgl.util.vector.Matrix4f;

import cataclysm.DefaultParameters;
import cataclysm.PhysicsStats;
import cataclysm.PhysicsWorld;
import cataclysm.broadphase.staticmeshes.StaticMeshManager;
import cataclysm.constraints.AbstractConstraint;
import cataclysm.constraints.AnchorPoint;
import cataclysm.contact_creation.DoubleBodyContact;
import cataclysm.contact_creation.SingleBodyContact;
import cataclysm.datastructures.BufferedManager;

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
	
	/**
	 * La liste des contacts Wrapper vs Triangle donnant lieu à une pénétration des
	 * solides.
	 */
	private final List<SingleBodyContact> meshContacts = new ArrayList<SingleBodyContact>();

	/**
	 * La liste des contacts Wrapper vs Wrapper donnant lieu à une pénétration des
	 * solides.
	 */
	private final List<DoubleBodyContact> bodyContacts = new ArrayList<DoubleBodyContact>();

	
	public RigidBodyManager(PhysicsWorld world, StaticMeshManager meshes, PhysicsStats stats) {
		this.world = world;
		this.meshes = meshes;
		this.stats = stats;
		this.updator = new RigidBodyManagerUpdate(world.getParameters().getCollisionFilter(), world.getParameters().getPadding());
	}

	/**
	 * Ajoute un corps rigide dans la simulation.
	 * 
	 * @param transform La position et la rotation de l'objet en world-space.
	 * @param params    Les paramêtres par défaut
	 * @param builders  Les enveloppes de l'objet.
	 * 
	 * @return L'objet nouvellement créé.
	 */
	public RigidBody newBody(Matrix4f transform, DefaultParameters params, WrapperBuilder... builders) {
		RigidBody body = new RigidBody(transform, params, this.generator, builders);
		addElement(body);
		return body;
	}

	@Override
	protected void processAddedAndRemovedElements(List<RigidBody> added, List<RigidBody> removed) {
		updator.processAddedAndRemovedElements(added, removed);
	}

	@Override
	protected void internalUpdate() {
		updator.updateBodies(this, meshes, world.getParameters().getCallbacks(), stats, meshContacts, bodyContacts);
	}
	
	@Override
	protected void cleanAddedAndRemovedElements(List<RigidBody> added, List<RigidBody> removed) {
		List<AbstractConstraint> contraintsToDelete = new ArrayList<AbstractConstraint>();
		for (RigidBody body : removed) {
			for (Wrapper w : body.getWrappers())
				generator.freeID(w.getID());
			for (AnchorPoint point : body.getAnchorPoints()) {
				contraintsToDelete.add(point.getConstraint());
			}
		}
		super.cleanAddedAndRemovedElements(added, removed);

		contraintsToDelete.forEach(constraint -> world.deleteConstraint(constraint));
	}



	/**
	 * Ajoute les points d'ancrage de la contrainte aux rigidbody reli�s par la
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
	 * Retire les points d'ancrage de la contrainte des rigidbody reli�s par la
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
	
	public List<SingleBodyContact> getMeshContacts() {
		return meshContacts;
	}

	public List<DoubleBodyContact> getBodyContacts() {
		return bodyContacts;
	}

	@Override
	public void cleanUp() {
		super.cleanUp();
	}

}
