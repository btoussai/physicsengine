package cataclysm.wrappers;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;

import cataclysm.GeometryQuery;
import cataclysm.PhysicsStats;
import cataclysm.PhysicsWorld;
import cataclysm.RayTest;
import cataclysm.broadphase.AABB;
import cataclysm.broadphase.ArrayBasedBroadPhaseTree;
import cataclysm.broadphase.BroadPhaseTree;
import cataclysm.broadphase.staticmeshes.StaticMeshManager;
import cataclysm.broadphase.staticmeshes.Triangle;
import cataclysm.constraints.AbstractConstraint;
import cataclysm.constraints.AnchorPoint;
import cataclysm.contact_creation.AbstractDoubleBodyContact;
import cataclysm.contact_creation.AbstractSingleBodyContact;
import cataclysm.datastructures.BufferedManager;
import cataclysm.datastructures.IDGenerator;
import cataclysm.parallel.PhysicsWorkerPool;
import cataclysm.record.RigidBodyRepr;
import math.vector.Matrix4f;

/**
 * Contient l'ensemble des objets de la simulation.
 * 
 * @author Briac
 *
 */
public class RigidBodyManager extends BufferedManager<RigidBody> implements GeometryQuery{

	private final BodyUpdator updator;

	private final PhysicsWorld world;

	private final StaticMeshManager meshes;

	private final PhysicsStats stats;

	private final PolyhedralMassProperties poly = new PolyhedralMassProperties();
	
	private final IDGenerator wrapperGenerator = new IDGenerator();

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

	private Consumer<RigidBody> callbackOnAdd;

	private Consumer<RigidBody> callbackOnRemove;

	public RigidBodyManager(PhysicsWorld world, StaticMeshManager meshes, PhysicsStats stats) {
		this.world = world;
		this.meshes = meshes;
		this.stats = stats;
		this.updator = new RigidBodyManagerUpdate(world.getParameters().getCollisionFilter(),
				world.getParameters().getPadding());
	}

	public RigidBodyManager(PhysicsWorld world, StaticMeshManager meshes, PhysicsStats stats,
			PhysicsWorkerPool workers) {
		this.world = world;
		this.meshes = meshes;
		this.stats = stats;
		this.updator = new RigidBodyManagerParallelUpdate(workers, world.getParameters().getCollisionFilter(),
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
		RigidBody body = new RigidBody(transform, world.getParameters(), this.generator, wrapperGenerator, poly, builders);
		addElement(body);
		return body;
	}

	public RigidBody newBody(RigidBodyRepr repr) {
		RigidBody body = new RigidBody(world.getParameters(), this.generator, wrapperGenerator, repr);
		addElement(body);
		return body;
	}

	@Override
	protected void processAddedAndRemovedElements(List<RigidBody> added, List<RigidBody> removed) {
		if (callbackOnRemove != null)
			removed.forEach(callbackOnRemove);
		if (callbackOnAdd != null)
			added.forEach(callbackOnAdd);

		updator.processAddedAndRemovedElements(added, removed, meshes);

		if (world.getActiveRecord() != null) {
			world.getUpdateStats().physicsRecorder.start();
			world.getActiveRecord().getCurrentFrame().fillBodies(added, removed);
			world.getUpdateStats().physicsRecorder.pause();
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
		wrapperGenerator.reset();
	}

	public Consumer<RigidBody> getCallbackOnAdd() {
		return callbackOnAdd;
	}

	public void setCallbackOnAdd(Consumer<RigidBody> callbackOnAdd) {
		this.callbackOnAdd = callbackOnAdd;
	}

	public Consumer<RigidBody> getCallbackOnRemove() {
		return callbackOnRemove;
	}

	public void setCallbackOnRemove(Consumer<RigidBody> callbackOnRemove) {
		this.callbackOnRemove = callbackOnRemove;
	}

	public List<RigidBody> getElements() {
		return super.elements;
	}

	@Override
	protected void processAddedAndRemovedElements(List<RigidBody> added, List<RigidBody> removed,
			PhysicsWorkerPool workers) {
		processAddedAndRemovedElements(added, removed);
	}

	@Override
	protected void internalUpdate(PhysicsWorkerPool workers) {
		internalUpdate();
	}

//	public ArrayBasedBroadPhaseTree<Wrapper> getBVH() {
//		return updator.getBVH();
//	}
	
	public BroadPhaseTree<Wrapper> getBVH() {
		return updator.getBVH();
	}

	@Override
	public void rayTest(RayTest test) {
		updator.rayTest(test);
	}

	@Override
	public void boxTriangleQuery(AABB box, Set<Triangle> set) {
		updator.boxTriangleQuery(box, set);
	}

	@Override
	public void boxWrapperQuery(AABB box, Set<Wrapper> set) {
		updator.boxWrapperQuery(box, set);
	}

}
