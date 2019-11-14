package cataclysm.contact_creation;

import java.util.HashSet;
import java.util.List;

import cataclysm.CataclysmCallbacks;
import cataclysm.PhysicsStats;
import cataclysm.broadphase.Pair;
import cataclysm.broadphase.PairManager;
import cataclysm.broadphase.staticmeshes.StaticMeshManager;
import cataclysm.broadphase.staticmeshes.Triangle;
import cataclysm.wrappers.CapsuleWrapper;
import cataclysm.wrappers.ConvexHullWrapper;
import cataclysm.wrappers.RigidBody;
import cataclysm.wrappers.RigidBodyManager;
import cataclysm.wrappers.SphereWrapper;
import cataclysm.wrappers.Wrapper;

/**
 * Cette classe permet de tester la collision de deux solides et construit une
 * zone de contact le cas �ch�ant.
 * 
 * @author Briac
 *
 */
public class CollisionTest {

	/**
	 * Instancie un nouveau testeur de collision. Il permet de construire une zone
	 * de contact entre deux solides ou entre un solide et un maillage statique.
	 */
	public CollisionTest() {

	}

	/**
	 * Teste les collisions entre les rigidbody et les staticmesh.
	 * 
	 * @param bodies
	 * @param meshes
	 * @param callbacks
	 * @param stats
	 * @param meshContactsDest 
	 */
	public void meshContacts(RigidBodyManager bodies, StaticMeshManager meshes, CataclysmCallbacks callbacks,
			PhysicsStats stats, List<SingleBodyContact> meshContactsDest) {
		meshContactsDest.clear();
		
		HashSet<Triangle> triangles = new HashSet<Triangle>();

		for (RigidBody body : bodies) {
			if (body.isSleeping() || body.getInvMass() == 0) {
				continue;
			}
			for (Wrapper wrapper : body) {
				triangles.clear();
				meshes.boxTest(wrapper.getNode().getBox(), triangles);

				// System.out.println("Triangles touch�s: " + triangles.size());

				CollideWrapperTriangle.test(wrapper, triangles, callbacks, stats, meshContactsDest);
				
			}
		}
		
		stats.bodyToMeshActiveContacts = meshContactsDest.size();
	}

	/**
	 * Teste les collisions entre les paires d'objets et construit des zones de
	 * contact le cas �ch�ant.
	 * 
	 * @param pairs
	 * @param callbacks
	 * @param stats
	 * @param bodyContacts 
	 */
	public void convexContacts(PairManager pairs, CataclysmCallbacks callbacks, PhysicsStats stats, List<DoubleBodyContact> bodyContacts) {
		
		bodyContacts.clear();

		if (callbacks.getOnCollision() != null) {
			for (Pair pair : pairs) {
				if (pair.isSleeping())
					continue;
				ContactArea area = pair.getContact().area;
				convexContact(pair.getWrapperA(), pair.getWrapperB(), area);
				if (area.isCollisionOccuring()) {
					bodyContacts.add(pair.getContact());
					callbacks.getOnCollision().accept(pair.getWrapperA(), pair.getWrapperB());
				}
			}
		} else {
			
			for (Pair pair : pairs) {
				if (pair.isSleeping())
					continue;
				ContactArea area = pair.getContact().area;
				convexContact(pair.getWrapperA(), pair.getWrapperB(), area);
				if (area.isCollisionOccuring()) {
					bodyContacts.add(pair.getContact());
				}
			}
			
		}
		
		stats.bodyToBodyContacts = pairs.size();
		stats.bodyToBodyActiveContacts = bodyContacts.size();
	}

	/**
	 * Teste si deux solides convexes sont en contact et construit une zone de
	 * contact le cas �ch�ant.
	 * 
	 * @param A Le premier solide.
	 * @param B Le second solide.
	 */
	private void convexContact(Wrapper A, Wrapper B, ContactArea contact) {

		// System.out.println("Wrapper vs Wrapper");

		switch (A.getType()) {
		case Sphere:
			if (B.getType() == Wrapper.Type.Sphere) {
				CollideSpheres.test((SphereWrapper) A, (SphereWrapper) B, contact);
				break;
			} else if (B.getType() == Wrapper.Type.Capsule) {
				CollideSphereCapsule.test((SphereWrapper) A, (CapsuleWrapper) B, contact);
				break;
			} else {
				CollideSphereHull.test((SphereWrapper) A, (ConvexHullWrapper) B, contact);
				break;
			}
		case Capsule:
			if (B.getType() == Wrapper.Type.Capsule) {
				CollideCapsules.test((CapsuleWrapper) A, (CapsuleWrapper) B, contact);
				break;
			} else {
				CollideCapsuleHull.test((CapsuleWrapper) A, (ConvexHullWrapper) B, contact);
				break;
			}
		case ConvexHull:
			CollideHulls.test((ConvexHullWrapper) A, (ConvexHullWrapper) B, contact);
			break;
		}

	}

}
