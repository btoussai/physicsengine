package cataclysm.contact_creation;

import java.util.Comparator;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.function.Consumer;

import org.lwjgl.util.vector.Vector3f;

import cataclysm.CataclysmCallbacks;
import cataclysm.PhysicsStats;
import cataclysm.broadphase.staticmeshes.Triangle;
import cataclysm.wrappers.CapsuleWrapper;
import cataclysm.wrappers.ConvexHullWrapper;
import cataclysm.wrappers.ConvexHullWrapperHalfEdge;
import cataclysm.wrappers.SphereWrapper;
import cataclysm.wrappers.TriangleAsHull;
import cataclysm.wrappers.Wrapper;

/**
 * Permet de tester la collision entre un wrapper et un ensemble de triangles.
 * 
 * @author Briac
 *
 */
class CollideWrapperTriangle {

	/**
	 * Un wrapper pour {@link Vector3f} avec l'ajout d'une surcharge de
	 * {@link Object#hashCode()} dans le but de placer les vecteurs dans une table
	 * de hashage.
	 * 
	 * @author Briac
	 *
	 */
	private static class VectorRepr {

		private Vector3f v;

		VectorRepr(Vector3f v) {
			this.v = v;
		}

		void set(Vector3f v) {
			if(v == null) {
				throw new NullPointerException("Le vecteur est null.");
			}
			this.v = v;
		}

		@Override
		public boolean equals(Object o) {
			return v.equals(v);
		}

		@Override
		public int hashCode() {
			return 31 * (31 * Float.floatToRawIntBits(v.z) + Float.floatToRawIntBits(v.y))
					+ Float.floatToRawIntBits(v.z);
		}

	}

	/**
	 * Une variable de travail
	 */
	private static VectorRepr vrepr = new VectorRepr(new Vector3f());

	/**
	 * La liste des sommets d�j� trait�s. Si un contact r�p�torie un sommet d�j�
	 * trait�, celui-ci sera ignor�.
	 */
	private static HashSet<VectorRepr> voidedFeatures = new HashSet<VectorRepr>();

	/**
	 * Permet de transformer les triangles en enveloppe convexe.
	 */
	private static final TriangleAsHull triangleHull = TriangleAsHull.buildNew();

	/**
	 * Teste la collision entre une enveloppe convexe et un ensemble de triangles.
	 * 
	 * @param wrapper
	 * @param triangles
	 * @param callbacks
	 * @param stats 
	 * @param meshContactsDest 
	 */
	public static void test(Wrapper wrapper, HashSet<Triangle> triangles, CataclysmCallbacks callbacks, PhysicsStats stats, List<SingleBodyContact> meshContactsDest) {
		
		//System.out.println("Wrapper vs Triangle");
		
		voidedFeatures.clear();

		buildNewContacts(wrapper, triangles);
		
		List<SingleBodyContact> contacts = wrapper.getMeshContacts();
		contacts.sort(Comparator.comparingDouble((c) -> c.area.getPenetrationDepth()));
		
		stats.bodyToMeshContacts += contacts.size();

		for (SingleBodyContact contact : contacts) {
			if(!contact.area.isCollisionOccuring()) {
				break;
			}
			
			boolean featureAlreadyProcessed = false;
			ContactFeature feature = contact.area.getFeatureOnB();
			switch (feature.getType()) {
			//Updated from SAT
			case Face:
				featureAlreadyProcessed = isFaceProcessed(contact.getTriangle());
				break;
			case HalfEdge:
				featureAlreadyProcessed = isHalfEdgeProcessed(feature.getHalfedge());
				break;
			case None:
				//Attention, ce code est faux dans le cas g�n�ral :(
				featureAlreadyProcessed = isFaceProcessed(contact.getTriangle());
				break;
			//Updated from GJK
			case Triangle:
				featureAlreadyProcessed = isTriangleProcessed(feature.getV1(), feature.getV2(), feature.getV3());
				break;
			case Segment:
				featureAlreadyProcessed = isSegmentProcessed(feature.getV1(), feature.getV2());
				break;
			case Vertex:
				featureAlreadyProcessed = isVertexProcessed(feature.getV1());
				break;
			default:
				break;
			}

			if (featureAlreadyProcessed) {
				//contact.area.setNoCollision();
			}
			
			if(callbacks.getOnCollisionWithGround() != null) {
				callbacks.getOnCollisionWithGround().accept(wrapper);
			}

			markTriangleAsProcessed(contact.getTriangle());
			
			meshContactsDest.add(contact);
		}

	}

	/**
	 * Vide {@link CollideWrapper#newContacts} puis parcourt les triangles pour
	 * d�terminer les nouveaux contacts � ajouter dans
	 * {@link CollideWrapper#newContacts}
	 * 
	 * @param wrapper
	 * @param triangles
	 */
	private static void buildNewContacts(Wrapper wrapper, HashSet<Triangle> triangles) {
		Consumer<ContactArea> collisionTest = null;
		
		switch (wrapper.getType()) {
		case Sphere:
			collisionTest = (ContactArea contact) -> CollideSphereHull.test((SphereWrapper)wrapper, triangleHull, contact);
			break;
		case Capsule:
			collisionTest = (ContactArea contact) -> CollideCapsuleHull.test((CapsuleWrapper)wrapper, triangleHull, contact);
			break;
		case ConvexHull:
			collisionTest = (ContactArea contact) -> CollideHulls.test((ConvexHullWrapper)wrapper, triangleHull, contact);
			break;
		default:
			break;
		}
		

		// On parcourt les contacts d�j� connus pour les actualiser.
		List<SingleBodyContact> contacts = wrapper.getMeshContacts();
		Iterator<SingleBodyContact> it = contacts.iterator();
		while (it.hasNext()) {
			SingleBodyContact contact = it.next();
			Triangle triangle = contact.getTriangle();
			boolean present = triangles.remove(triangle);
			if (present) {//update du contact
				triangleHull.setFrom(triangle);
				collisionTest.accept(contact.area);
			} else {
				it.remove();
			}
		}

		//On parcourt les triangles inconnus et on construit de nouveau contacts.
		for (Triangle triangle : triangles) {
			triangleHull.setFrom(triangle);
			SingleBodyContact contact = new SingleBodyContact(wrapper.getType().maxContacts, wrapper.getBody());
			collisionTest.accept(contact.area);
			contact.setTriangle(triangle);
			contacts.add(contact);
		}

	}

	private static boolean isVertexProcessed(Vector3f v) {
		vrepr.set(v);
		return voidedFeatures.contains(vrepr);
	}
	
	private static boolean isSegmentProcessed(Vector3f v1, Vector3f v2) {
		if (isVertexProcessed(v1)) {
			return true;
		} else {
			return isVertexProcessed(v2);
		}
	}

	private static boolean isTriangleProcessed(Vector3f v1, Vector3f v2, Vector3f v3) {
		if (isVertexProcessed(v1)) {
			return true;
		} else if (isVertexProcessed(v2)) {
			return true;
		} else {
			return isVertexProcessed(v3);
		}
	}
	
	private static boolean isFaceProcessed(Triangle triangle) {
		if (isVertexProcessed(triangle.v1)) {
			return true;
		} else if (isVertexProcessed(triangle.v2)) {
			return true;
		} else {
			return isVertexProcessed(triangle.v3);
		}
	}
	
	private static boolean isHalfEdgeProcessed(ConvexHullWrapperHalfEdge edge) {
		if (isVertexProcessed(edge.getTail())) {
			return true;
		} else {
			return isVertexProcessed(edge.getHead());
		}
	}

	private static void markTriangleAsProcessed(Triangle triangle) {
		voidedFeatures.add(new VectorRepr(triangle.v1));
		voidedFeatures.add(new VectorRepr(triangle.v2));
		voidedFeatures.add(new VectorRepr(triangle.v3));
	}

}