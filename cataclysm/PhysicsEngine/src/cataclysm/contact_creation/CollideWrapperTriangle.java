package cataclysm.contact_creation;

import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.function.Consumer;

import cataclysm.CataclysmCallbacks;
import cataclysm.broadphase.staticmeshes.Triangle;
import cataclysm.wrappers.CapsuleWrapper;
import cataclysm.wrappers.ConvexHullWrapper;
import cataclysm.wrappers.ConvexHullWrapperHalfEdge;
import cataclysm.wrappers.SphereWrapper;
import cataclysm.wrappers.TriangleAsHull;
import cataclysm.wrappers.Wrapper;
import math.vector.Vector3f;

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
	private final VectorRepr vrepr = new VectorRepr(new Vector3f());

	/**
	 * La liste des sommets d�j� trait�s. Si un contact r�p�torie un sommet d�j�
	 * trait�, celui-ci sera ignor�.
	 */
	private final HashSet<VectorRepr> voidedFeatures = new HashSet<VectorRepr>();

	/**
	 * Permet de transformer les triangles en enveloppe convexe.
	 */
	private final TriangleAsHull triangleHull = TriangleAsHull.buildNew();
	
	private final CollideSphereHull collideSphereHull = new CollideSphereHull();
	private final CollideCapsuleHull collideCapsuleHull = new CollideCapsuleHull();
	private final CollideHulls collideHulls = new CollideHulls();

	/**
	 * Teste la collision entre une enveloppe convexe et un ensemble de triangles.
	 * 
	 * @param wrapper
	 * @param callbacks
	 * @param meshContacts 
	 */
	public void test(Wrapper wrapper, CataclysmCallbacks callbacks, List<AbstractSingleBodyContact> meshContacts) {
		
		//System.out.println("Wrapper vs Triangle");
		
		voidedFeatures.clear();

		updateContacts(wrapper);
		
		List<AbstractSingleBodyContact> contacts = wrapper.getMeshContacts();
		contacts.sort(Comparator.comparingDouble((c) -> c.area.getPenetrationDepth()));

		for (AbstractSingleBodyContact contact : contacts) {
			if(!contact.area.isCollisionOccuring()) {
				contact.resetImpulses();
				continue;
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
			
			meshContacts.add(contact);
		}

	}

	/**
	 * Met à  jour les contacts avec les triangles de ce wrapper.
	 * 
	 * @param wrapper
	 */
	private void updateContacts(Wrapper wrapper) {
		final Consumer<ContactArea> collisionTest;
		switch (wrapper.getType()) {
		case Sphere:
			collisionTest = (ContactArea contact) -> collideSphereHull.test((SphereWrapper)wrapper, triangleHull, contact);
			break;
		case Capsule:
			collisionTest = (ContactArea contact) -> collideCapsuleHull.test((CapsuleWrapper)wrapper, triangleHull, contact);
			break;
		case ConvexHull:
			collisionTest = (ContactArea contact) -> collideHulls.test((ConvexHullWrapper)wrapper, triangleHull, contact);
			break;
		default:
			collisionTest = null;
			break;
		}
		
		for(AbstractSingleBodyContact contact : wrapper.getMeshContacts()) {
			triangleHull.setFrom(contact.getTriangle());
			collisionTest.accept(contact.area);
		}
	}

	private boolean isVertexProcessed(Vector3f v) {
		vrepr.set(v);
		return voidedFeatures.contains(vrepr);
	}
	
	private boolean isSegmentProcessed(Vector3f v1, Vector3f v2) {
		if (isVertexProcessed(v1)) {
			return true;
		} else {
			return isVertexProcessed(v2);
		}
	}

	private boolean isTriangleProcessed(Vector3f v1, Vector3f v2, Vector3f v3) {
		if (isVertexProcessed(v1)) {
			return true;
		} else if (isVertexProcessed(v2)) {
			return true;
		} else {
			return isVertexProcessed(v3);
		}
	}
	
	private boolean isFaceProcessed(Triangle triangle) {
		if (isVertexProcessed(triangle.v1)) {
			return true;
		} else if (isVertexProcessed(triangle.v2)) {
			return true;
		} else {
			return isVertexProcessed(triangle.v3);
		}
	}
	
	private boolean isHalfEdgeProcessed(ConvexHullWrapperHalfEdge edge) {
		if (isVertexProcessed(edge.getTail())) {
			return true;
		} else {
			return isVertexProcessed(edge.getHead());
		}
	}

	private void markTriangleAsProcessed(Triangle triangle) {
		voidedFeatures.add(new VectorRepr(triangle.v1));
		voidedFeatures.add(new VectorRepr(triangle.v2));
		voidedFeatures.add(new VectorRepr(triangle.v3));
	}

}
