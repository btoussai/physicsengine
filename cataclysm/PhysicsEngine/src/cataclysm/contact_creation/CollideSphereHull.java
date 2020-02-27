package cataclysm.contact_creation;

import cataclysm.wrappers.ConvexHullWrapper;
import cataclysm.wrappers.ConvexHullWrapperFace;
import cataclysm.wrappers.SphereWrapper;
import math.vector.Vector3f;

/**
 * Permet de tester la collision entre une sphere et une enveloppe convexe ou un triangle.
 * @author Briac
 *
 */
class CollideSphereHull {

	private final Vector3f closest = new Vector3f();
	private final Vector3f normal = new Vector3f();
	
	private final ContactFeature onA = new ContactFeature();
	private final ContactFeature onB = new ContactFeature();
	
	private final GJK gjk = new GJK();

	void test(SphereWrapper sphere, ConvexHullWrapper hull, ContactArea contact) {

		float distance = gjk.distance(sphere, hull, null, closest);
		
		//System.out.println("Distance:" + distance);
		//System.out.println("closest:" + closest);

		if (distance < sphere.getRadius()) {

			if (distance > 0.0f) {// shallow contact

				Vector3f.sub(closest, sphere.getCentroid(), normal);
				normal.scale(1.0f / distance);
				
				gjk.getClosestFeatureOnB(onB);

			} else {

				distance = Float.NEGATIVE_INFINITY;
				ConvexHullWrapperFace referenceFace = null;
				for (ConvexHullWrapperFace face : hull.getFaces()) {
					float d = face.signedDistance(sphere.getCentroid());
					if (d > distance) {
						distance = d;
						referenceFace = face;
					}
				}

				normal.set(referenceFace.getNormal());
				normal.negate();
				onB.setFrom(referenceFace);
			}

			float depth = distance - sphere.getRadius();

			float toContactPoint = distance - 0.5f * depth;
			
			Vector3f contactPoint = contact.contactPoints[0];
			contactPoint.set(sphere.getCentroid());
			contactPoint.x += normal.x * toContactPoint;
			contactPoint.y += normal.y * toContactPoint;
			contactPoint.z += normal.z * toContactPoint;
			
			onA.setFrom(sphere.getCentroid());
			
			contact.penetrations[0] = depth;
			contact.rebuild(normal, depth, 1, onA, onB);
		}else {
			contact.setNoCollision();
		}

	}

}
