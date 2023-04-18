package cataclysm.contact_creation;

import cataclysm.wrappers.ConvexHullWrapper;
import cataclysm.wrappers.SphereWrapper;
import math.vector.Vector3f;

/**
 * Permet de tester la collision entre une sphere et une enveloppe convexe ou un
 * triangle.
 * 
 * @author Briac
 *
 */
class CollideSphereHull {

	private final Vector3f closest = new Vector3f();
	private final Vector3f normal = new Vector3f();

	private final ContactFeature onA = new ContactFeature();
	private final ContactFeature onB = new ContactFeature();

	private final GJK gjk = new GJK();

	void test(SphereWrapper sphere, ConvexHullWrapper hull, ContactZone contact) {

		float distance = gjk.distance(sphere, hull, null, closest);

		// System.out.println("Distance:" + distance);
		// System.out.println("closest:" + closest);

		if (distance < sphere.getRadius()) {

			if (distance > 0.0f) {// shallow contact

				Vector3f.sub(closest, sphere.getCentroid(), normal);
				normal.scale(1.0f / distance);

				gjk.getClosestFeatureOnB(onB);

			} else {
				
				Vector3f sphereCenter = new Vector3f(sphere.getCentroid());
				hull.transformVertexWorldSpaceToWrapperSpace(sphereCenter, sphereCenter);
				
				distance = Float.NEGATIVE_INFINITY;
				int referenceFace = 0;
				for (int face = 0; face < hull.getConvexHullData().faceCount; face++) {
					float d = hull.getConvexHullData().signedDistance(sphereCenter, face);
					if (d > distance) {
						distance = d;
						referenceFace = face;
					}
				}

				distance *= hull.getScale();
				hull.getConvexHullData().getNormal(referenceFace, normal);
				hull.transformNormalWrapperSpaceToWorldSpace(normal, normal);
				normal.negate();
				onB.setFromHullFace(referenceFace);
			}

			float depth = distance - sphere.getRadius();

			float toContactPoint = distance - 0.5f * depth;

			float x = sphere.getCentroid().x + normal.x * toContactPoint;
			float y = sphere.getCentroid().y + normal.y * toContactPoint;
			float z = sphere.getCentroid().z + normal.z * toContactPoint;

			onA.setFrom(sphere.getCentroid());

			contact.setContactPointAndPenetrationDepth(0, x, y, z, depth);
			contact.rebuild(normal, depth, 1, onA, onB);
		} else {
			contact.setNoCollision();
		}

	}

}
