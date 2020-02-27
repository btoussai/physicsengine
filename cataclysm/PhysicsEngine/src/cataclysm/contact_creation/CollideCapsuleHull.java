package cataclysm.contact_creation;

import cataclysm.Epsilons;
import cataclysm.wrappers.CapsuleWrapper;
import cataclysm.wrappers.ConvexHullWrapper;
import cataclysm.wrappers.ConvexHullWrapperFace;
import cataclysm.wrappers.ConvexHullWrapperHalfEdge;
import math.vector.Vector3f;

/**
 * Permet de tester la collision entre une capsule et une enveloppe convexe.
 * 
 * @author Briac
 *
 */
class CollideCapsuleHull {

	private final Vector3f normal = new Vector3f();
	private final Vector3f closestOnHull = new Vector3f();
	private final Vector3f closestOnSegment = new Vector3f();

	private final Vector3f C1 = new Vector3f();
	private final Vector3f C2 = new Vector3f();
	private final Vector3f capsuleSegment = new Vector3f();
	private float capsuleSegmentLengthSquared = 0;

	private final Vector3f clipPlaneNormal = new Vector3f();

	private final ContactFeature onA = new ContactFeature();
	private final ContactFeature onB = new ContactFeature();
	
	private final GJK gjk = new GJK();

	/**
	 * Permet de tester la collision entre une capsule et une enveloppe convexe.
	 * 
	 * @param capsule
	 * @param hull
	 * @param contact
	 */
	void test(CapsuleWrapper capsule, ConvexHullWrapper hull, ContactArea contact) {
		float distance = gjk.distance(capsule, hull, closestOnSegment, closestOnHull);

		if (distance < capsule.getRadius()) {

			onA.setFrom(capsule.getCenter1(), capsule.getCenter2());

			C1.set(capsule.getCenter1());
			C2.set(capsule.getCenter2());
			Vector3f.sub(C2, C1, capsuleSegment);
			capsuleSegmentLengthSquared = capsuleSegment.lengthSquared();

			if (distance > 0.0f) {
				// shallow contact, capsuleSegment est � l'ext�rieur du solide.
				gjk.getClosestFeatureOnB(onB);

				Vector3f.sub(closestOnHull, closestOnSegment, normal);
				normal.scale(1.0f / distance);

				float orthoCheck = Vector3f.dot(normal, capsuleSegment);
				if (orthoCheck * orthoCheck < Epsilons.ORTHOGONAL_LIMIT_2 * capsuleSegmentLengthSquared) {
					// Contact sur la partie cylindrique.

					if (checkCapsuleLayingOnFace(normal, hull)) {
						stackingScenario(capsule, hull, distance, contact);
					} else {
						oneContactPointScenario(normal, distance, capsule, contact);
					}

				} else {
					// Contact sur une extr�mit�.
					oneContactPointScenario(normal, distance, capsule, contact);
				}

			} else {// deep contact

				float distanceToBestFace = distanceToFaces(capsule, hull);
				float distanceToBestEdge = distanceToEdges(capsule, hull);

				if (distanceToBestFace >= distanceToBestEdge) {// deep face contact
					onB.setFrom(bestFace);
					stackingScenario(capsule, hull, distanceToBestFace, contact);
				} else {// deep edge contact
					onB.setFrom(bestEdge);

					float depth = distanceToBestEdge - capsule.getRadius();
					closestPointsBetweenSegments(C1, C2, bestEdge.getTail(), bestEdge.getHead(), C1, C2);

					contact.contactPoints[0].set(0.5f * (C1.x + C2.x), 0.5f * (C1.y + C2.y), 0.5f * (C1.z + C2.z));
					contact.penetrations[0] = depth;
					Vector3f.negate(bestEdgeNormal, normal);
					contact.rebuild(normal, depth, 1, onA, onB);
				}

			}

		} else {
			contact.setNoCollision();
		}

	}

	private ConvexHullWrapperFace bestFace = null;

	/**
	 * Teste si la capsule repose sur une des faces du solide.
	 * 
	 * @param contactNormal La normale du contact, unitaire.
	 * @param hull
	 * @return
	 */
	private boolean checkCapsuleLayingOnFace(Vector3f contactNormal, ConvexHullWrapper hull) {
		float bestDot = Float.MAX_VALUE;

		for (ConvexHullWrapperFace face : hull.getFaces()) {
			Vector3f faceNormal = face.getNormal();
			float dotNormal = Vector3f.dot(contactNormal, faceNormal);
			if (dotNormal < bestDot) {
				bestDot = dotNormal;
				bestFace = face;
			}

		}

		float dotOrtho = Vector3f.dot(bestFace.getNormal(), capsuleSegment);
		return dotOrtho * dotOrtho < Epsilons.ORTHOGONAL_LIMIT_2 * capsuleSegmentLengthSquared;
	}

	/**
	 * Cherche la face maximisant la distance � capsuleSegment, dans le cas o�
	 * capsuleSegment intersecte le solide. (car la distance est n�gative)
	 * 
	 * @param capsule
	 * @param hull
	 * @return une distance n�gative !
	 */
	private float distanceToFaces(CapsuleWrapper capsule, ConvexHullWrapper hull) {
		float distance = Float.NEGATIVE_INFINITY;
		for (ConvexHullWrapperFace face : hull.getFaces()) {
			float d = Math.min(face.signedDistance(C1), face.signedDistance(C2));
			if (d > distance) {
				distance = d;
				bestFace = face;
			}
		}

		return distance;
	}

	private ConvexHullWrapperHalfEdge bestEdge = null;
	private final Vector3f bestEdgeNormal = new Vector3f();
	private final Vector3f segmentCrossEdgeVec = new Vector3f();
	private final Vector3f edgeVec = new Vector3f();
	private final Vector3f capsuleToEdge = new Vector3f();
	private final Vector3f hullCentroid = new Vector3f();
	private final Vector3f capsuleCentroid = new Vector3f();
	private final Vector3f capsuleToHull = new Vector3f();

	/**
	 * Cherche l'ar�te maximisant la distance � capsuleSegment, dans le cas o�
	 * capsuleSegment intersecte le solide. (car la distance est n�gative)
	 * 
	 * @param capsule
	 * @param hull
	 * @return une distance n�gative !
	 */
	private float distanceToEdges(CapsuleWrapper capsule, ConvexHullWrapper hull) {

		hullCentroid.set(hull.getCentroid());
		capsuleCentroid.set(capsule.getCentroid());
		Vector3f.sub(hullCentroid, capsuleCentroid, capsuleToHull);

		float distance = Float.NEGATIVE_INFINITY;
		ConvexHullWrapperHalfEdge[] edges = hull.getEdges();
		for (int i = 0; i < edges.length; i += 2) {
			ConvexHullWrapperHalfEdge edge = edges[i];

			Vector3f.sub(edge.getHead(), edge.getTail(), edgeVec);
			Vector3f.cross(capsuleSegment, edgeVec, segmentCrossEdgeVec);
			Vector3f.sub(edge.getHead(), capsuleCentroid, capsuleToEdge);

			float length = segmentCrossEdgeVec.length();
			if (length < Epsilons.MIN_LENGTH) {// edge parallel to segment
				continue;
			}

			float one_over_length = 1.0f / length;
			if (Vector3f.dot(segmentCrossEdgeVec, capsuleToHull) > 0) {
				one_over_length = -one_over_length;
			}

			float d = Vector3f.dot(segmentCrossEdgeVec, capsuleToEdge) * one_over_length;
			d = -Math.abs(d);
			if (d > distance) {
				distance = d;
				bestEdge = edge;
				bestEdgeNormal.set(segmentCrossEdgeVec.x * one_over_length, segmentCrossEdgeVec.y * one_over_length,
						segmentCrossEdgeVec.z * one_over_length);
			}
		}

		return distance;
	}

	/**
	 * Traite le cas o� la capsule est couch�e sur une face.
	 * 
	 * @param capsule
	 * @param distanceToBestFace
	 */
	private void stackingScenario(CapsuleWrapper capsule, ConvexHullWrapper hull, float distanceToBestFace,
			ContactArea contact) {

		// Clip segment against bestFace
		Vector3f faceNormal = bestFace.getNormal();
		for (ConvexHullWrapperHalfEdge edge : bestFace) {
			Vector3f.sub(edge.getHead(), edge.getTail(), edgeVec);
			Vector3f.cross(faceNormal, edgeVec, clipPlaneNormal);

			float clipPlaneOffset = Vector3f.dot(clipPlaneNormal, edge.getTail());

			clipSegmentAgainstPlane(clipPlaneNormal, clipPlaneOffset);

		}

		normal.set(bestFace.getNormal());
		normal.negate();

		float r = capsule.getRadius();
		float d1 = bestFace.signedDistance(C1);
		float d2 = bestFace.signedDistance(C2);

		boolean C1_above = d1 > r;
		boolean C2_above = d2 > r;

		if (C1_above && C2_above) {
			oneContactPointScenario(normal, distanceToBestFace, capsule, contact);
			return;
		}

		/*
		 * if (Math.abs(d1 - d2) > Epsilons.ALLOWED_PENETRATION) { if (C1_above &&
		 * !C2_above) { float f = (d1 - r) / (d1 - d2); C1.x += f * capsuleSegment.x;
		 * C1.y += f * capsuleSegment.y; C1.z += f * capsuleSegment.z; d1 = r; }
		 * 
		 * if (C2_above && !C1_above) { float f = (d2 - r) / (d2 - d1); C2.x += -f *
		 * capsuleSegment.x; C2.y += -f * capsuleSegment.y; C2.z += -f *
		 * capsuleSegment.z; d2 = r; } }
		 */

		C1.x += d1 * normal.x;
		C1.y += d1 * normal.y;
		C1.z += d1 * normal.z;

		C2.x += d2 * normal.x;
		C2.y += d2 * normal.y;
		C2.z += d2 * normal.z;

		float depth = distanceToBestFace - capsule.getRadius();
		contact.contactPoints[0].set(C1);
		contact.contactPoints[1].set(C2);
		contact.penetrations[0] = d1 - r;
		contact.penetrations[1] = d2 - r;
		contact.rebuild(normal, depth, 2, onA, onB);
	}

	/**
	 * Traite le cas o� il n'y a qu'un seul point de contact.
	 * 
	 * @param normal
	 * @param distance
	 * @param capsule
	 */
	private void oneContactPointScenario(Vector3f normal, float distance, CapsuleWrapper capsule, ContactArea contact) {
		float depth = distance - capsule.getRadius();

		float toContactPoint = distance + 0.5f * depth;
		contact.contactPoints[0].set(closestOnSegment.x + toContactPoint * normal.x,
				closestOnSegment.y + toContactPoint * normal.y, closestOnSegment.z + toContactPoint * normal.z);
		contact.penetrations[0] = depth;
		contact.rebuild(normal, depth, 1, onA, onB);
	}

	/**
	 * Fait en sorte que le segment C1C2 soit enti�rement du c�t� du plan.
	 * 
	 * @param planeNormal
	 * @param planeOffset
	 */
	private void clipSegmentAgainstPlane(Vector3f planeNormal, float planeOffset) {

		float NdotN = planeNormal.lengthSquared();
		float NdotC1 = Vector3f.dot(planeNormal, C1);
		float NdotC2 = Vector3f.dot(planeNormal, C2);
		boolean C1_inside = NdotC1 - planeOffset > 0;
		boolean C2_inside = NdotC2 - planeOffset > 0;

		if (!C1_inside || !C2_inside) {
			float denom = Vector3f.dot(planeNormal, capsuleSegment);
			boolean orthogonal = denom * denom <= Epsilons.ORTHOGONAL_LIMIT_2 * capsuleSegmentLengthSquared * NdotN;
			// Mind the <= comparison instead of the < comparison.
			// That is because capsuleSegment can be zero after several clampings.
			// In that case, we would have orthogonal = false and denom = 0.
			// We would end up dividing by zero :'(

			if (orthogonal) {

				if (!C1_inside) {
					float t = (planeOffset - NdotC1) / NdotN;
					C1.set(C1.x + t * planeNormal.x, C1.y + t * planeNormal.y, C1.z + t * planeNormal.z);
				}
				if (!C2_inside) {
					float t = (planeOffset - NdotC2) / NdotN;
					C2.set(C2.x + t * planeNormal.x, C2.y + t * planeNormal.y, C2.z + t * planeNormal.z);
				}
				Vector3f.sub(C2, C1, capsuleSegment);
				capsuleSegmentLengthSquared = capsuleSegment.lengthSquared();

			} else {

				if (!C1_inside) {
					float t = (planeOffset - NdotC1) / denom;
					C1.set(C1.x + t * capsuleSegment.x, C1.y + t * capsuleSegment.y, C1.z + t * capsuleSegment.z);
				}
				if (!C2_inside) {
					float t = (planeOffset - NdotC2) / denom;
					C2.set(C2.x + t * capsuleSegment.x, C2.y + t * capsuleSegment.y, C2.z + t * capsuleSegment.z);
				}
				Vector3f.sub(C2, C1, capsuleSegment);
				capsuleSegmentLengthSquared = capsuleSegment.lengthSquared();

			}

		}

	}

	private Vector3f AB = new Vector3f();
	private Vector3f CD = new Vector3f();
	private Vector3f AC = new Vector3f();

	/**
	 * Calcule la position du point appartenant � AB �tant le plus proche de CD.
	 * Stocke le r�sultat dans destOnFirstSegment. Calcule la position du point
	 * appartenant � CD �tant le plus proche de AB. Stocke le r�sultat dans
	 * destOnSecondSegment.
	 * 
	 * @param A
	 * @param B
	 * @param C
	 * @param D
	 * @param destOnAB
	 * @param destOnCD
	 */
	private void closestPointsBetweenSegments(Vector3f A, Vector3f B, Vector3f C, Vector3f D, Vector3f destOnAB,
			Vector3f destOnCD) {

		Vector3f.sub(B, A, AB);
		Vector3f.sub(D, C, CD);
		Vector3f.sub(C, A, AC);

		float ABCD = Vector3f.dot(AB, CD);
		float ABAC = Vector3f.dot(AB, AC);
		float CDAC = Vector3f.dot(CD, AC);

		float AB2 = AB.lengthSquared();
		float CD2 = CD.lengthSquared();

		float det = AB2 * CD2 - ABCD * ABCD;
		if (det < (1.0f - Epsilons.PARALLEL_LIMIT_2) * AB2 * CD2) {
			return;
		}

		float one_over_det = 1.0f / det;

		float t = one_over_det * (CD2 * ABAC - ABCD * CDAC);
		float s = one_over_det * (ABCD * ABAC - AB2 * CDAC);

		if (t < 0.0f) {
			t = 0.0f;
			s = -CDAC / CD2;
		} else if (t > 1.0f) {
			t = 1.0f;
			s = (-CDAC + ABCD) / CD2;
		}

		if (s < 0.0f) {
			t = ABAC / AB2;
			s = 0.0f;

			if (t < 0.0f) {
				t = 0.0f;
			} else if (t > 1.0f) {
				t = 1.0f;
			}
		} else if (s > 1.0f) {
			t = (ABAC + ABCD) / AB2;
			s = 1.0f;

			if (t < 0.0f) {
				t = 0.0f;
			} else if (t > 1.0f) {
				t = 1.0f;
			}
		}

		destOnAB.set(A.x + t * AB.x, A.y + t * AB.y, A.z + t * AB.z);
		destOnCD.set(C.x + s * CD.x, C.y + s * CD.y, C.z + s * CD.z);
	}

}
