package cataclysm.contact_creation;

import java.util.ArrayList;
import java.util.List;

import org.lwjgl.util.vector.Vector3f;

import cataclysm.Epsilons;
import cataclysm.wrappers.ConvexHullWrapperFace;
import cataclysm.wrappers.ConvexHullWrapperHalfEdge;

class PolygonClipping {

	// Quelques variables statiques pour �viter des new
	private static final Vector3f clipPlaneNormal = new Vector3f();
	private static final Vector3f edgeTail = new Vector3f();
	private static final Vector3f edgeVec = new Vector3f();
	private static final Vector3f previousToCurrent = new Vector3f();

	private static int inputListSize = 0;
	private static int outputListSize = 0;
	private static List<Vector3f> inputList = new ArrayList<Vector3f>();
	private static List<Vector3f> outputList = new ArrayList<Vector3f>();

	/**
	 * Clippe la face incidente contre les bords de la face de référence. C'est une
	 * implémentation de l'algorithme de Sutherland-Hodgman.
	 * 
	 * @param incidentFace
	 * @param referenceFace
	 * @return
	 */
	static List<Vector3f> clipIncidentFaceAgainstReferenceFace(ConvexHullWrapperFace incidentFace,
			ConvexHullWrapperFace referenceFace) {
		inputListSize = 0;
		outputListSize = 0;

		// On remplit inputList avec les points de incidentFace
		ConvexHullWrapperHalfEdge edge0 = incidentFace.getEdge0();
		ConvexHullWrapperHalfEdge edge = edge0;
		do {
			inputListSize = addVertex(inputList, inputListSize, edge.getTail());
			edge = edge.getNext();
		} while (edge != edge0);

		Vector3f faceNormal = referenceFace.getNormal();

		edge0 = referenceFace.getEdge0();
		edge = edge0;
		do {

			edgeTail.set(edge.getTail());
			Vector3f.sub(edge.getHead(), edge.getTail(), edgeVec);
			Vector3f.cross(faceNormal, edgeVec, clipPlaneNormal);

			float clipPlaneOffset = Vector3f.dot(clipPlaneNormal, edgeTail);
			float clipPlaneLength2 = clipPlaneNormal.lengthSquared();

			if (inputListSize > 1) {

				Vector3f previous = inputList.get(inputListSize - 1);
				boolean previousInside = Vector3f.dot(clipPlaneNormal, previous) - clipPlaneOffset >= 0;

				for (int i = 0; i < inputListSize; i++) {
					Vector3f current = inputList.get(i);

					boolean inside = Vector3f.dot(clipPlaneNormal, current) - clipPlaneOffset >= 0;

					if (inside) {
						if (!previousInside) {
							Vector3f.sub(current, previous, previousToCurrent);

							Vector3f dest = getNextVertex(outputList, outputListSize);
							outputListSize++;
							clipEdgeToPlane(clipPlaneNormal, clipPlaneOffset, clipPlaneLength2, previous,
									previousToCurrent, dest);
						}
						outputListSize = addVertex(outputList, outputListSize, current);

					} else if (previousInside) {
						Vector3f.sub(current, previous, previousToCurrent);

						Vector3f dest = getNextVertex(outputList, outputListSize);
						outputListSize++;
						clipEdgeToPlane(clipPlaneNormal, clipPlaneOffset, clipPlaneLength2, previous, previousToCurrent,
								dest);
					}

					previousInside = inside;
					previous = current;
				}

			}

			if (outputListSize == 0) {
				// On cherche le point le plus proche du plan et on le projette dessus.
				float bestDistance = Float.NEGATIVE_INFINITY;
				Vector3f bestVertex = null;
				for (int i = 0; i < inputListSize; i++) {
					Vector3f vertex = inputList.get(i);
					float distance = Vector3f.dot(clipPlaneNormal, vertex) - clipPlaneOffset;
					if (distance > bestDistance) {
						bestDistance = distance;
						bestVertex = vertex;
					}
				}
				if (bestDistance < 0) {
					bestDistance /= clipPlaneLength2;
					bestVertex.x -= bestDistance * clipPlaneNormal.x;
					bestVertex.y -= bestDistance * clipPlaneNormal.y;
					bestVertex.z -= bestDistance * clipPlaneNormal.z;
				}
					outputListSize = addVertex(outputList, outputListSize, bestVertex);
			}

			swapLists();
			inputListSize = outputListSize;
			outputListSize = 0;

			edge = edge.getNext();
		} while (edge != edge0);

		return new ArrayList<Vector3f>(inputList.subList(0, inputListSize));
	}

	/**
	 * Calcule l'intersection de l'arête avec le plan.
	 * 
	 * @param planeNormal
	 * @param planeOffset
	 * @param edgeBase
	 * @param edgeVec
	 * @param dest
	 */
	private static void clipEdgeToPlane(Vector3f planeNormal, float planeOffset, float planeLength2, Vector3f edgeBase,
			Vector3f edgeVec, Vector3f dest) {

		float denom = Vector3f.dot(planeNormal, edgeVec);
		float NdotEdgeBase = Vector3f.dot(planeNormal, edgeBase);

		boolean orthogonal = denom * denom < Epsilons.ORTHOGONAL_LIMIT_2 * edgeVec.lengthSquared() * planeLength2;

		if (orthogonal) {
			float t = (planeOffset - NdotEdgeBase) / planeLength2;
			dest.set(edgeBase.x + t * planeNormal.x, edgeBase.y + t * planeNormal.y, edgeBase.z + t * planeNormal.z);
		} else {
			float t = (planeOffset - NdotEdgeBase) / denom;
			dest.set(edgeBase.x + t * edgeVec.x, edgeBase.y + t * edgeVec.y, edgeBase.z + t * edgeVec.z);

		}

	}

	private static void swapLists() {
		List<Vector3f> temp = inputList;
		inputList = outputList;
		outputList = temp;
	}

	private static int addVertex(List<Vector3f> list, int currentListSize, Vector3f vertex) {
		if (list.size() <= currentListSize) {
			list.add(new Vector3f());
		}
		list.get(currentListSize).set(vertex);
		return currentListSize + 1;
	}

	private static Vector3f getNextVertex(List<Vector3f> list, int currentListSize) {
		if (list.size() <= currentListSize) {
			list.add(new Vector3f());
		}
		return list.get(currentListSize);
	}

}
