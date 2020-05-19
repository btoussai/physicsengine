package cataclysm.contact_creation;

import java.util.ArrayList;
import java.util.List;

import cataclysm.Epsilons;
import cataclysm.wrappers.ConvexHullWrapper;
import cataclysm.wrappers.ConvexHullWrapper.FloatLayout;
import math.vector.Vector3f;

class PolygonClipping {

	// Quelques variables statiques pour �viter des new
	private final Vector3f clipPlaneNormal = new Vector3f();
	private final Vector3f edgeTail = new Vector3f();
	private final Vector3f edgeVec = new Vector3f();
	private final Vector3f faceNormal = new Vector3f();
	private final Vector3f previousToCurrent = new Vector3f();

	private int inputListSize = 0;
	private int outputListSize = 0;
	private List<Vector3f> inputList = new ArrayList<Vector3f>();
	private List<Vector3f> outputList = new ArrayList<Vector3f>();

	/**
	 * Clippe la face incidente contre les bords de la face de référence. C'est une
	 * implémentation de l'algorithme de Sutherland-Hodgman.
	 * 
	 * @param incidentFace
	 * @param referenceFace
	 * @param clippedVertices
	 */
	void clipIncidentFaceAgainstReferenceFace(ConvexHullWrapper incident, int incidentFace,
			ConvexHullWrapper reference, int referenceFace, List<Vector3f> clippedVertices) {
		inputListSize = 0;
		outputListSize = 0;

		// On remplit inputList avec les points de incidentFace
		int edge0 = incident.getFaceEdge0(incidentFace);
		int edge = edge0;
		do {
			incident.get(FloatLayout.Vertices, incident.getEdgeTail(edge), edgeTail);
			inputListSize = addVertex(inputList, inputListSize, edgeTail);
			edge = incident.getEdgeNext(edge);
		} while (edge != edge0);

		reference.get(FloatLayout.FaceNormals, referenceFace, faceNormal);

		edge0 = reference.getFaceEdge0(referenceFace);
		edge = edge0;
		do {

			reference.get(FloatLayout.Vertices, reference.getEdgeTail(edge), edgeTail);
			reference.sub(FloatLayout.Vertices, reference.getEdgeHead(edge), edgeTail, edgeVec);
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
				// the faces aren't overlapping, we return
				clippedVertices.clear();
				return;
			}

			swapLists();
			inputListSize = outputListSize;
			outputListSize = 0;

			edge = reference.getEdgeNext(edge);
		} while (edge != edge0);

		clippedVertices.clear();
		clippedVertices.addAll(inputList.subList(0, inputListSize));
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
	private void clipEdgeToPlane(Vector3f planeNormal, float planeOffset, float planeLength2, Vector3f edgeBase,
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

	private void swapLists() {
		List<Vector3f> temp = inputList;
		inputList = outputList;
		outputList = temp;
	}

	private int addVertex(List<Vector3f> list, int currentListSize, Vector3f vertex) {
		if (list.size() <= currentListSize) {
			list.add(new Vector3f());
		}
		list.get(currentListSize).set(vertex);
		return currentListSize + 1;
	}

	private Vector3f getNextVertex(List<Vector3f> list, int currentListSize) {
		if (list.size() <= currentListSize) {
			list.add(new Vector3f());
		}
		return list.get(currentListSize);
	}

}
