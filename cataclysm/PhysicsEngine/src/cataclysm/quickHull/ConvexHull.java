package cataclysm.quickHull;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.lwjgl.util.vector.Vector3f;

import cataclysm.wrappers.ConvexHullWrapperData;
import cataclysm.wrappers.ConvexHullWrapperFace;
import cataclysm.wrappers.ConvexHullWrapperHalfEdge;

/**
 * Représente un ensemble convexe de points.
 * 
 * @author Briac
 *
 */
public class ConvexHull {

	/**
	 * L'ensemble des faces de l'enveloppe convexe.
	 */
	List<Face> faces;

	/**
	 * Le nuage de point initial autour duquel l'enveloppe est construite.
	 */
	List<Vector3f> initialVertices;

	ConvexHull(List<Vector3f> points) {
		initialVertices = points;
	}

	void initFromTetrahedron(int[] indicesTetra, Vector3f[] verticesTetra, List<Vector3f> points) {
		faces = new ArrayList<Face>(4);

		Vertex A = new Vertex(verticesTetra[0]);
		Vertex B = new Vertex(verticesTetra[1]);
		Vertex C = new Vertex(verticesTetra[2]);
		Vertex D = new Vertex(verticesTetra[3]);

		Face F1 = createFace(A, B, C, D);
		Face F2 = createFace(D, A, B, C);
		Face F3 = createFace(C, D, A, B);
		Face F4 = createFace(B, C, D, A);

		makeAdjacent(F1, F2);
		makeAdjacent(F1, F3);
		makeAdjacent(F1, F4);
		makeAdjacent(F2, F3);
		makeAdjacent(F2, F4);
		makeAdjacent(F3, F4);

		for (int i = 0; i < points.size(); i++) {

			if (isTetrahedronVertex(i, indicesTetra)) {
				continue;
			}

			Vector3f point = points.get(i);
			assignToConflictList(point, faces);

		}

	}

	private boolean isTetrahedronVertex(int index, int[] indicesTetra) {
		for (int i : indicesTetra) {
			if (i == index) {
				return true;
			}
		}
		return false;
	}

	/**
	 * Assigne un point à laliste de conflit d'une des faces. Cette face est la plus
	 * éloignée du point.
	 * 
	 * @param point
	 * @param faces
	 */
	public void assignToConflictList(Vector3f point, List<Face> faces) {

		float maxDistance = QuickHull.epsilon;
		Face maxFace = null;
		for (int j = 0; j < faces.size(); j++) {
			Face face = faces.get(j);

			float distance = face.signedDistance(point);
			if (distance > maxDistance) {
				maxDistance = distance;
				maxFace = face;
			}
		}
		if (maxFace != null) {
			maxFace.getConflictList().add(point);
			if (maxFace.getFurthestDistance() < maxDistance) {
				maxFace.setFurthest(point);
				maxFace.setFurthestDistance(maxDistance);
			}
		}

	}

	private Face createFace(Vertex A, Vertex B, Vertex C, Vertex opposite) {

		Vector3f normalDir = Vector3f.sub(A.getPosition(), opposite.getPosition(), null);

		Face face = new Face(A, B, C, normalDir);
		faces.add(face);

		return face;
	}

	private void makeAdjacent(Face F1, Face F2) {

		HalfEdge firstA = F1.getEdge();
		HalfEdge edgeA = firstA;

		do {
			HalfEdge firstB = F2.getEdge();
			HalfEdge edgeB = firstB;

			do {

				if (edgeA.getNext().getTail() == edgeB.getTail() && edgeA.getTail() == edgeB.getNext().getTail()) {
					HalfEdge.makeTwins(edgeA, edgeB);
					return;
				}

				edgeB = edgeB.getNext();
			} while (edgeB != firstB);

			edgeA = edgeA.getNext();
		} while (edgeA != firstA);

	}

	/**
	 * Construit une représentation graphique non optimisée de l'enveloppe convexe,
	 * sous forme de faces triangulaires.
	 * 
	 * @param indices  La liste dans laquelle stocker les indices des sommets
	 *                 construisant les faces.
	 * @param vertices La liste dans laquelle stocker les sommets de l'enveloppe
	 *                 convexe.
	 */
	public void getFaces(List<Integer> indices, List<Vector3f> vertices) {
		indices.clear();
		vertices.clear();

		int totalVertexCount = 0;
		for (Face face : faces) {

			HalfEdge firstEdge = face.getEdge();
			HalfEdge edge = firstEdge;

			int indexVertexOnFace = 0;
			do {

				vertices.add(new Vector3f(edge.getTail().getPosition()));

				if (indexVertexOnFace >= 2) {// Triangulation basique de la face
					indices.add(totalVertexCount);
					indices.add(totalVertexCount + indexVertexOnFace - 1);
					indices.add(totalVertexCount + indexVertexOnFace);
				}

				edge = edge.getNext();
				indexVertexOnFace++;
			} while (edge != firstEdge);

			totalVertexCount = vertices.size();

		}

	}

	/**
	 * Recupère les arêtes de l'enveloppe convexe.
	 * 
	 * @param vertices La liste dans laquelle stocker les sommets.
	 */
	public void getEdges(List<Vector3f> vertices) {
		vertices.clear();

		for (Face face : faces) {

			HalfEdge firstEdge = face.getEdge();
			HalfEdge edge = firstEdge;

			do {

				vertices.add(new Vector3f(edge.getTail().getPosition()));
				vertices.add(new Vector3f(edge.getTwin().getTail().getPosition()));

				edge = edge.getNext();
			} while (edge != firstEdge);

		}
	}

	/**
	 * Recupère les sommets de l'enveloppe convexe.
	 * 
	 * @param vertices La liste dans laquelle stocker les sommets.
	 */
	public void getVertices(List<Vector3f> vertices) {
		vertices.clear();

		for (Face face : faces) {

			HalfEdge firstEdge = face.getEdge();
			HalfEdge edge = firstEdge;

			do {
				vertices.add(new Vector3f(edge.getTail().getPosition()));

				edge = edge.getNext();
			} while (edge != firstEdge);

		}
	}

	public ConvexHullWrapperData convertToWrapperData() {

		Map<HalfEdge, ConvexHullWrapperHalfEdge> map = new HashMap<HalfEdge, ConvexHullWrapperHalfEdge>();

		List<ConvexHullWrapperFace> wFaces = new ArrayList<ConvexHullWrapperFace>();

		List<Vector3f> wVertices = new ArrayList<Vector3f>();
		List<Vector3f> wNormals = new ArrayList<Vector3f>();
		List<Vector3f> wCentroids = new ArrayList<Vector3f>();

		// Build faces and edges
		for (Face face : this.faces) {
			face.convertToWrapperFace(map, wFaces, wNormals, wCentroids);
		}

		// Make twins and give indices.
		ConvexHullWrapperHalfEdge[] wEdges = new ConvexHullWrapperHalfEdge[map.size()];
		int i = 0;
		for (Entry<HalfEdge, ConvexHullWrapperHalfEdge> entry : map.entrySet()) {
			HalfEdge edge = entry.getKey();
			ConvexHullWrapperHalfEdge copyEdge = entry.getValue();

			if (copyEdge.getTwinIndex() == -1) {
				wEdges[i] = copyEdge;
				copyEdge.setIndex(i);
				wFaces.get(copyEdge.getFaceIndex()).setEdge0(i);

				ConvexHullWrapperHalfEdge copyTwin = map.get(edge.getTwin());
				wEdges[i + 1] = copyTwin;
				copyTwin.setIndex(i + 1);
				wFaces.get(copyTwin.getFaceIndex()).setEdge0(i + 1);

				copyEdge.setTwin(i + 1);
				copyTwin.setTwin(i);

				i += 2;
			}

		}

		// Resolve prev/next adjacency.
		for (Entry<HalfEdge, ConvexHullWrapperHalfEdge> entry : map.entrySet()) {
			HalfEdge edge = entry.getKey();
			ConvexHullWrapperHalfEdge copyEdge = entry.getValue();

			copyEdge.setNext(map.get(edge.getNext()).getIndex());
			copyEdge.setPrev(map.get(edge.getPrev()).getIndex());
		}

		// Build vertices
		for (Entry<HalfEdge, ConvexHullWrapperHalfEdge> entry : map.entrySet()) {
			HalfEdge edge = entry.getKey();
			ConvexHullWrapperHalfEdge copyEdge = entry.getValue();

			int tailIndex = -1;
			ConvexHullWrapperHalfEdge it = copyEdge;
			do {
				if ((tailIndex = it.getTailIndex()) != -1) {
					break;
				}

				it = wEdges[wEdges[it.getTwinIndex()].getNextIndex()];
			} while (it != copyEdge);

			if (tailIndex == -1) {
				tailIndex = wVertices.size();
				Vector3f tail = new Vector3f(edge.getTail().getPosition());
				wVertices.add(tail);
			}

			it = copyEdge;
			do {
				it.setTail(tailIndex);
				it = wEdges[wEdges[it.getTwinIndex()].getNextIndex()];
			} while (it != copyEdge);

		}

		return new ConvexHullWrapperData(wFaces.toArray(new ConvexHullWrapperFace[0]), wEdges,
				wVertices.toArray(new Vector3f[0]), wNormals.toArray(new Vector3f[] {}),
				wCentroids.toArray(new Vector3f[] {}));
	}

	@Override
	public String toString() {
		String str = new String("Hull: ");
		for (Face face : faces) {
			str += "\n" + face.toString();
		}

		return str;
	}

}
