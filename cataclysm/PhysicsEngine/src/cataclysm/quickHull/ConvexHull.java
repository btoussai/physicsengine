package cataclysm.quickHull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import cataclysm.wrappers.ConvexHullWrapperData;
import cataclysm.wrappers.PolyhedralMassProperties;
import math.vector.Vector3f;

/**
 * Repr�sente un ensemble convexe de points.
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
	
	private final PolyhedralMassProperties massProperties = new PolyhedralMassProperties();

	ConvexHull(List<Vector3f> points) {
		initialVertices = points;
	}

	void initFromTetrahedron(int[] indicesTetra, Vector3f[] verticesTetra, List<Vector3f> points, float epsilon) {
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
			assignToConflictList(point, faces, epsilon);

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
	 * Assigne un point � laliste de conflit d'une des faces. Cette face est la plus
	 * �loign�e du point.
	 * 
	 * @param point
	 * @param faces
	 * @param epsilon
	 */
	public void assignToConflictList(Vector3f point, List<Face> faces, float epsilon) {

		float maxDistance = epsilon;
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

		Vector3f normalDir = Vector3f.sub(A.getPosition(), opposite.getPosition());

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
	 * Construit une repr�sentation graphique non optimis�e de l'enveloppe convexe,
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
	 * Recup�re les ar�tes de l'enveloppe convexe.
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
	 * Recup�re les sommets de l'enveloppe convexe.
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

	private static final int E_TAIL = 0;
	private static final int E_NEXT = 1;
	private static final int E_TWIN = 2;
	private static final int E_FACE = 3;
	private static final int E_SIZE = 4;

	private static final int F_SIZE = 1;

	public ConvexHullWrapperData convertToWrapperData() {

		int faceCount = faces.size();
		int vertexCount = 0;
		int edgeCount = 0;

		short[] intData;
		float[] floatData;

		// Map each edge to its index in the array
		Map<HalfEdge, Short> edges = new HashMap<HalfEdge, Short>();
		for (Face f : faces) {
			HalfEdge e0 = f.getEdge();
			HalfEdge e = e0;
			do {
				if (!edges.containsKey(e)) {// twin and edge are new
					short edgePtr = (short) edgeCount;
					short twinPtr = (short) (edgePtr + 1);
					edgeCount += 2;
					edges.put(e.getTwin(), twinPtr);
					edges.put(e, edgePtr);
				}

				e = e.getNext();
			} while (e != e0);

		}

		intData = new short[F_SIZE * faceCount + E_SIZE * edgeCount];
		Arrays.fill(intData, (short) -1);

		List<Vector3f> vertices = new ArrayList<Vector3f>();

		// Build vertices
		for (Entry<HalfEdge, Short> entry : edges.entrySet()) {
			HalfEdge edge = entry.getKey();
			int edgePtr = entry.getValue();

			short tailIndex = -1;
			HalfEdge it = edge;
			int itPtr = edgePtr;
			do {
				if ((tailIndex = intData[F_SIZE * faceCount + E_SIZE * itPtr + E_TAIL]) != -1) {
					break;
				}

				it = it.getTwin().getNext();
				itPtr = edges.get(it);
			} while (it != edge);

			if (tailIndex == -1) {
				tailIndex = (short) vertices.size();
				Vector3f tail = new Vector3f(edge.getTail().getPosition());
				vertices.add(tail);
			}

			it = edge;
			itPtr = edgePtr;
			do {
				intData[F_SIZE * faceCount + E_SIZE * itPtr + E_TAIL] = tailIndex;
				it = it.getTwin().getNext();
				itPtr = edges.get(it);
			} while (it != edge);

		}

		vertexCount = vertices.size();
		//	  						Vertices, 	FaceNormals, 	FaceCentroids, PlaneOffsets, 
		floatData = new float[3 * vertexCount + 3 * faceCount + 3 * faceCount + faceCount
              // BackupVertices, BackupFaceNormals, BackupFaceCentroids;
		       + 3 * vertexCount + 3 * faceCount + 3 * faceCount];

		for (int f = 0; f < faces.size(); f++) {
			Face face = faces.get(f);
			HalfEdge e0 = face.getEdge();

			intData[f] = edges.get(e0);// Set pointer to e0 int face

			HalfEdge e = e0;
			do {
				int edgePtr = edges.get(e);
				// set edge data, edge tail is already set at this point
				intData[F_SIZE * faceCount + E_SIZE * edgePtr + E_NEXT] = edges.get(e.getNext());
				intData[F_SIZE * faceCount + E_SIZE * edgePtr + E_TWIN] = edges.get(e.getTwin());
				intData[F_SIZE * faceCount + E_SIZE * edgePtr + E_FACE] = (short) f;

				e = e.getNext();
			} while (e != e0);

			int NBackupStart = 6 * vertexCount + 7 * faceCount + 3 * f;
			int CBackupStart = 6 * vertexCount + 10 * faceCount + 3 * f;
			int NStart = 3 * vertexCount + 3 * f;
			int CStart = 3 * vertexCount + 3 * faceCount + 3 * f;
			int PStart = 3 * vertexCount + 6 * faceCount + f;
			
			Vector3f N = face.getNormal();
			Vector3f C = face.getCenter();

			floatData[NStart + 0] = floatData[NBackupStart + 0] = N.x;
			floatData[NStart + 1] = floatData[NBackupStart + 1] = N.y;
			floatData[NStart + 2] = floatData[NBackupStart + 2] = N.z;

			floatData[CStart + 0] = floatData[CBackupStart + 0] = C.x;
			floatData[CStart + 1] = floatData[CBackupStart + 1] = C.y;
			floatData[CStart + 2] = floatData[CBackupStart + 2] = C.z;
			
			floatData[PStart] = Vector3f.dot(N, C);
		}

		float maxRadius = 0;
		for (int i = 0; i < vertexCount; i++) {
			Vector3f v = vertices.get(i);
			int vBackupStart = 3 * vertexCount + 7 * faceCount + 3 * i;
			int vStart = 3 * i;
			floatData[vStart + 0] = floatData[vBackupStart + 0] = v.x;
			floatData[vStart + 1] = floatData[vBackupStart + 1] = v.y;
			floatData[vStart + 2] = floatData[vBackupStart + 2] = v.z;
			maxRadius = Math.max(maxRadius, v.lengthSquared());
		}
		maxRadius = (float) Math.sqrt(maxRadius);

		return new ConvexHullWrapperData(faceCount, edgeCount, vertexCount, intData, floatData, maxRadius, massProperties);
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
