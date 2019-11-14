package cataclysm.quickHull;

import java.util.Arrays;
import java.util.List;

import org.lwjgl.util.vector.Vector3f;

/**
 * Contient une méthode static pour générer le tétrahèdre le plus grand possible
 * à partir d'un nuage de points.
 * 
 * @author Briac
 *
 */
final class Tetrahedron {

	/**
	 * Construit un tétrahèdre le plus grand possible à partir de la liste de
	 * sommets.
	 * 
	 * @param hull   L'enveloppe convexe à construire.
	 * @param points La liste des sommets.
	 * @return False si le tétrahèdre n'a pas pu être construit.
	 */
	static boolean buildInitialHull(ConvexHull hull, List<Vector3f> points) {

		if (points.size() < 4) {
			return false;
		}

		// Les indices des 4 sommets du tétrahèdre.
		int[] indicesTetra = { -1, -1, -1, -1 };
		// Les sommets du tétrahèdre.
		Vector3f[] verticesTetra = new Vector3f[4];

		Vector3f[] boxVertices = new Vector3f[6];
		int[] boxIndices = new int[6];
		Arrays.fill(boxVertices, points.get(0));

		makeBiggestBox(points, boxIndices, boxVertices);

		computeEpsilon(boxVertices);

		if (!makeLongestLine(points, boxIndices, boxVertices, indicesTetra, verticesTetra)) {
			return false;
		}

		if (!makeBiggestTriangle(points, indicesTetra, verticesTetra)) {
			return false;
		}

		if (!makeBiggestTetrahedron(points, indicesTetra, verticesTetra)) {
			return false;
		}

		hull.initFromTetrahedron(indicesTetra, verticesTetra, points);

		return true;
	}

	private static void makeBiggestBox(List<Vector3f> points, int[] boxIndices, Vector3f[] boxVertices) {
		for (int i = 0; i < points.size(); i++) {
			Vector3f point = points.get(i);

			if (point.x < boxVertices[0].x) {
				boxVertices[0] = point;
				boxIndices[0] = i;
			} else if (point.x > boxVertices[1].x) {
				boxVertices[1] = point;
				boxIndices[1] = i;
			}

			if (point.y < boxVertices[2].y) {
				boxVertices[2] = point;
				boxIndices[2] = i;
			} else if (point.y > boxVertices[3].y) {
				boxVertices[3] = point;
				boxIndices[3] = i;
			}

			if (point.z < boxVertices[4].z) {
				boxVertices[4] = point;
				boxIndices[4] = i;
			} else if (point.z > boxVertices[5].z) {
				boxVertices[5] = point;
				boxIndices[5] = i;
			}

		}
	}

	private static boolean makeBiggestTetrahedron(List<Vector3f> points, int[] indicesTetra, Vector3f[] verticesTetra) {

		Vector3f AB = Vector3f.sub(verticesTetra[1], verticesTetra[0], null);
		Vector3f AC = Vector3f.sub(verticesTetra[2], verticesTetra[0], null);
		Vector3f normal = Vector3f.cross(AB, AC, null);

		float maxDistance = -1;
		Vector3f AD = new Vector3f();
		for (int i = 0; i < points.size(); i++) {
			Vector3f point = points.get(i);

			Vector3f.sub(point, verticesTetra[0], AD);

			float distance = Math.abs(Vector3f.dot(normal, AD));
			if (distance > maxDistance) {
				maxDistance = distance;
				indicesTetra[3] = i;
				verticesTetra[3] = point;
			}

		}

		if (indicesTetra[3] == indicesTetra[2] || indicesTetra[3] == indicesTetra[1]
				|| indicesTetra[3] == indicesTetra[0]) {
			return false;
		}

		return true;
	}

	private static boolean makeBiggestTriangle(List<Vector3f> points, int[] indicesTetra, Vector3f[] verticesTetra) {
		float maxArea = -1;

		Vector3f AB = Vector3f.sub(verticesTetra[1], verticesTetra[0], null);
		Vector3f AC = new Vector3f();
		Vector3f normal = new Vector3f();
		for (int i = 0; i < points.size(); i++) {
			Vector3f point = points.get(i);

			Vector3f.sub(point, verticesTetra[0], AC);

			Vector3f.cross(AB, AC, normal);

			float area = normal.lengthSquared();
			if (area > maxArea) {
				maxArea = area;
				indicesTetra[2] = i;
				verticesTetra[2] = point;
			}

		}

		if (indicesTetra[2] == indicesTetra[0] || indicesTetra[2] == indicesTetra[1]) {
			return false;
		}
		return true;
	}

	private static boolean makeLongestLine(List<Vector3f> points, int[] boxIndices, Vector3f[] box, int[] indicesTetra,
			Vector3f[] verticesTetra) {
		float maxDistance = 0;
		Vector3f AB = new Vector3f();

		for (int i = 0; i < box.length; i++) {
			Vector3f A = box[i];
			for (int j = i + 1; j < box.length; j++) {
				Vector3f B = box[j];

				Vector3f.sub(B, A, AB);
				float distance = AB.lengthSquared();
				if (distance > maxDistance) {
					maxDistance = distance;
					indicesTetra[0] = boxIndices[i];
					indicesTetra[1] = boxIndices[j];
				}

			}
		}

		if (indicesTetra[0] == indicesTetra[1]) {
			return false;
		}

		verticesTetra[0] = points.get(indicesTetra[0]);
		verticesTetra[1] = points.get(indicesTetra[1]);

		return true;
	}

	/**
	 * Calcule la valeur de la constante de précision à partir des coordonnées
	 * maximales du nuage de point.
	 * 
	 * @param box
	 */
	private static void computeEpsilon(Vector3f[] box) {

		float maxX = 0f, maxY = 0f, maxZ = 0f;

		for (Vector3f vertex : box) {
			maxX = Math.max(maxX, Math.abs(vertex.x));
			maxY = Math.max(maxY, Math.abs(vertex.y));
			maxZ = Math.max(maxZ, Math.abs(vertex.z));
		}

		QuickHull.epsilon = 3.0f * (maxX + maxY + maxZ) * QuickHull.FLT_EPSILON;

		if (QuickHull.DEBUG) {
			System.out.println("Epsilon: " + QuickHull.epsilon);
		}
	}

}
