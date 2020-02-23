package cataclysm.quickHull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import org.lwjgl.util.vector.Vector3f;

/**
 * Cette classe permet de construire l'enveloppe convexe en 3D d'un nuage de
 * points.
 * 
 * @author Briac
 *
 */
public class QuickHull {

	/**
	 * Repr�sente la pr�cision g�n�rale pour l'algorithme.
	 */
	static final float FLT_EPSILON = 1E-4f;

	/**
	 * Repr�sente la pr�cision pour l'enveloppe convexe en cours de construction.
	 * Celle-ci est proportionnelle � la taille de l'enveloppe (spatialement)
	 */
	public static float epsilon = 0;

	/**
	 * Repr�sente une borne sup�rieure au nombre d'it�rations.
	 */
	private static int MAX_ITERATIONS;

	/**
	 * Le nombre d'it�ration pour l'enveloppe convexe en cours de calcul.
	 */
	public static int iterations = 0;

	public static final boolean DEBUG = false;

	public static ConvexHull buildConvexHull(Vector3f[] points, int max_points) {
		return buildConvexHull(Arrays.asList(points), max_points);
	}

	public static ConvexHull buildConvexHull(List<Vector3f> points, int max_points) {

		MAX_ITERATIONS = Math.min(max_points, points.size());
		if (MAX_ITERATIONS < 4) {
			throw new IllegalArgumentException("Error, at least 4 points are needed to build a convex hull");
		}

		ConvexHull hull = new ConvexHull(points);
		if (!Tetrahedron.buildInitialHull(hull, points)) {
			throw new IllegalArgumentException("Error while building the convex hull. The points are coplanar");
		}

		// System.out.println("Initial hull: ");
		// System.out.println(hull);
		iterations = 0;

		Vector3f vertex = new Vector3f();
		Face conflictFace = nextConflictVertex(hull, vertex);
		while (conflictFace != null && conflictFace.getFurthestDistance() > epsilon && iterations < MAX_ITERATIONS) {

			iterations++;

			if (DEBUG) {
				System.out.println("\n\n#######	Iteration " + iterations);
			}

			addVertexToHull(vertex, conflictFace, hull);
			conflictFace = nextConflictVertex(hull, vertex);
		}

		if (DEBUG) {
			int remainingVertices = 0;
			for (Face face : hull.faces) {
				remainingVertices += face.getConflictList().size();
			}
			System.out.println("ConvexHull computed in " + iterations + " iterations.");
			System.out.println("Vertices: " + points.size() + " remaining vertices: " + remainingVertices);
			System.out.println("Epsilon: " + epsilon);
		}

		return hull;
	}

	/**
	 * Det�rmine le prochain sommet � ajouter � l'enveloppe et range ses coordonn�es
	 * dans le vecteur en param�tre.
	 * 
	 * @param hull
	 * @param vertex
	 * @return la face contenant le sommet � ajouter.
	 */
	private static Face nextConflictVertex(ConvexHull hull, Vector3f vertex) {

		float maxDistance = epsilon;
		Vector3f furthest = null;
		Face conflictFace = null;

		for (Face face : hull.faces) {

			float distance = face.getFurthestDistance();
			if (distance > maxDistance) {
				maxDistance = distance;
				furthest = face.getFurthest();
				conflictFace = face;
			}
		}

		if (furthest != null) {
			vertex.set(furthest);
			if (DEBUG) {
				System.out.println("\nNext conflict vertex: " + vertex + " Distance: " + maxDistance);
				System.out.println("Conflict face: " + conflictFace);
			}
		}

		return conflictFace;
	}

	/**
	 * Ajoute un sommet � l'enveloppe.
	 * 
	 * @param vertex Les coordonn�es du sommet.
	 * @parma conflictFace La face dont le sommet est le plus proche.
	 * @param hull
	 */
	private static void addVertexToHull(Vector3f vertex, Face conflictFace, ConvexHull hull) {

		List<HalfEdge> horizon = buildhorizon(vertex, conflictFace, hull);

		// System.out.println("Apr�s construction de l'horizon: " + hull);

		buildNewFaces(horizon, vertex, hull);

	}

	/**
	 * Calcule la liste des arr�tes d�limitant la zone "visible" par le sommet. Les
	 * arr�tes sont list�es CCW.
	 * 
	 * @param vertex
	 * @param conflictFace
	 * @param hull
	 * @return
	 */
	private static List<HalfEdge> buildhorizon(Vector3f vertex, Face conflictFace, ConvexHull hull) {

		List<HalfEdge> horizon = new ArrayList<HalfEdge>();

		conflictFace.computeHorizon(vertex, conflictFace.getEdge(), horizon);

		if (DEBUG) {
			System.out.println("Horizon: ");
		}
		// assert
		HalfEdge prev = horizon.get(horizon.size() - 1);
		for (HalfEdge current : horizon) {
			if (DEBUG) {
				System.out.println("	" + current);
			}

			if (prev.getTwin().getTail().getPosition() != current.getTail().getPosition()) {

				System.err.println("Arrrh l'horizon n'est pas une boucle !");
				System.exit(0);
			}
			prev = current;
		}

		return horizon;
	}

	/**
	 * Construit les nouvelles faces de l'enveloppe � partir de l'horizon et
	 * supprime les anciennes faces.
	 * 
	 * @param horizon
	 * @param hull
	 */
	private static void buildNewFaces(List<HalfEdge> horizon, Vector3f eyePos, ConvexHull hull) {

		Vertex newVertex = new Vertex(new Vector3f(eyePos));
		List<Face> newFaces = new ArrayList<Face>(horizon.size());

		Face previousFace = null;
		for (HalfEdge edge : horizon) {
			// Mark outer faces as unvisited.
			edge.getTwin().getFace().setVisited(false);

			HalfEdge prev = new HalfEdge(newVertex);
			HalfEdge next = new HalfEdge(edge.getTwin().getTail());

			Face face = new Face(edge, next, prev);
			newFaces.add(face);

			if (previousFace != null) {// Soude les faces adjacentes
				HalfEdge twin = previousFace.getEdge().getNext();
				HalfEdge.makeTwins(prev, twin);
			}
			previousFace = face;
		}

		// Soude la premi�re et la derni�re face.
		HalfEdge firstFacePrevEdge = newFaces.get(0).getEdge().getPrev();
		HalfEdge lastFaceNextEdge = newFaces.get(newFaces.size() - 1).getEdge().getNext();
		HalfEdge.makeTwins(firstFacePrevEdge, lastFaceNextEdge);

		// System.out.println("Apr�s suppression des anciennes faces: " + hull);

		mergeFaces(newFaces, hull);

		List<Vector3f> orphans = getOrphans(hull);

		// System.out.println("Apr�s fusion des nouvelles faces: " + hull);

		resolveOrphans(newFaces, hull, orphans);
	}

	/**
	 * Supprime les anciennes faces � l'int�rieur de l'horizon.
	 * 
	 * @return La liste des sommets en cours de traitement.
	 */
	private static List<Vector3f> getOrphans(ConvexHull hull) {

		List<Vector3f> orphans = new ArrayList<Vector3f>();
		Iterator<Face> it = hull.faces.iterator();
		while (it.hasNext()) {
			Face oldFace = it.next();
			if (oldFace.isVisited()) {
				// r�cup�re les sommets en cours de traitement.
				orphans.addAll(oldFace.getConflictList());
				// supprime les faces marqu�es comme supprim�es.
				it.remove();
			}
		}

		return orphans;
	}

	/**
	 * Int�gre les nouvelles faces � l'enveloppe actuelle, fusionne certaines faces
	 * si elles sont trop parall�les.
	 * 
	 * @param newFaces
	 * @param hull
	 * @param orphans
	 */
	private static void mergeFaces(List<Face> newFaces, ConvexHull hull) {

		for (Face face : newFaces) {
			if (face.isVisited()) {
				continue;// Already merged.
			}

			while (adjacentMerge(face)) {

			}

		}

		newFaces.removeIf(x -> x.isVisited());
		hull.faces.addAll(newFaces);
	}

	/**
	 * It�re sur les arr�tes d'une face et fusionne la premi�re face voisine non
	 * convexe ou coplanaire.
	 * 
	 * @param face
	 * @return true si il y a eu fusion.
	 */
	private static boolean adjacentMerge(Face face) {

		HalfEdge first = face.getEdge();
		HalfEdge edge = first;

		do {

			if (edge.isCoplanar()) {
				// merge !

				if (DEBUG) {
					System.out.println("Merging face !");
				}

				edge.absorbNeighborFace();

				if (DEBUG) {
					System.out.println("End of Merging face !");
				}

				return true;
			}

			edge = edge.getNext();
		} while (edge != first);

		return false;
	}

	/**
	 * R�assigne les sommets des listes de conflit des anciennes faces aux listes de
	 * conflits des nouvelles faces.
	 * 
	 * @param newFaces
	 * @param hull
	 */
	private static void resolveOrphans(List<Face> newFaces, ConvexHull hull, List<Vector3f> orphans) {

		for (Vector3f point : orphans) {
			hull.assignToConflictList(point, newFaces);
		}

	}

	public static int getMAX_ITERATIONS() {
		return MAX_ITERATIONS;
	}

	public static void setMAX_ITERATIONS(int mAX_ITERATIONS) {
		MAX_ITERATIONS = mAX_ITERATIONS;
	}

}
