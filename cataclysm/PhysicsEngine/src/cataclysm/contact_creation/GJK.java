package cataclysm.contact_creation;

import org.lwjgl.util.vector.Vector3f;

import cataclysm.wrappers.Wrapper;

/**
 * Cette classe contient une impl�mentation de l'algorithme GJK permettant de
 * calculer la distance entre deux convexes. D'apr�s la pr�sentation "Erin Catto
 * � Computing Distance using GJK" disponible ici <a
 * href=https://box2d.org/downloads/>https://box2d.org/downloads/</a>.
 * 
 * @author Briac
 *
 */
class GJK {

	private static Simplex simplex = new Simplex();
	private static Simplex previousSimplex = new Simplex();
	private static final Vector3f direction = new Vector3f();
	private static final SimplexVertex support = new SimplexVertex();

	private static final Vector3f AB = new Vector3f();
	private static final Vector3f AC = new Vector3f();
	private static final Vector3f AD = new Vector3f();

	private static final Vector3f triangleNormal = new Vector3f();
	private static final Vector3f temp = new Vector3f();

	private static final Vector3f ABC_normal = new Vector3f();
	private static final Vector3f ACD_normal = new Vector3f();
	private static final Vector3f ADB_normal = new Vector3f();

	private static boolean intersectionFound = false;
	private static boolean repeatedVertex = false;

	private static final boolean DEBUG = false;

	/**
	 * Repr�sente un sommet du simplex.
	 * 
	 * @author Briac
	 *
	 */
	private static class SimplexVertex {

		Vector3f position;
		Vector3f positionOnA;
		Vector3f positionOnB;

		SimplexVertex() {
			position = new Vector3f();
			positionOnA = new Vector3f();
			positionOnB = new Vector3f();
		}

		@Override
		public String toString() {
			return "Pos: " + position + " PosOnA: " + positionOnA + " PosOnB: " + positionOnB;
		}

		public void set(SimplexVertex vertex) {
			if (vertex != this) {
				position.set(vertex.position);
				positionOnA.set(vertex.positionOnA);
				positionOnB.set(vertex.positionOnB);
			}
		}

		public void blend2(SimplexVertex v1, float w1, SimplexVertex v2, float w2) {
			blend2(v1.position, w1, v2.position, w2, position);
			blend2(v1.positionOnA, w1, v2.positionOnA, w2, positionOnA);
			blend2(v1.positionOnB, w1, v2.positionOnB, w2, positionOnB);
		}

		public void blend3(SimplexVertex v1, float w1, SimplexVertex v2, float w2, SimplexVertex v3, float w3) {
			blend3(v1.position, w1, v2.position, w2, v3.position, w3, position);
			blend3(v1.positionOnA, w1, v2.positionOnA, w2, v3.positionOnA, w3, positionOnA);
			blend3(v1.positionOnB, w1, v2.positionOnB, w2, v3.positionOnB, w3, positionOnB);
		}

		private static void blend2(Vector3f v1, float w1, Vector3f v2, float w2, Vector3f dest) {
			dest.x = v1.x * w1 + v2.x * w2;
			dest.y = v1.y * w1 + v2.y * w2;
			dest.z = v1.z * w1 + v2.z * w2;
		}

		private static void blend3(Vector3f v1, float w1, Vector3f v2, float w2, Vector3f v3, float w3, Vector3f dest) {
			dest.x = v1.x * w1 + v2.x * w2 + v3.x * w3;
			dest.y = v1.y * w1 + v2.y * w2 + v3.y * w3;
			dest.z = v1.z * w1 + v2.z * w2 + v3.z * w3;
		}

	}

	/**
	 * Repr�sente le simplex utilis� dans l'algorithme: un point, un segment, un
	 * triangle ou un t�trah�dre.
	 * 
	 * @author Briac
	 *
	 */
	private static class Simplex {

		/**
		 * Repr�sente le type du simplexe.
		 * 
		 * @author Briac
		 *
		 */
		enum SimplexType {
			Tetrahedron(4, null), Triangle(3, Tetrahedron), Segment(2, Triangle), Vertex(1, Segment);

			int simplexVertexCount;
			SimplexType next;

			SimplexType(int count, SimplexType next) {
				this.simplexVertexCount = count;
				this.next = next;
			}

			SimplexType evolve() {
				return next;
			}
		}

		SimplexVertex A = new SimplexVertex();
		SimplexVertex B = new SimplexVertex();
		SimplexVertex C = new SimplexVertex();
		SimplexVertex D = new SimplexVertex();

		SimplexVertex closest = new SimplexVertex();
		float closestDistance;

		SimplexType type;

		private Simplex() {

		}

		@Override
		public String toString() {
			String string = type + "-Simplex:";
			if (type.simplexVertexCount >= 1)
				string += "\nA        " + A;
			if (type.simplexVertexCount >= 2)
				string += "\nB        " + B;
			if (type.simplexVertexCount >= 3)
				string += "\nC        " + C;
			if (type.simplexVertexCount >= 4)
				string += "\nD        " + D;
			string += "\nclosestDistance  " + closestDistance;
			string += "\nClosest  " + closest;
			return string;
		}

		/**
		 * Initialise le simplexe et range la direction de recherche dans searchDir.
		 * 
		 * @param bodyA
		 * @param bodyB
		 * @param searchDir
		 */
		public void init(Wrapper bodyA, Wrapper bodyB, Vector3f searchDir) {
			A.positionOnA.set(bodyA.getCentroid());
			A.positionOnB.set(bodyB.getCentroid());

			Vector3f.sub(A.positionOnA, A.positionOnB, A.position);
			A.position.negate(searchDir);

			closestDistance = Float.POSITIVE_INFINITY;
			type = SimplexType.Vertex;
		}

		/**
		 * Recopie le simplexe pr�c�dent en ajoutant un sommet pour construire le
		 * nouveau simplexe.
		 * 
		 * @param previous
		 * 
		 * @param point
		 */
		public void addVertex(Simplex previous, SimplexVertex point) {
			switch (previous.type) {
			case Tetrahedron:
			case Triangle:
				if (point.position.equals(previous.C.position)) {
					repeatedVertex = true;
					return;
				}
			case Segment:
				if (point.position.equals(previous.B.position)) {
					repeatedVertex = true;
					return;
				}
			case Vertex:
				if (point.position.equals(previous.A.position)) {
					repeatedVertex = true;
					return;
				}
			}

			type = previous.type.evolve();

			A.set(point);
			switch (previous.type) {
			case Tetrahedron:
			case Triangle:
				D.set(previous.C);
			case Segment:
				C.set(previous.B);
			case Vertex:
				B.set(previous.A);
			}

			closestDistance = previous.closestDistance;
			closest.set(previous.closest);
		}

		public void permutation(SimplexVertex v1, SimplexVertex v2, SimplexVertex v3, SimplexVertex v4,
				SimplexType newType) {
			A = v1;
			B = v2;
			C = v3;
			D = v4;
			this.type = newType;
		}

	}

	/**
	 * Calcule la distance de séparation entre deux solides convexes.
	 * 
	 * @param bodyA      Le solide A
	 * @param bodyB      Le solide B
	 * @param closestOnA Un vecteur dans lequel stocker la position du point le plus
	 *                   proche de B appartenant � A. Null est valide.
	 * @param closestOnB Un vecteur dans lequel stocker la position du point le plus
	 *                   proche de A appartenant � B. Null est valide.
	 * @return La distance de s�paration (positive) ou -Float.MAX_VALUE si les
	 *         solides se touchent.
	 */
	static float distance(Wrapper bodyA, Wrapper bodyB, Vector3f closestOnA, Vector3f closestOnB) {

		previousSimplex.init(bodyA, bodyB, direction);

		if (DEBUG) {
			System.out.println("######GJK");
			System.out.println("Init: " + previousSimplex);
		}

		checkIntersection(bodyA, bodyB);

		// System.exit(0);

		if (intersectionFound) {
			if (DEBUG) {
				System.out.println("Les deux solides sont en intersection, fin.");
			}
			return -Float.MAX_VALUE;
		} else {

			if (closestOnA != null) {
				closestOnA.set(simplex.closest.positionOnA);
			}
			if (closestOnB != null) {
				closestOnB.set(simplex.closest.positionOnB);
			}

			float bestDistance = (float) Math.sqrt(simplex.closestDistance);

			if (DEBUG) {
				System.out.println("\nClosest: " + simplex.closest);
				System.out.println("bestDistance: " + bestDistance);
			}

			return bestDistance;
		}
	}

	/**
	 * Permet de r�cup�rer la partie du simplexe la plus proche de l'origine apr�s
	 * un appel � {@link GJK#distance(Wrapper, Wrapper, Vector3f, Vector3f)} ayant
	 * aboutit � une s�paration entre les deux convexes.
	 * 
	 * @param dest
	 */
	static void getClosestFeatureOnA(ContactFeature dest) {
		Vector3f v1, v2, v3;
		switch (simplex.type) {
		case Triangle:
			v1 = simplex.A.positionOnA;
			v2 = simplex.B.positionOnA;
			v3 = simplex.C.positionOnA;

			if (v1.equals(v2)) {
				if (v1.equals(v3)) {
					dest.setFrom(v1);
				} else {
					dest.setFrom(v1, v3);
				}
			} else {
				if (v1.equals(v3)) {
					dest.setFrom(v1, v2);
				} else {
					if (v2.equals(v3)) {
						dest.setFrom(v1, v2);
					} else {
						dest.setFrom(v1, v2, v3);
					}
				}
			}
			break;
		case Segment:
			v1 = simplex.A.positionOnA;
			v2 = simplex.B.positionOnA;
			if (v1.equals(v2)) {
				dest.setFrom(v1);
			} else {
				dest.setFrom(v1, v2);
			}
			break;
		case Vertex:
			v1 = simplex.A.positionOnA;
			dest.setFrom(v1);
			break;
		case Tetrahedron:
		default:
			throw new ArithmeticException(
					"Erreur dans GJK, les objets sont en contact, impossible de r�cup�rer la feature la plus proche de l'origine");
		}

	}

	/**
	 * Permet de r�cup�rer la partie du simplexe la plus proche de l'origine apr�s
	 * un appel � {@link GJK#distance(Wrapper, Wrapper, Vector3f, Vector3f)} ayant
	 * aboutit � une s�paration entre les deux convexes.
	 * 
	 * @param dest
	 */
	static void getClosestFeatureOnB(ContactFeature dest) {
		Vector3f v1, v2, v3;
		switch (simplex.type) {
		case Triangle:
			v1 = simplex.A.positionOnB;
			v2 = simplex.B.positionOnB;
			v3 = simplex.C.positionOnB;

			if (v1.equals(v2)) {
				if (v1.equals(v3)) {
					dest.setFrom(v1);
				} else {
					dest.setFrom(v1, v3);
				}
			} else {
				if (v1.equals(v3)) {
					dest.setFrom(v1, v2);
				} else {
					if (v2.equals(v3)) {
						dest.setFrom(v1, v2);
					} else {
						dest.setFrom(v1, v2, v3);
					}
				}
			}
			break;
		case Segment:
			v1 = simplex.A.positionOnB;
			v2 = simplex.B.positionOnB;
			if (v1.equals(v2)) {
				dest.setFrom(v1);
			} else {
				dest.setFrom(v1, v2);
			}
			break;
		case Vertex:
			v1 = simplex.A.positionOnB;
			dest.setFrom(v1);
			break;
		case Tetrahedron:
		default:
			throw new ArithmeticException(
					"Erreur dans GJK, les objets sont en contact, impossible de r�cup�rer la feature la plus proche de l'origine");
		}

	}

	/**
	 * Teste l'intersection entre les deux solides.
	 * 
	 */
	private static void checkIntersection(Wrapper bodyA, Wrapper bodyB) {
		intersectionFound = false;
		repeatedVertex = false;

		do {

			if (DEBUG) {
				System.out.println("\n\nDirection: " + direction);
			}

			getSupportPoint(bodyA, bodyB, direction, support);

			if (DEBUG) {
				System.out.println("\nSupport point: " + support);
			}

			simplex.addVertex(previousSimplex, support);

			if (repeatedVertex) {
				if (DEBUG) {
					System.out.println("\n## STOP: R�p�tition d'un sommet !");
				}
				swapSimplex();
				return;
			}

			if (DEBUG) {
				System.out.println("\nApr�s ajout: " + simplex);
			}

			getClosestPoint();

			if (DEBUG) {
				System.out.println("\nFin it�ration: " + simplex);
			}

			if (intersectionFound) {
				if (DEBUG) {
					System.out.println("\n## STOP: intersection trouv�e !");
				}
				return;
			}

			Vector3f closest = simplex.closest.position;
			float distance = closest.lengthSquared();
			if (distance < 1E-6f) {
				intersectionFound = true;
				if (DEBUG) {
					System.out.println("\n## STOP: intersection trouv�e !");
				}
				return;
			} else if (distance < simplex.closestDistance) {
				simplex.closestDistance = distance;
			} else {
				// La distance ne progresse pas, on r�tablit le simplexe de l'it�ration
				// pr�c�dente.
				swapSimplex();
				if (DEBUG) {
					System.out.println("\n## STOP: la distance ne progresse pas.");
				}
				return;
			}

			closest.negate(direction);
			swapSimplex();

		} while (true);
	}

	/**
	 * Cherche le point appartenant au simplex se situant le plus proche de
	 * l'origine. Place le r�sultat dans simplex.closest
	 */
	private static void getClosestPoint() {

		switch (simplex.type) {
		case Segment:
			getClosestPointOnSegment();
			break;
		case Triangle:
			getClosestPointOnTriangle();
			break;
		case Tetrahedron:
			getClosestPointOnTetrahedron();
			break;
		default:
			throw new ArithmeticException("Erreur dans GJK !");
		}

	}

	private static void getClosestPointOnSegment() {

		Vector3f OA = simplex.A.position;
		Vector3f OB = simplex.B.position;
		Vector3f.sub(OB, OA, AB);

		float AB2 = AB.lengthSquared();

		float u = Vector3f.dot(OB, AB) / AB2;

		if (u >= 1.0f) {
			if (DEBUG)
				System.out.println("Segment --> A");
			simplex.closest.set(simplex.A);
			simplex.type = Simplex.SimplexType.Vertex;
		} else {
			if (DEBUG)
				System.out.println("Segment --> Segment");
			simplex.closest.blend2(simplex.A, u, simplex.B, 1.0f - u);
		}

	}

	private static void getClosestPointOnTriangle() {

		Vector3f OA = simplex.A.position;
		Vector3f OB = simplex.B.position;
		Vector3f OC = simplex.C.position;

		Vector3f.sub(OB, OA, AB);
		Vector3f.sub(OC, OA, AC);

		float AB2 = AB.lengthSquared();
		float AC2 = AC.lengthSquared();

		float uAB = Vector3f.dot(OB, AB) / AB2;
		float uAC = Vector3f.dot(OC, AC) / AC2;
		if (uAC >= 1.0f && uAB >= 1.0f) {
			if (DEBUG)
				System.out.println("Triangle --> A");
			simplex.closest.set(simplex.A);
			simplex.type = Simplex.SimplexType.Vertex;
			return;
		}

		Vector3f.cross(AB, AC, triangleNormal);
		float areaABC2 = triangleNormal.lengthSquared();

		float wABC = Vector3f.dot(Vector3f.cross(OA, OB, temp), triangleNormal) / areaABC2;
		if (wABC <= 0.0f) {
			if (DEBUG)
				System.out.println("Triangle --> AB");
			simplex.closest.blend2(simplex.A, uAB, simplex.B, 1.0f - uAB);
			simplex.type = Simplex.SimplexType.Segment;
			return;
		}

		float vABC = Vector3f.dot(Vector3f.cross(OC, OA, temp), triangleNormal) / areaABC2;
		if (vABC <= 0.0f) {
			if (DEBUG)
				System.out.println("Triangle --> AC");
			simplex.closest.blend2(simplex.A, uAC, simplex.C, 1.0f - uAC);
			simplex.permutation(simplex.A, simplex.C, simplex.B, simplex.D, Simplex.SimplexType.Segment);
			return;
		}

		// uABC > 0.0f && vABC > 0.0f && wABC > 0.0f
		simplex.closest.blend3(simplex.A, 1.0f - vABC - wABC, simplex.B, vABC, simplex.C, wABC);

		if (Vector3f.dot(triangleNormal, OA) > 0) {// origin under ABC plane
			simplex.permutation(simplex.A, simplex.C, simplex.B, simplex.D, Simplex.SimplexType.Triangle);
		}

		if (DEBUG)
			System.out.println("Triangle --> Triangle");

	}

	private static void getClosestPointOnTetrahedron() {

		Vector3f OA = simplex.A.position;
		Vector3f OB = simplex.B.position;
		Vector3f OC = simplex.C.position;
		Vector3f OD = simplex.D.position;

		Vector3f.sub(OB, OA, AB);
		Vector3f.sub(OC, OA, AC);
		Vector3f.sub(OD, OA, AD);

		float AB2 = AB.lengthSquared();
		float AC2 = AC.lengthSquared();
		float AD2 = AD.lengthSquared();

		float uAB = Vector3f.dot(OB, AB) / AB2;
		float uAC = Vector3f.dot(OC, AC) / AC2;
		float uAD = Vector3f.dot(OD, AD) / AD2;
		if (uAC >= 1.0f && uAB >= 1.0f && uAD >= 1.0f) {
			if (DEBUG)
				System.out.println("Tetra�dre --> A");
			simplex.closest.set(simplex.A);
			simplex.type = Simplex.SimplexType.Vertex;
			return;
		}

		Vector3f.cross(AB, AC, ABC_normal);
		float areaABC2 = ABC_normal.lengthSquared();

		Vector3f.cross(AC, AD, ACD_normal);
		float areaACD2 = ACD_normal.lengthSquared();

		Vector3f.cross(AD, AB, ADB_normal);
		float areaADB2 = ADB_normal.lengthSquared();

		Vector3f.cross(OA, OB, temp);
		float wABC = Vector3f.dot(temp, ABC_normal) / areaABC2;
		float vADB = -Vector3f.dot(temp, ADB_normal) / areaADB2;

		if (wABC <= 0.0f && vADB <= 0.0f) {
			if (DEBUG)
				System.out.println("Tetra�dre --> AB");
			if (uAB < 0.0f)
				uAB = 0.0f;
			simplex.closest.blend2(simplex.A, uAB, simplex.B, 1.0f - uAB);
			simplex.type = Simplex.SimplexType.Segment;
			return;
		}

		Vector3f.cross(OA, OC, temp);
		float wACD = Vector3f.dot(temp, ACD_normal) / areaACD2;
		float vABC = -Vector3f.dot(temp, ABC_normal) / areaABC2;

		if (wACD <= 0.0f && vABC <= 0.0f) {
			if (DEBUG)
				System.out.println("Tetra�dre --> AC");
			if (uAC < 0.0f)
				uAC = 0.0f;
			simplex.closest.blend2(simplex.A, uAC, simplex.C, 1.0f - uAC);
			simplex.permutation(simplex.A, simplex.C, simplex.B, simplex.D, Simplex.SimplexType.Segment);
			return;
		}

		Vector3f.cross(OA, OD, temp);
		float wADB = Vector3f.dot(temp, ADB_normal) / areaADB2;
		float vACD = -Vector3f.dot(temp, ACD_normal) / areaACD2;

		if (wADB <= 0.0f && vACD <= 0.0f) {
			if (DEBUG)
				System.out.println("Tetra�dre --> AD");
			if (uAD < 0.0f)
				uAD = 0.0f;
			simplex.closest.blend2(simplex.A, uAD, simplex.D, 1.0f - uAD);
			simplex.permutation(simplex.A, simplex.D, simplex.C, simplex.B, Simplex.SimplexType.Segment);
			return;
		}

		if (vABC >= 0.0f && wABC >= 0.0f && Vector3f.dot(ABC_normal, OA) < 0.0f) {// origin above ABC
			if (DEBUG)
				System.out.println("Tetra�dre --> ABC");
			simplex.closest.blend3(simplex.A, 1.0f - vABC - wABC, simplex.B, vABC, simplex.C, wABC);
			simplex.type = Simplex.SimplexType.Triangle;
			return;
		}

		if (vACD >= 0.0f && wACD >= 0.0f && Vector3f.dot(ACD_normal, OA) < 0.0f) {// origin above ACD
			if (DEBUG)
				System.out.println("Tetra�dre --> ACD");
			simplex.closest.blend3(simplex.A, 1.0f - vACD - wACD, simplex.C, vACD, simplex.D, wACD);
			simplex.permutation(simplex.A, simplex.C, simplex.D, simplex.B, Simplex.SimplexType.Triangle);
			return;
		}

		if (vADB >= 0.0f && wADB >= 0.0f && Vector3f.dot(ADB_normal, OA) < 0.0f) {// origin above ADB
			if (DEBUG)
				System.out.println("Tetra�dre --> ADB");
			simplex.closest.blend3(simplex.A, 1.0f - vADB - wADB, simplex.D, vADB, simplex.B, wADB);
			simplex.permutation(simplex.A, simplex.D, simplex.B, simplex.C, Simplex.SimplexType.Triangle);
			return;
		}

		intersectionFound = true;// origin inside tetrahedron.
	}

	/**
	 * Cherche le point de support, c'est le point le plus �loign� dans la direction
	 * searchDir de l'ensemble (bodyA-bodyB). Stocke le r�sultat dans support.
	 * 
	 * @param bodyA
	 * @param bodyB
	 * @param searchDir
	 * @param support
	 */
	private static void getSupportPoint(Wrapper bodyA, Wrapper bodyB, Vector3f searchDir, SimplexVertex support) {
		bodyA.getSupport(searchDir, false, support.positionOnA);
		bodyB.getSupport(searchDir, true, support.positionOnB);
		Vector3f.sub(support.positionOnA, support.positionOnB, support.position);
	}

	/**
	 * Echange les r�les de simplex et previousSimplex.
	 */
	private static void swapSimplex() {
		Simplex temp = previousSimplex;
		previousSimplex = simplex;
		simplex = temp;
	}

}
