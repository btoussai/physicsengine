package cataclysm.broadphase.staticmeshes;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import cataclysm.broadphase.AABB;
import math.vector.Vector3f;

/**
 * Repr�sente un noeud dans l'octree.
 * 
 * @author Briac
 *
 */
public class OctreeCell {

	private final Vector3f center;
	private final float size;
	private final float half_size;

	/**
	 * La profondeur de la cellule dans l'arbre de l'octree. La cellule a une
	 * profondeur valant 0 si c'est une feuille, sup�rieure � z�ro sinon.
	 */
	private final int depth;

	private OctreeCell[] children;
	private List<Triangle> triangles;

	OctreeCell(Vector3f center, float size, int depth) {
		this.center = center;
		this.size = size;
		this.half_size = 0.5f * size;
		this.depth = depth;
	}

	/**
	 * Construit les 8 noeuds fils de la cellule.
	 */
	private void buildChildren() {
		children = new OctreeCell[8];

		float offset = 0.25f * size;
		children[0] = new OctreeCell(new Vector3f(center.x - offset, center.y - offset, center.z - offset), half_size,
				depth - 1);
		children[1] = new OctreeCell(new Vector3f(center.x + offset, center.y - offset, center.z - offset), half_size,
				depth - 1);
		children[2] = new OctreeCell(new Vector3f(center.x - offset, center.y + offset, center.z - offset), half_size,
				depth - 1);
		children[3] = new OctreeCell(new Vector3f(center.x + offset, center.y + offset, center.z - offset), half_size,
				depth - 1);

		children[4] = new OctreeCell(new Vector3f(center.x - offset, center.y - offset, center.z + offset), half_size,
				depth - 1);
		children[5] = new OctreeCell(new Vector3f(center.x + offset, center.y - offset, center.z + offset), half_size,
				depth - 1);
		children[6] = new OctreeCell(new Vector3f(center.x - offset, center.y + offset, center.z + offset), half_size,
				depth - 1);
		children[7] = new OctreeCell(new Vector3f(center.x + offset, center.y + offset, center.z + offset), half_size,
				depth - 1);

	}

	/**
	 * Effectue l'insertion du triangle dans l'octree.
	 * 
	 * @param triangle
	 * @param min
	 * @param max
	 */
	void insertTriangle(Triangle triangle, Vector3f min, Vector3f max, Vector3f[] edges, Vector3f axis) {

		if (!intersectsTriangle(triangle, min, max, edges, axis)) {
			return;
		}

		if (depth == 0) {
			if (triangles == null) {
				triangles = new ArrayList<Triangle>(5);
			}
			triangles.add(triangle);
			return;
		}

		if (children == null) {
			buildChildren();
		}

		for (int i = 0; i < children.length; i++) {
			children[i].insertTriangle(triangle, min, max, edges, axis);
		}

	}

	/**
	 * Retire un triangle de l'octree.
	 * 
	 * @param triangle
	 * @param min
	 * @param max
	 */
	void removeTriangle(Triangle triangle, Vector3f min, Vector3f max, Vector3f[] edges, Vector3f axis) {

		if (!intersectsTriangle(triangle, min, max, edges, axis)) {
			return;
		}

		if (depth == 0) {
			if (triangles != null) {
				triangles.remove(triangle);
			}
			return;
		}

		if (children != null) {
			for (int i = 0; i < children.length; i++) {
				children[i].removeTriangle(triangle, min, max, edges, axis);
			}
		}
	}

	/**
	 * R�cup�re l'ensemble des triangles en intersection avec l'AABB
	 * 
	 * @param box
	 * @param dest
	 */
	void boxTest(AABB box, HashSet<Triangle> dest) {

		if (depth == 0) {
			if (triangles != null) {
				dest.addAll(triangles);
				return;
			}
		}

		if (children == null) {
			return;
		}

		int position = getPosition(box);
		for (int i = 0; i < children.length; i++) {
			if ((position & (1 << i)) != 0) {
				children[i].boxTest(box, dest);
			}
		}
	}

	/**
	 * Effectue un test de lancer de rayon.
	 * 
	 * @param start           Le point de d�part du rayon.
	 * @param dir             La direction du rayon, le vecteur doit �tre unitaire.
	 * @param maxLength       La distance maximale que le rayon est autoris� �
	 *                        parcourir.
	 * @param t0              La distance min d'intersection entre le rayon et la
	 *                        cellule
	 * @param t1              La distance max d'intersection entre le rayon et la
	 *                        cellule
	 * @param backfaceCulling true pour rejeter le triangle si N.dir > 0.
	 * @param normalDest      Permet de stocker la normale du point d'intersection.
	 * @return La distance au point d'intersection ou bien maxLength si le test
	 *         �choue.
	 */
	float rayTest(Vector3f start, Vector3f dir, float maxLength, float t0, float t1, boolean backfaceCulling,
			Vector3f normalDest) {

		if (depth == 0 && triangles != null) {
			for (Triangle triangle : triangles) {
				float d = triangle.rayTest(start, dir, maxLength, normalDest, backfaceCulling);
				if (d < maxLength) {
					normalDest.set(triangle.normal);
					maxLength = d;
				}
			}

		} else if (children != null) {

			int quadrant0 = getQuadrant(start, dir, t0);
			int quadrant1 = getQuadrant(start, dir, t1);

			if (quadrant0 == quadrant1) {
				float d = children[quadrant0].rayTest(start, dir, maxLength, t0, t1, backfaceCulling, normalDest);
				maxLength = Math.min(d, maxLength);
				return maxLength;
			}

			int xor = quadrant0 ^ quadrant1;
			float tx = (xor & 0b001) == 0 ? t1 : (center.x - start.x) / dir.x;
			float ty = (xor & 0b010) == 0 ? t1 : (center.y - start.y) / dir.y;
			float tz = (xor & 0b100) == 0 ? t1 : (center.z - start.z) / dir.z;

			int current_index = quadrant0;
			int next_index;
			float tmin = t0;
			do {
				tmin = Math.min(tx, Math.min(ty, tz));
				if (tx == tmin) {
					tx = t1;
					next_index = current_index ^ 0b001;
				} else if (ty == tmin) {
					ty = t1;
					next_index = current_index ^ 0b010;
				} else {
					tz = t1;
					next_index = current_index ^ 0b100;
				}

				if (t0 < maxLength && tmin > 0) {
					float d = children[current_index].rayTest(start, dir, maxLength, t0, tmin, backfaceCulling,
							normalDest);
					if(d < maxLength) {
						return d;
					}
				}

				current_index = next_index;
				t0 = tmin;

			} while (t0 != t1 && t0 < maxLength);
		}
		return maxLength;
	}

	/**
	 * Calcule un code binaire correspondant au noeud fils dans lequel se situe le
	 * point 'start + t * dir'.
	 * 
	 * @param start Le point de d�part du rayon
	 * @param dir   La direction du rayon
	 * @param t     L'avancement sur le rayon
	 * @return l'indice du noeud fils
	 */
	private int getQuadrant(Vector3f start, Vector3f dir, float t) {
		int quadrant = 0;
		float x = start.x + t * dir.x;
		float y = start.y + t * dir.y;
		float z = start.z + t * dir.z;
		if (x > center.x) {
			quadrant += 0b001;
		}
		if (y > center.y) {
			quadrant += 0b010;
		}
		if (z > center.z) {
			quadrant += 0b100;
		}
		return quadrant;
	}

	/**
	 * Calcule un code binaire permettant de d�terminer les noeuds fils que l'AABB intersecte.
	 * 
	 * @param min
	 * @param max
	 * @return
	 */
	private int getPosition(AABB box) {
		int position = 0b11111111;

		if (box.max.x < center.x) {
			position &= 0b01010101;
		} else if (box.min.x > center.x) {
			position &= 0b10101010;
		}

		if (box.max.y < center.y) {
			position &= 0b00110011;
		} else if (box.min.y > center.y) {
			position &= 0b11001100;
		}

		if (box.max.z < center.z) {
			position &= 0b00001111;
		} else if (box.min.z > center.z) {
			position &= 0b11110000;
		}

		return position;
	}

	/**
	 * Teste si le triangle est en intersection avec la cellule.
	 * 
	 * @param triangle
	 * @param min
	 * @param max
	 * @return false si le triangle est � l'ext�rieur.
	 */
	private boolean intersectsTriangle(Triangle triangle, Vector3f min, Vector3f max, Vector3f[] edges, Vector3f axis) {

		// On utilise le SAT (separating axis theorem)

		// Test avec les projections du triangle sur les axes XYZ.
		if (max.x < center.x - half_size || min.x > center.x + half_size) {
			return false;
		}
		if (max.y < center.y - half_size || min.y > center.y + half_size) {
			return false;
		}
		if (max.z < center.z - half_size || min.z > center.z + half_size) {
			return false;
		}

		// On projette la cellule sur la normale du triangle.
		Vector3f n = triangle.normal;
		Vector3f toCenter = new Vector3f();

		Vector3f.sub(center, triangle.v1, toCenter);
		if (normalTest(n, toCenter)) {
			return false;
		}

		// On teste avec les normales aux ar�tes du triangle
		// et avec les produits vectoriels des ar�tes du triangle et des axes XYZ

		// Vector3f.sub(center, triangle.p1, toCenter); //D�j� calcul� plus haut.
		// edge12 ^ n
		Vector3f.cross(edges[0], n, axis);
		if (axisTest(axis, toCenter)) {
			return false;
		}

		if (edgeTest(toCenter, edges[0], edges[2], axis)) {
			return false;
		}

		Vector3f.sub(center, triangle.v2, toCenter);
		// edge23 ^ n
		Vector3f.cross(edges[1], n, axis);
		if (axisTest(axis, toCenter)) {
			return false;
		}
		if (edgeTest(toCenter, edges[1], edges[0], axis)) {
			return false;
		}

		Vector3f.sub(center, triangle.v3, toCenter);
		// edge31 ^ n
		Vector3f.cross(edges[2], n, axis);
		if (axisTest(axis, toCenter)) {
			return false;
		}
		if (edgeTest(toCenter, edges[2], edges[1], axis)) {
			return false;
		}

		// On a pas trouv� d'axe de s�paration, la triangle est en intersection avec la
		// cellule.
		return true;
	}

	/**
	 * Teste si le sommet est � l'int�rieur de la cellule.
	 * 
	 * @param vertex
	 * @return
	 */
	boolean containsVertex(Vector3f v) {
		if (v.x >= center.x - half_size && v.x <= center.x + half_size) {
			if (v.y >= center.y - half_size && v.y <= center.y + half_size) {
				if (v.z >= center.z - half_size && v.z <= center.z + half_size) {
					return true;
				}
			}
		}

		return false;
	}

	/**
	 * Cherche un axe de s�paration parmi les produits vectoriels entre edge et les
	 * axes XYZ.
	 * 
	 * @param toCenter
	 * @param edge
	 * @param edgePrev
	 * @param axis
	 * @return true si un axe de s�paration � �t� trouv�.
	 */
	private boolean edgeTest(Vector3f toCenter, Vector3f edge, Vector3f edgePrev, Vector3f axis) {
		// edge ^ X
		axis.set(0, edge.z, -edge.y);
		if (Vector3f.dot(axis, edgePrev) < 0) {
			axis.negate();
		}
		if (axisTest(axis, toCenter)) {
			return true;
		}

		// edge ^ Y
		axis.set(-edge.z, 0, edge.x);
		if (Vector3f.dot(axis, edgePrev) < 0) {
			axis.negate();
		}
		if (axisTest(axis, toCenter)) {
			return true;
		}

		// edge ^ Z
		axis.set(edge.y, -edge.x, 0);
		if (Vector3f.dot(axis, edgePrev) < 0) {
			axis.negate();
		}
		if (axisTest(axis, toCenter)) {
			return true;
		}

		return false;
	}

	/**
	 * Teste si la normale du triangle est un axe de s�paration.
	 * 
	 * @param normal
	 * @param toCenter
	 * @return true si normal est un axe de s�paration.
	 */
	private boolean normalTest(Vector3f normal, Vector3f toCenter) {
		float proj = half_size * (Math.abs(normal.x) + Math.abs(normal.y) + Math.abs(normal.z));
		float offset = Vector3f.dot(normal, toCenter);
		return (offset + proj < 0 || offset - proj > 0);
	}

	/**
	 * Teste si axis est un axe de s�paration.
	 * 
	 * @param axis
	 * @param toCenter
	 * @return true si axis est un axe de s�paration.
	 */
	private boolean axisTest(Vector3f axis, Vector3f toCenter) {
		float offset = Vector3f.dot(axis, toCenter);
		float boxProj = half_size * (Math.abs(axis.x) + Math.abs(axis.y) + Math.abs(axis.z));
		return offset - boxProj > 0;
	}

	/**
	 * Explore l'octree et ajoute une boite affichable dans la liste boxes � chaque
	 * cellule rencontr�e.
	 * 
	 * @param boxes        La liste dans laquelle ranger les boites affichables.
	 * @param maxDepth     La profondeur maximale d'exploration.
	 * @param leavesOnly   N'ajoute que les cellules de profondeur maxDepth.
	 * @param nonVoidBoxes Ignore les cellules vides.
	 */
	public void exploreHierarchy(List<OctreeCellRenderable> boxes, int maxDepth, boolean leavesOnly,
			boolean nonVoidBoxes) {

		if (maxDepth > 0) {

			if (!leavesOnly && (children != null || !nonVoidBoxes)) {
				OctreeCellRenderable box = new OctreeCellRenderable(center, half_size, depth);
				boxes.add(box);
			}

			if (children != null) {
				for (OctreeCell child : children) {
					child.exploreHierarchy(boxes, maxDepth - 1, leavesOnly, nonVoidBoxes);
				}
			}

		} else if (maxDepth == 0 && (triangles != null || !nonVoidBoxes)) {
			OctreeCellRenderable box = new OctreeCellRenderable(center, half_size, depth);
			boxes.add(box);
		}

	}

}
