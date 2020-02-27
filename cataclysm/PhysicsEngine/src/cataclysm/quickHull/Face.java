package cataclysm.quickHull;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import cataclysm.wrappers.ConvexHullWrapperFace;
import cataclysm.wrappers.ConvexHullWrapperHalfEdge;
import math.vector.Vector3f;

/**
 * Repr�sente une face d'une structure half edge.
 * 
 * @author Briac
 *
 */
public class Face {

	private Vector3f normal = new Vector3f();
	private Vector3f center = new Vector3f();
	private float d_equ;
	private float area;
	
	/**
	 * Une r�ference vers une arr�te de cette face.
	 */
	private HalfEdge edge;

	/**
	 * La liste des points en dehors de l'enveloppe actuelle, dont la distance aux
	 * faces de l'enveloppe est minimale pour cette face.
	 */
	private List<Vector3f> conflictList = new ArrayList<Vector3f>();
	/**
	 * Le point appartenant � la {@link #conflictList} �tant le plus �loign� du
	 * plan.
	 */
	private Vector3f furthest;
	/**
	 * La distance au plan de {@link #furthest}.
	 */
	private float furthestDistance = 0;

	/**
	 * Indique si la face a d�j� �t� visit�e lors du calcul de l'horizon. voir
	 * {@link #computeHorizon}
	 */
	private boolean visited = false;

	/**
	 * Le constructeur � partir de trois arr�tes donn�es en ordre CCW. Les 3 arr�tes
	 * sont soud�es les unes au autres pendant l'appel.
	 * 
	 * @param A
	 * @param B
	 * @param C
	 */
	public Face(HalfEdge A, HalfEdge B, HalfEdge C) {
		edge = A;

		A.setFace(this);
		A.prev = C;
		A.next = B;

		B.setFace(this);
		B.prev = A;
		B.next = C;

		C.setFace(this);
		C.prev = B;
		C.next = A;

		rebuildPlane();

	}

	/**
	 * Le constructeur standard, on suppose que les sommets sont donn�es CCW.
	 * 
	 * @param A
	 * @param B
	 * @param C
	 */
	public Face(Vertex A, Vertex B, Vertex C) {

		buildEdges(A, B, C);

		rebuildPlane();

	}

	/**
	 * Le constructeur alternatif, l'ordre des sommets n'importe pas.
	 * 
	 * @param A
	 * @param B
	 * @param C
	 * @param normalDir La direction dans laquelle doit pointer la normale.
	 */
	public Face(Vertex A, Vertex B, Vertex C, Vector3f normalDir) {

		Vector3f AB = Vector3f.sub(B.getPosition(), A.getPosition());
		Vector3f AC = Vector3f.sub(C.getPosition(), A.getPosition());
		Vector3f.cross(AB, AC, normal);

		if (Vector3f.dot(normalDir, normal) > 0) {
			buildEdges(A, B, C);
		} else {
			buildEdges(A, C, B);
		}

		rebuildPlane();

	}

	public float signedDistance(Vector3f position) {
		return Vector3f.dot(position, normal) + d_equ;
	}

	private void buildEdges(Vertex A, Vertex B, Vertex C) {

		HalfEdge AB = new HalfEdge(A);
		AB.setFace(this);

		HalfEdge BC = new HalfEdge(B);
		BC.setFace(this);

		HalfEdge CA = new HalfEdge(C);
		CA.setFace(this);

		AB.next = BC;
		AB.prev = CA;

		BC.next = CA;
		BC.prev = AB;

		CA.next = AB;
		CA.prev = BC;

		this.edge = AB;

	}

	/**
	 * Calcule l'horizon pour un point donn�. Il s'agit des arr�tes d�limitant les
	 * faces visibles depuis ce point.
	 * 
	 * @param eyePos
	 * @param startEdge
	 * @param horizon
	 * @param epsilon 
	 */
	public void computeHorizon(Vector3f eyePos, HalfEdge startEdge, List<HalfEdge> horizon, float epsilon) {

		visited = true;
		HalfEdge currentEdge = startEdge;
		do {

			// Cross currentEdge
			HalfEdge twinEdge = currentEdge.getTwin();
			Face neighborFace = twinEdge.getFace();

			if (!neighborFace.visited) {

				boolean visible = neighborFace.signedDistance(eyePos) > epsilon;
				if (visible) {// continue the search in the neighbor face
					neighborFace.computeHorizon(eyePos, twinEdge.getNext(), horizon, epsilon);
				} else {
					// neighborFace.setVisited(true);// visit neighbor
					horizon.add(currentEdge);
				}

			}
			currentEdge = currentEdge.getNext();

		} while (currentEdge != startEdge);

	}

	/**
	 * Builds the best fitting plane equation for this face with the Newell
	 * algorithm.
	 */
	public void rebuildPlane() {

		normal.set(0, 0, 0);
		center.set(0, 0, 0);

		HalfEdge current = edge;
		HalfEdge previous = edge.prev;

		Vector3f A = previous.getTail().getPosition();
		Vector3f B = current.getTail().getPosition();

		int n = 0;
		do {
			normal.x += (A.y - B.y) * (A.z + B.z);
			normal.y += (A.z - B.z) * (A.x + B.x);
			normal.z += (A.x - B.x) * (A.y + B.y);

			Vector3f.add(center, B, center);

			previous = current;
			A = B;
			current = current.next;
			B = current.getTail().getPosition();
			n++;
		} while (current != edge);

		center.scale(1.0f / n);

		normal.normalise();
		d_equ = -Vector3f.dot(center, normal);

		computeArea();
	}

	public List<Vector3f> getConflictList() {
		return conflictList;
	}

	public HalfEdge getEdge() {
		return edge;
	}

	public void setEdge(HalfEdge edge) {
		this.edge = edge;
	}

	public boolean isVisited() {
		return visited;
	}

	public void setVisited(boolean visited) {
		this.visited = visited;
	}

	public Vector3f getCenter() {
		return center;
	}

	@Override
	public String toString() {
		String str = new String("Face: -normal:" + normal + " -center: " + center + " -visited:" + visited);

		return str;
	}

	public float getFurthestDistance() {
		return furthestDistance;
	}

	public void setFurthestDistance(float furthestDistance) {
		this.furthestDistance = furthestDistance;
	}

	public Vector3f getFurthest() {
		return furthest;
	}

	public void setFurthest(Vector3f furthest) {
		this.furthest = furthest;
	}

	public Vector3f getNormal() {
		return normal;
	}

	public boolean isTriangle() {
		return edge == edge.next.next.next;
	}

	private void computeArea() {
		float area = 0;

		Vector3f AB = new Vector3f();
		Vector3f BC = new Vector3f();
		Vector3f cross = new Vector3f();

		HalfEdge edge = this.edge;
		Vector3f.sub(edge.getTail().getPosition(), edge.prev.getTail().getPosition(), BC);

		do {
			edge = edge.next;
			AB = BC;
			Vector3f.sub(edge.getTail().getPosition(), edge.prev.getTail().getPosition(), BC);
			Vector3f.cross(AB, BC, cross);
			area += Vector3f.dot(normal, cross);

		} while (edge != this.edge);

		this.area = Math.abs(area);
	}

	public float getArea() {
		return area;
	}

	public void convertToWrapperFace(Map<HalfEdge, ConvexHullWrapperHalfEdge> map,
			List<ConvexHullWrapperFace> wFaces, List<Vector3f> wNormals, List<Vector3f> wCentroids) {

		ConvexHullWrapperFace copyFace = new ConvexHullWrapperFace();
		copyFace.setIndex(wFaces.size());
		wFaces.add(copyFace);
		wNormals.add(new Vector3f(this.getNormal()));
		wCentroids.add(new Vector3f(this.getCenter()));

		ConvexHullWrapperHalfEdge copyEdgePrev = new ConvexHullWrapperHalfEdge();
		copyEdgePrev.setFace(wFaces.size()-1);

		map.put(edge, copyEdgePrev);

		HalfEdge edge = this.edge.next;
		ConvexHullWrapperHalfEdge copyEdge = null;

		do {
			copyEdge = new ConvexHullWrapperHalfEdge();
			copyEdge.setFace(wFaces.size()-1);

			map.put(edge, copyEdge);

			edge = edge.next;
		} while (edge != this.edge);

	}

}
