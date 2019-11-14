package cataclysm.quickHull;

/**
 * 
 * Représente une arète d'une structure half edge.
 * 
 * @author Briac
 *
 */
public class HalfEdge {

	/**
	 * Une référence vers le sommet de départ de l'arrête.
	 */
	private Vertex tail;

	/**
	 * L'arête suivante de la face.
	 */
	HalfEdge next;

	/**
	 * L'arête précédente de la face.
	 */
	HalfEdge prev;

	/**
	 * L'arête equivalente appartenant à la face adjacente à cette arête.
	 */
	HalfEdge twin;

	/**
	 * La face possédant cette arête.
	 */
	private Face face;

	public HalfEdge(Vertex tailVertex) {
		this.tail = tailVertex;
	}

	public Vertex getTail() {
		return tail;
	}

	public void setTail(Vertex tail) {
		this.tail = tail;
	}

	public HalfEdge getNext() {
		return next;
	}

	public void setNext(HalfEdge next) {
		this.next = next;
	}

	public HalfEdge getPrev() {
		return prev;
	}

	public void setPrev(HalfEdge prev) {
		this.prev = prev;
	}

	public HalfEdge getTwin() {
		return twin;
	}

	public void setTwin(HalfEdge twin) {
		this.twin = twin;
	}

	public static void makeTwins(HalfEdge edgeA, HalfEdge edgeB) {
		edgeA.twin = edgeB;
		edgeB.twin = edgeA;
	}

	public Face getFace() {
		return face;
	}

	public void setFace(Face face) {
		this.face = face;
	}

	public String toString() {
		String str = new String("Edge: -tail: " + tail + " -head: " + twin.tail);

		return str;
	}

	/**
	 * Teste si les deux faces adjacentes de cette arrêtes sont considérées comme
	 * convexes.
	 * 
	 * @return
	 */
	public boolean isCoplanar() {
		Face f1 = face;
		Face f2 = twin.face;

		float d1 = f1.signedDistance(f2.getCenter());
		float d2 = f2.signedDistance(f1.getCenter());

		if (Math.abs(d1) < QuickHull.epsilon && Math.abs(d2) < QuickHull.epsilon) {
			return true;
		}

		return false;
	}

	/**
	 * Fusionne la face de l'arète voisine dans la face de cette
	 * arète.
	 */
	public void absorbNeighborFace() {

		twin.face.setVisited(true);// mark old face as deleted

		face.setEdge(prev);// change la référence de la face car cette arrête (this) sera supprimée.

		// indique aux arrêtes de la face voisine qu'elles appartiennent à cette face.
		HalfEdge it = twin.next;
		do {
			it.setFace(face);
			it = it.next;
		} while (it != twin);

		// lie les arrêtes entre elles.
		prev.next = twin.next;
		next.prev = twin.prev;

		twin.prev.next = next;
		twin.next.prev = prev;

		HalfEdge inA = prev;
		HalfEdge outA = prev.next;
		fixTopology(inA, outA);

		HalfEdge inB = next.prev;
		HalfEdge outB = next;
		fixTopology(inB, outB);

		face.rebuildPlane();
	}

	private void fixTopology(HalfEdge in, HalfEdge out) {

		if (in.twin.face == out.twin.face) {
			// problème: le sommet entre in et out n'a que deux faces adjacentes.

			Face neighbor = in.twin.face;
			if (neighbor.isTriangle()) {

				HalfEdge link = out.twin.prev;
				link.prev = in.prev;
				link.prev.next = link;

				link.next = out.next;
				link.next.prev = link;

				link.face.setVisited(true); // mark old face as deleted
				link.face = face;

			} else {

				in.next = out.next;
				in.next.prev = in;

				HalfEdge twin = out.twin;
				twin.next = twin.next.next;
				twin.next.prev = twin;

				in.twin = twin;
				twin.twin = in;

			}

		}

	}

}
