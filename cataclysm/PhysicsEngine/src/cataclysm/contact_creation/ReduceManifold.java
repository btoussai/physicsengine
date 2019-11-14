package cataclysm.contact_creation;

import java.util.List;

import org.lwjgl.util.vector.Vector3f;

import cataclysm.Epsilons;

class ReduceManifold {

	/**
	 * Quelques variables pour reduceManifold.
	 */
	private static final Vector3f QA = new Vector3f();
	private static final Vector3f QB = new Vector3f();
	private static final Vector3f QC = new Vector3f();
	private static final Vector3f QX_x_QY = new Vector3f();

	private static final Vector3f AB = new Vector3f();
	private static final Vector3f AC = new Vector3f();

	private static final Vector3f temp = new Vector3f();

	/**
	 * Supprime des points pour obtenir une surface de contact avec 4 points au
	 * maximum.
	 * 
	 * @param inputList
	 * @param planeNormal 
	 * @param contactPoints 
	 * @return le nombre de points de contact.
	 */
	static int reduceManifold(List<Vector3f> inputList, Vector3f planeNormal, Vector3f[] contactPoints) {

		if (inputList.size() <= 4) {
			
			for (int i = 0; i < inputList.size(); i++) {
				contactPoints[i].set(inputList.get(i));
			}
			return inputList.size();
		}

		Vector3f A = inputList.get(0);
		Vector3f B = null, C = null, D = null;

		// On maximise la distance entre A et B.
		float AB2 = 0;
		for (int i = 1; i < inputList.size(); i++) {
			Vector3f vertex = inputList.get(i);
			Vector3f.sub(vertex, A, AB);
			float d2 = AB.lengthSquared();
			if (d2 > AB2) {
				AB2 = d2;
				B = vertex;
			}
		}
		if (AB2 < Epsilons.MIN_LENGTH_2) {
			contactPoints[0].set(A);
			return 1;
		}
		Vector3f.sub(B, A, AB);

		// On maximise l'aire de ABC en valeur absolue.
		float areaABC = 0;
		for (int i = 1; i < inputList.size(); i++) {
			Vector3f vertex = inputList.get(i);
			Vector3f.sub(vertex, A, AC);

			Vector3f.cross(AB, AC, temp);
			float area = Vector3f.dot(planeNormal, temp);
			if (Math.abs(area) > Math.abs(areaABC)) {
				areaABC = area;
				C = vertex;
			}
		}
		if (Math.abs(areaABC) < Epsilons.MIN_LENGTH) {
			contactPoints[0].set(A);
			contactPoints[1].set(B);
			return 2;
		}

		// On �change B et C pour garder une orientation CCW.
		if (areaABC < 0) {

			temp.set(C);
			C.set(B);
			B.set(temp);

		}

		// On cherche D tel que ABCD soit le plus grand possible.
		float areaD = 0;
		for (Vector3f Q : inputList) {

			Vector3f.sub(A, Q, QA);
			Vector3f.sub(B, Q, QB);
			Vector3f.sub(C, Q, QC);
			
			Vector3f.cross(QA, QB, QX_x_QY);
			float area = Vector3f.dot(planeNormal, QX_x_QY);
			
			//On a area < 0 !
			if (area < areaD) {
				areaD = area;
				D = Q;
			}
			
			Vector3f.cross(QB, QC, QX_x_QY);
			area = Vector3f.dot(planeNormal, QX_x_QY);
			
			//On a area < 0 !
			if (area < areaD) {
				areaD = area;
				D = Q;
			}
			
			Vector3f.cross(QC, QA, QX_x_QY);
			area = Vector3f.dot(planeNormal, QX_x_QY);
			
			//On a area < 0 !
			if (area < areaD) {
				areaD = area;
				D = Q;
			}

		}
		if (Math.abs(areaD) < Epsilons.MIN_LENGTH) {
			contactPoints[0].set(A);
			contactPoints[1].set(B);
			contactPoints[2].set(C);
			return 3;
		}
		
		contactPoints[0].set(A);
		contactPoints[1].set(B);
		contactPoints[2].set(C);
		contactPoints[3].set(D);

		return 4;
	}

}
