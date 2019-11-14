package cataclysm.wrappers;

import org.lwjgl.util.vector.Matrix3f;
import org.lwjgl.util.vector.Vector3f;

/**
 * Cette classe fournit une impl�mentation de l'algorithme de B. Mirtich pour le
 * calcul des propri�t�s massiques d'un solide polygonal. D'apr�s l'article
 * "Fast and Accurate Computation of Polyhedral Mass Properties".
 * 
 * Un ajout personnel permet de traiter le cas d'un solide creux, o� la mati�re
 * se concentre sur la surface.
 * 
 * @author Briac
 *
 */
class PolyhedralMassProperties {

	/**
	 * Repr�sente le plan de projection utilis� pour calculer les int�grales
	 * surfaciques sur une face. La projection maximise l'aire de la face sur le
	 * plan de projection.
	 * 
	 * @author Briac
	 *
	 */
	private static enum Projection {
		XY, YZ, ZX;
	}

	/**
	 * La plan de projection actuel.
	 */
	private static Projection projection;

	/**
	 * Calcule le volume, la position du centre de masse et le tenseur d'inertie du
	 * solide. On suppose le solide de densit� uniforme.
	 * 
	 * @param hull    Le solide dont on souhaite caculer les propri�t�s.
	 * @param CM      Le vecteur dans lequel stocker les coordonn�es du centre de
	 *                masse.
	 * @param inertia La matrice dans laquelle stocker les coordonn�es du tenseur
	 *                d'inertie. Le tenseur d'inertie est exprim� par rapport �
	 *                l'origine du rep�re.
	 * @return la masse du solide.
	 */
	public static float computeProperties(ConvexHullWrapper hull, Vector3f CM, Matrix3f inertia) {

		computeVolumeIntegrals(hull.getData());

		float rx = 0, ry = 0, rz = 0;

		float volume = T1;
		float surfaceArea = U1;

		MassProperties massProperties = hull.getMassProperties();
		massProperties.setVolume(volume);
		massProperties.setSurfaceArea(surfaceArea);
		float mass = massProperties.computeMass();

		if (!massProperties.isHollow()) {

			float T1_i = 1 / T1;

			rx = Tx * T1_i;
			ry = Ty * T1_i;
			rz = Tz * T1_i;

			inertia.m00 = ((Ty2 + Tz2) * T1_i) * mass;
			inertia.m11 = ((Tx2 + Tz2) * T1_i) * mass;
			inertia.m22 = ((Tx2 + Ty2) * T1_i) * mass;

			inertia.m01 = inertia.m10 = -(Txy * T1_i) * mass;
			inertia.m02 = inertia.m20 = -(Tzx * T1_i) * mass;
			inertia.m12 = inertia.m21 = -(Tyz * T1_i) * mass;

		} else {

			float U1_i = 1 / U1;

			rx = Ux * U1_i;
			ry = Uy * U1_i;
			rz = Uz * U1_i;

			inertia.m00 = ((Uy2 + Uz2) * U1_i) * mass;
			inertia.m11 = ((Ux2 + Uz2) * U1_i) * mass;
			inertia.m22 = ((Ux2 + Uy2) * U1_i) * mass;

			inertia.m01 = inertia.m10 = -(Uxy * U1_i) * mass;
			inertia.m02 = inertia.m20 = -(Uzx * U1_i) * mass;
			inertia.m12 = inertia.m21 = -(Uyz * U1_i) * mass;

		}

		CM.set(rx, ry, rz);

		// System.out.println("CM: " + CM);
		// System.out.println("Volume: " + T1 + " Surface Area: " + U1);
		// System.out.println("Inertia :\n" + inertia);

		return mass;
	}

	// U<x> --> int�grale sur la surface de <x>.
	private static float U1, Ux, Uy, Uz, Ux2, Uy2, Uz2, Uxy, Uyz, Uzx;
	// T<x> --> in�grale sur le volume de <x>.
	private static float T1, Tx, Ty, Tz, Tx2, Ty2, Tz2, Txy, Tyz, Tzx;
	// F<x> --> int�grale sur la surface de <x>
	private static float F1, Fa, Fb, Fc, Fab, Fbc, Fca, Fa2, Fb2, Fc2, Fa3, Fb3, Fc3, Fa2b, Fb2c, Fc2a;
	// Composantes de la normale d'une face
	private static float na, nb, nc;
	// PI<x> --> int�grale sur le contour d'une face de <x>
	private static float pi_1, pi_a, pi_b, pi_a2, pi_b2, pi_a3, pi_b3;
	private static float pi_ab, pi_a2b, pi_ab2;
	private static float a0, a1, b0, b1;

	private static void computeVolumeIntegrals(ConvexHullWrapperData data) {

		T1 = Tx = Ty = Tz = Tx2 = Ty2 = Tz2 = Txy = Tyz = Tzx = 0;
		U1 = Ux = Uy = Uz = Ux2 = Uy2 = Uz2 = Uxy = Uyz = Uzx = 0;

		for (ConvexHullWrapperFace face : data.faces) {

			computeFaceIntegrals(face);

			U1 += F1;

			switch (projection) {
			case XY:
				T1 += na * Fa;

				Tx += na * Fa2;
				Ty += nb * Fb2;
				Tz += nc * Fc2;

				Tx2 += na * Fa3;
				Ty2 += nb * Fb3;
				Tz2 += nc * Fc3;

				Txy += na * Fa2b;
				Tyz += nb * Fb2c;
				Tzx += nc * Fc2a;

				Ux += Fa;
				Uy += Fb;
				Uz += Fc;

				Ux2 += Fa2;
				Uy2 += Fb2;
				Uz2 += Fc2;

				Uxy += Fab;
				Uyz += Fbc;
				Uzx += Fca;

				break;
			case YZ:
				T1 += nc * Fc;

				Ty += na * Fa2;
				Tz += nb * Fb2;
				Tx += nc * Fc2;

				Ty2 += na * Fa3;
				Tz2 += nb * Fb3;
				Tx2 += nc * Fc3;

				Tyz += na * Fa2b;
				Tzx += nb * Fb2c;
				Txy += nc * Fc2a;

				Uy += Fa;
				Uz += Fb;
				Ux += Fc;

				Uy2 += Fa2;
				Uz2 += Fb2;
				Ux2 += Fc2;

				Uyz += Fab;
				Uzx += Fbc;
				Uxy += Fca;

				break;
			case ZX:
				T1 += nb * Fb;

				Tz += na * Fa2;
				Tx += nb * Fb2;
				Ty += nc * Fc2;

				Tz2 += na * Fa3;
				Tx2 += nb * Fb3;
				Ty2 += nc * Fc3;

				Tzx += na * Fa2b;
				Txy += nb * Fb2c;
				Tyz += nc * Fc2a;

				Uz += Fa;
				Ux += Fb;
				Uy += Fc;

				Uz2 += Fa2;
				Ux2 += Fb2;
				Uy2 += Fc2;

				Uzx += Fab;
				Uxy += Fbc;
				Uyz += Fca;

				break;
			}

		}

		Tx /= 2f;
		Ty /= 2f;
		Tz /= 2f;

		Tx2 /= 3f;
		Ty2 /= 3f;
		Tz2 /= 3f;

		Txy /= 2f;
		Tyz /= 2f;
		Tzx /= 2f;
	}

	private static void computeFaceIntegrals(ConvexHullWrapperFace face) {

		Vector3f normal = face.getNormal();
		chooseProjection(normal);

		computeProjectionIntegrals(face);

		float na2 = na * na;
		float na3 = na2 * na;
		float nb2 = nb * nb;
		float nb3 = nb2 * nb;

		float w = -face.getPlaneOffset();
		float w2 = w * w;
		float w3 = w2 * w;
		float k1 = 1.0f / nc;
		float k2 = k1 * k1;
		float k3 = k2 * k1;
		float k4 = k3 * k1;

		F1 = k1 * pi_1;
		Fa = k1 * pi_a;
		Fb = k1 * pi_b;
		Fc = -k2 * (na * pi_a + nb * pi_b + w * pi_1);

		Fab = k1 * pi_ab;
		Fbc = -k2 * (nb * pi_b2 + w * pi_b);
		Fca = -k2 * (na * pi_a2 + w * pi_a);

		Fa2 = k1 * pi_a2;
		Fb2 = k1 * pi_b2;
		Fc2 = k3 * (na2 * pi_a2 + 2 * na * nb * pi_ab + nb2 * pi_b2 + 2 * na * w * pi_a + 2 * nb * w * pi_b
				+ w2 * pi_1);

		Fa3 = k1 * pi_a3;
		Fb3 = k1 * pi_b3;
		Fc3 = -k4 * (na3 * pi_a3 + 3 * na2 * nb * pi_a2b + 3 * na * nb2 * pi_ab2 + nb3 * pi_b3 + 3 * na2 * w * pi_a2
				+ 6 * na * nb * w * pi_ab + 3 * nb2 * w * pi_b2 + 3 * na * w2 * pi_a + 3 * nb * w2 * pi_b + w3 * pi_1);

		Fa2b = k1 * pi_a2b;
		Fb2c = -k2 * (na * pi_ab2 + nb * pi_b3 + w * pi_b2);
		Fc2a = k3 * (na2 * pi_a3 + 2 * na * nb * pi_a2b + nb2 * pi_ab2 + 2 * na * w * pi_a2 + 2 * nb * w * pi_ab
				+ w2 * pi_a);
	}

	private static void computeProjectionIntegrals(ConvexHullWrapperFace face) {
		pi_1 = pi_a = pi_b = pi_a2 = pi_b2 = pi_a3 = pi_b3 = pi_ab = pi_a2b = pi_ab2 = 0;

		for (ConvexHullWrapperHalfEdge edge : face) {

			loadCoords(edge);

			float Da = a1 - a0;
			float Db = b1 - b0;

			float a02 = a0 * a0;
			float a03 = a02 * a0;
			float a04 = a03 * a0;
			float a12 = a1 * a1;
			float a13 = a12 * a1;

			float b02 = b0 * b0;
			float b03 = b02 * b0;
			float b04 = b03 * b0;
			float b12 = b1 * b1;
			float b13 = b12 * b1;

			float C1 = a1 + a0;
			float Ca = a1 * C1 + a02;
			float Ca2 = a1 * Ca + a03;
			float Ca3 = a1 * Ca2 + a04;
			float Cb = b12 + b1 * b0 + b02;
			float Cb2 = b1 * Cb + b03;
			float Cb3 = b1 * Cb2 + b04;
			float Cab = 3 * a12 + 2 * a1 * a0 + a02;
			float Kab = a12 + 2 * a1 * a0 + 3 * a02;
			float Ca2b = a0 * Cab + 4 * a13;
			float Ka2b = a1 * Kab + 4 * a03;
			float Cab2 = 4 * b13 + 3 * b12 * b0 + 2 * b1 * b02 + b03;
			float Kab2 = b13 + 2 * b12 * b0 + 3 * b1 * b02 + 4 * b03;

			pi_1 += Db * C1;
			pi_a += Db * Ca;
			pi_a2 += Db * Ca2;
			pi_a3 += Db * Ca3;
			pi_b += Da * Cb;
			pi_b2 += Da * Cb2;
			pi_b3 += Da * Cb3;
			pi_ab += Db * (b1 * Cab + b0 * Kab);
			pi_a2b += Db * (b1 * Ca2b + b0 * Ka2b);
			pi_ab2 += Da * (a1 * Cab2 + a0 * Kab2);

		}

		pi_1 /= 2f;
		pi_a /= 6f;
		pi_a2 /= 12f;
		pi_a3 /= 20f;
		pi_b /= -6f;
		pi_b2 /= -12f;
		pi_b3 /= -20f;
		pi_ab /= 24f;
		pi_a2b /= 60f;
		pi_ab2 /= -60f;

	}

	private static void loadCoords(ConvexHullWrapperHalfEdge edge) {
		// load a0, a1, b0, b1 from edge's end points
		Vector3f start = edge.getTail();
		Vector3f end = edge.getHead();
		switch (projection) {
		case XY:
			a0 = start.x;
			a1 = end.x;
			b0 = start.y;
			b1 = end.y;
			break;
		case YZ:
			a0 = start.y;
			a1 = end.y;
			b0 = start.z;
			b1 = end.z;
			break;
		case ZX:
			a0 = start.z;
			a1 = end.z;
			b0 = start.x;
			b1 = end.x;
			break;
		}
	}

	private static void chooseProjection(Vector3f normal) {

		float absX = Math.abs(normal.x);
		float absY = Math.abs(normal.y);
		float absZ = Math.abs(normal.z);

		if (absX > absY) {
			if (absX > absZ) {
				projection = Projection.YZ; // x > y && x > z
				na = normal.y;
				nb = normal.z;
				nc = normal.x;
			} else {
				projection = Projection.XY; // x > y && z > x
				na = normal.x;
				nb = normal.y;
				nc = normal.z;
			}
		} else {
			if (absY > absZ) {
				projection = Projection.ZX; // y > x && y > z
				na = normal.z;
				nb = normal.x;
				nc = normal.y;
			} else {
				projection = Projection.XY; // y > x && z > y
				na = normal.x;
				nb = normal.y;
				nc = normal.z;
			}
		}

	}

	/**
	 * Exprime le tenseur d'inertie apr�s une translation de l'objet.
	 * 
	 * @param inertia     Le tenseur d'inertie exprim� par rapport au centre de
	 *                    masse de l'objet.
	 * @param translation La transation � appliquer.
	 * @param mass        La masse de l'objet.
	 */
	static void translateInertia(Matrix3f inertia, Vector3f translation, float mass) {
		float rx2 = translation.x * translation.x;
		float ry2 = translation.y * translation.y;
		float rz2 = translation.z * translation.z;

		float rx = translation.x;
		float ry = translation.y;
		float rz = translation.z;

		inertia.m00 += (ry2 + rz2) * mass;
		inertia.m11 += (rx2 + rz2) * mass;
		inertia.m22 += (rx2 + ry2) * mass;

		inertia.m01 += -(rx * ry) * mass;
		inertia.m02 += -(rz * rx) * mass;
		inertia.m12 += -(ry * rz) * mass;

		inertia.m10 = inertia.m01;
		inertia.m20 = inertia.m02;
		inertia.m21 = inertia.m12;
	}

}
