package cataclysm.wrappers;

import cataclysm.wrappers.ConvexHullWrapper.FloatLayout;
import math.vector.Matrix3f;
import math.vector.Vector3f;

/**
 * An implementation of B. Mirtich's algorithm for the computation of the mass
 * properties of a polyhedra. From the article "Fast and Accurate Computation of
 * Polyhedral Mass Properties".
 * 
 * The original algorithm has been slightly modified by myself to treat the case
 * of a hollow polyhedra, on which the mass is concentrated on its outer shell.
 * 
 * @author Briac Toussaint
 *
 */
public class PolyhedralMassProperties {

	private static enum Projection {
		XY, YZ, ZX;
	}

	private Projection projection;

	/**
	 * Computes the inertia tensor and the center of mass of a convex polyhedra.
	 * The inertia tensor is computed about the origin, not about the center of mass!
	 * The mass properties of the wrapper are also updated (mass, volume, surface area).
	 * 
	 * @param hull    The convex polyhedra.
	 * @param CM      A destination vector for the center of mass.
	 * @param inertia A destination matrix for the inertia tensor.
	 * @return the mass of the convex polyhedra
	 */
	public float computeProperties(ConvexHullWrapper hull, Vector3f CM, Matrix3f inertia) {

		computeVolumeIntegrals(hull);

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

	// U<x> --> integrale sur la surface de <x>.
	private float U1, Ux, Uy, Uz, Ux2, Uy2, Uz2, Uxy, Uyz, Uzx;
	// T<x> --> inegrale sur le volume de <x>.
	private float T1, Tx, Ty, Tz, Tx2, Ty2, Tz2, Txy, Tyz, Tzx;
	// F<x> --> integrale sur la surface de <x>
	private float F1, Fa, Fb, Fc, Fab, Fbc, Fca, Fa2, Fb2, Fc2, Fa3, Fb3, Fc3, Fa2b, Fb2c, Fc2a;
	// Composantes de la normale d'une face
	private float na, nb, nc;
	// PI<x> --> integrale sur le contour d'une face de <x>
	private float pi_1, pi_a, pi_b, pi_a2, pi_b2, pi_a3, pi_b3;
	private float pi_ab, pi_a2b, pi_ab2;
	private float a0, a1, b0, b1;

	private void computeVolumeIntegrals(ConvexHullWrapper hull) {

		T1 = Tx = Ty = Tz = Tx2 = Ty2 = Tz2 = Txy = Tyz = Tzx = 0;
		U1 = Ux = Uy = Uz = Ux2 = Uy2 = Uz2 = Uxy = Uyz = Uzx = 0;

		for (int face = 0; face < hull.faceCount; face++) {
			computeFaceIntegrals(hull, face);

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

	private final Vector3f normal = new Vector3f();

	private void computeFaceIntegrals(ConvexHullWrapper hull, int face) {

		hull.getNormal(face, normal);
		chooseProjection(normal);

		computeProjectionIntegrals(hull, face);

		float na2 = na * na;
		float na3 = na2 * na;
		float nb2 = nb * nb;
		float nb3 = nb2 * nb;

		float w = -hull.getPlaneOffset(face);
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

	private void computeProjectionIntegrals(ConvexHullWrapper hull, int face) {
		pi_1 = pi_a = pi_b = pi_a2 = pi_b2 = pi_a3 = pi_b3 = pi_ab = pi_a2b = pi_ab2 = 0;

		int edge0 = hull.getFaceEdge0(face);
		int edge = edge0;
		do {

			loadCoords(hull, edge);

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

			edge = hull.getEdgeNext(edge);
		} while (edge != edge0);

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

	private final Vector3f start = new Vector3f();
	private final Vector3f end = new Vector3f();

	private void loadCoords(ConvexHullWrapper hull, int edge) {
		// load a0, a1, b0, b1 from edge's end points
		hull.get(FloatLayout.Vertices, hull.getEdgeTail(edge), start);
		hull.get(FloatLayout.Vertices, hull.getEdgeHead(edge), end);
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

	private void chooseProjection(Vector3f normal) {

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
	 * Translates the inertia tensor.
	 * 
	 * @param inertia     The inertia tensor about the center of mass of the object, to be modified in-place.
	 * @param translation The translation vector.
	 * @param mass        The mass of the object.
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
