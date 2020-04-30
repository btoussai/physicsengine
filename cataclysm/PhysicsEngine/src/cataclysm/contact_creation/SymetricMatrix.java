package cataclysm.contact_creation;

final class SymetricMatrix {

	enum Size {
		M1x1, M2x2, M3x3, M4x4;
	}

	private final double[] coefficients;
	private final double[] init_coefficients;
	public final Size size;

	public SymetricMatrix(Size size) {
		this.size = size;
		switch (size) {
		case M1x1:
			coefficients = new double[1];
			init_coefficients = new double[1];
			break;
		case M2x2:
			coefficients = new double[3];
			init_coefficients = new double[3];
			break;
		case M3x3:
			coefficients = new double[6];
			init_coefficients = new double[6];
			break;
		case M4x4:
			coefficients = new double[10];
			init_coefficients = new double[10];
			break;
		default:
			throw new IllegalArgumentException("Invalid enum value: " + size);
		}
	}

	public void set1x1(double m00) {
		coefficients[0] = m00;

		init_coefficients[0] = m00;
	}

	public void set2x2(double m00, double m10, double m11) {
		coefficients[0] = m00;
		coefficients[1] = m10;
		coefficients[2] = m11;

		init_coefficients[0] = m00;
		init_coefficients[1] = m10;
		init_coefficients[2] = m11;
	}

	public void set3x3(double m00, double m10, double m11, double m20, double m21, double m22) {
		coefficients[0] = m00;
		coefficients[1] = m10;
		coefficients[2] = m11;
		coefficients[3] = m20;
		coefficients[4] = m21;
		coefficients[5] = m22;

		init_coefficients[0] = m00;
		init_coefficients[1] = m10;
		init_coefficients[2] = m11;
		init_coefficients[3] = m20;
		init_coefficients[4] = m21;
		init_coefficients[5] = m22;
	}

	public void set4x4(double m00, double m10, double m11, double m20, double m21, double m22, double m30, double m31,
			double m32, double m33) {
		coefficients[0] = m00;
		coefficients[1] = m10;
		coefficients[2] = m11;
		coefficients[3] = m20;
		coefficients[4] = m21;
		coefficients[5] = m22;
		coefficients[6] = m30;
		coefficients[7] = m31;
		coefficients[8] = m32;
		coefficients[9] = m33;

		init_coefficients[0] = m00;
		init_coefficients[1] = m10;
		init_coefficients[2] = m11;
		init_coefficients[3] = m20;
		init_coefficients[4] = m21;
		init_coefficients[5] = m22;
		init_coefficients[6] = m30;
		init_coefficients[7] = m31;
		init_coefficients[8] = m32;
		init_coefficients[9] = m33;
	}

	/**
	 * Inverts the matrix. The matrix can be singular, hence the pseudo inverse is
	 * computed instead, through the formula: <br>
	 * A+ = lim A* (AA* + delta * I )^-1 when delta -> 0 <br>
	 * where A* denotes the conjugate transpose of A. <br>
	 * The computation is carried out with a small value compared to the norm of A
	 * for delta.
	 * 
	 */
	public void invertInPlace() {
		switch (size) {
		case M1x1:
			invert1x1();
			return;
		case M2x2:
			invert2x2();
			return;
		case M3x3:
			invert3x3();
			return;
		case M4x4:
			invert4x4();
			return;
		}
	}

	private void invert1x1() {
		coefficients[0] = 1.0f / coefficients[0];
	}

	private void invert2x2() {
		double m00 = coefficients[0], l00 = m00;
		double m10 = coefficients[1], l10 = m10;
		double m11 = coefficients[2], l11 = m11;
		// A * A
		m00 = l00 * l00 + l10 * l10;
		m10 = l00 * l10 + l10 * l11;
		m11 = l10 * l10 + l11 * l11;

		double norm = Math.sqrt(l00 * l00 + 2.0 * l10 * l10 + l11 * l11);
		double epsilon = norm * Math.ulp(1.0) * 1.0E7;

		m00 += epsilon;
		m11 += epsilon;

		// m00 m10
		// m10 m11

		double t00 = m11;
		double t10 = -m10;
		double t11 = m00;

		double det = m00 * t00 + m10 * t10;
		double inv_det = 1.0f / det;

		m00 = l00 * t00 + l10 * t10;
		m10 = l00 * t10 + l10 * t11;
		m11 = l10 * t10 + l11 * t11;

		coefficients[0] = m00 * inv_det;
		coefficients[1] = m10 * inv_det;
		coefficients[2] = m11 * inv_det;
	}

	private void invert3x3() {
		double m00 = coefficients[0], l00 = m00;
		double m10 = coefficients[1], l10 = m10;
		double m11 = coefficients[2], l11 = m11;
		double m20 = coefficients[3], l20 = m20;
		double m21 = coefficients[4], l21 = m21;
		double m22 = coefficients[5], l22 = m22;

		// A * A
		m00 = l00 * l00 + l10 * l10 + l20 * l20;
		m10 = l00 * l10 + l10 * l11 + l20 * l21;
		m11 = l10 * l10 + l11 * l11 + l21 * l21;
		m20 = l00 * l20 + l10 * l21 + l20 * l22;
		m21 = l10 * l20 + l11 * l21 + l21 * l22;
		m22 = l20 * l20 + l21 * l21 + l22 * l22;

		double norm = Math
				.sqrt(l00 * l00 + 2.0 * l10 * l10 + l11 * l11 + 2.0 * l20 * l20 + 2.0 * l21 * l21 + l22 * l22);
		double epsilon = norm * Math.ulp(1.0) * 1.0E7;

		m00 += epsilon;
		m11 += epsilon;
		m22 += epsilon;

		// m00 m10 m20
		// m10 m11 m21
		// m20 m21 m22

		double t00 = det2x2(m11, m21, m21, m22);
		double t10 = -det2x2(m10, m20, m21, m22);
		double t11 = det2x2(m00, m10, m20, m21);
		double t20 = det2x2(m10, m20, m11, m21);
		double t21 = -det2x2(m00, m20, m10, m21);
		double t22 = det2x2(m00, m10, m10, m11);

		double det = m00 * t00 + m10 * t10 + m20 * t20;
		double inv_det = 1.0f / det;

		m00 = l00 * t00 + l10 * t10 + l20 * t20;
		m10 = l00 * t10 + l10 * t11 + l20 * t21;
		m11 = l10 * t10 + l11 * t11 + l21 * t21;
		m20 = l00 * t20 + l10 * t21 + l20 * t22;
		m21 = l10 * t20 + l11 * t21 + l21 * t22;
		m22 = l20 * t20 + l21 * t21 + l22 * t22;

		coefficients[0] = m00 * inv_det;
		coefficients[1] = m10 * inv_det;
		coefficients[2] = m11 * inv_det;
		coefficients[3] = m20 * inv_det;
		coefficients[4] = m21 * inv_det;
		coefficients[5] = m22 * inv_det;
	}

	private void invert4x4() {
		double m00 = coefficients[0], l00 = m00;
		double m10 = coefficients[1], l10 = m10;
		double m11 = coefficients[2], l11 = m11;
		double m20 = coefficients[3], l20 = m20;
		double m21 = coefficients[4], l21 = m21;
		double m22 = coefficients[5], l22 = m22;
		double m30 = coefficients[6], l30 = m30;
		double m31 = coefficients[7], l31 = m31;
		double m32 = coefficients[8], l32 = m32;
		double m33 = coefficients[9], l33 = m33;

		// A * A
		m00 = l00 * l00 + l10 * l10 + l20 * l20 + l30 * l30;
		m10 = l00 * l10 + l10 * l11 + l20 * l21 + l30 * l31;
		m11 = l10 * l10 + l11 * l11 + l21 * l21 + l31 * l31;
		m20 = l00 * l20 + l10 * l21 + l20 * l22 + l30 * l32;
		m21 = l10 * l20 + l11 * l21 + l21 * l22 + l31 * l32;
		m22 = l20 * l20 + l21 * l21 + l22 * l22 + l32 * l32;
		m30 = l00 * l30 + l10 * l31 + l20 * l32 + l30 * l33;
		m31 = l10 * l30 + l11 * l31 + l21 * l32 + l31 * l33;
		m32 = l20 * l30 + l21 * l31 + l22 * l32 + l32 * l33;
		m33 = l30 * l30 + l31 * l31 + l32 * l32 + l33 * l33;

		double norm = Math.sqrt(l00 * l00 + 2.0 * l10 * l10 + l11 * l11 + 2.0 * l20 * l20 + 2.0 * l21 * l21 + l22 * l22
				+ 2.0 * l30 * l30 + 2.0 * l31 * l31 + 2.0 * l32 * l32 + l33 * l33);
		double epsilon = norm * Math.ulp(1.0) * 1.0E7;

		m00 += epsilon;
		m11 += epsilon;
		m22 += epsilon;
		m33 += epsilon;

		// m00 m10 m20 m30
		// m10 m11 m21 m31
		// m20 m21 m22 m32
		// m30 m31 m32 m33

		double s0 = m00 * m11 - m10 * m10;
		double s1 = m00 * m21 - m10 * m20;
		double s2 = m00 * m31 - m10 * m30;
		double s3 = m10 * m21 - m11 * m20;
		double s4 = m10 * m31 - m11 * m30;
		double s5 = m20 * m31 - m21 * m30;

		double c5 = m22 * m33 - m32 * m32;
		double c4 = m21 * m33 - m31 * m32;
		double c3 = m21 * m32 - m31 * m22;
		double c2 = m20 * m33 - m30 * m32;
		double c1 = m20 * m32 - m30 * m22;
		double c0 = m20 * m31 - m30 * m21;

		double det = s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0;
		double inv_det = 1.0 / det;

		double t00 = m11 * c5 - m21 * c4 + m31 * c3;
		double t10 = -m10 * c5 + m20 * c4 - m30 * c3;
		double t11 = m00 * c5 - m20 * c2 + m30 * c1;
		double t20 = m31 * s5 - m32 * s4 + m33 * s3;
		double t21 = -m30 * s5 + m32 * s2 - m33 * s1;
		double t22 = m30 * s4 - m31 * s2 + m33 * s0;
		double t30 = -m21 * s5 + m22 * s4 - m32 * s3;
		double t31 = m20 * s5 - m22 * s2 + m32 * s1;
		double t32 = -m20 * s4 + m21 * s2 - m32 * s0;
		double t33 = m20 * s3 - m21 * s1 + m22 * s0;

		m00 = l00 * t00 + l10 * t10 + l20 * t20 + l30 * t30;
		m10 = l00 * t10 + l10 * t11 + l20 * t21 + l30 * t31;
		m11 = l10 * t10 + l11 * t11 + l21 * t21 + l31 * t31;
		m20 = l00 * t20 + l10 * t21 + l20 * t22 + l30 * t32;
		m21 = l10 * t20 + l11 * t21 + l21 * t22 + l31 * t32;
		m22 = l20 * t20 + l21 * t21 + l22 * t22 + l32 * t32;
		m30 = l00 * t30 + l10 * t31 + l20 * t32 + l30 * t33;
		m31 = l10 * t30 + l11 * t31 + l21 * t32 + l31 * t33;
		m32 = l20 * t30 + l21 * t31 + l22 * t32 + l32 * t33;
		m33 = l30 * t30 + l31 * t31 + l32 * t32 + l33 * t33;

		coefficients[0] = m00 * inv_det;
		coefficients[1] = m10 * inv_det;
		coefficients[2] = m11 * inv_det;
		coefficients[3] = m20 * inv_det;
		coefficients[4] = m21 * inv_det;
		coefficients[5] = m22 * inv_det;
		coefficients[6] = m30 * inv_det;
		coefficients[7] = m31 * inv_det;
		coefficients[8] = m32 * inv_det;
		coefficients[9] = m33 * inv_det;

	}

	private double det2x2(double m00, double m01, double m10, double m11) {
		// m00 m10
		// m01 m11
		return m00 * m11 - m01 * m10;
	}

	public void computeNormalImpulses(float elasticity, float[] deltaV_N, float[] bias, float[] impulses_N) {

		switch (size) {
		case M1x1:
			computeImpulses1x1(elasticity, deltaV_N, bias, impulses_N);
			break;
		case M2x2:
			computeImpulses2x2(elasticity, deltaV_N, bias, impulses_N);
			break;
		case M3x3:
			computeImpulses3x3(elasticity, deltaV_N, bias, impulses_N);
			break;
		case M4x4:
			computeImpulses4x4(elasticity, deltaV_N, bias, impulses_N);
			break;
		}

	}

	private void computeImpulses1x1(float elasticity, float[] deltaV_N, float[] bias, float[] impulses_N) {
		double m00 = coefficients[0];
		double t0 = -(deltaV_N[0] + bias[0]);
		impulses_N[0] = (float) (m00 * t0);
	}

	private void computeImpulses2x2(float elasticity, float[] deltaV_N, float[] bias, float[] impulses_N) {
		double m00 = coefficients[0];
		double m10 = coefficients[1];
		double m11 = coefficients[2];

		double t0 = -(deltaV_N[0] + bias[0]);
		double t1 = -(deltaV_N[1] + bias[1]);

		impulses_N[0] = (float) (m00 * t0 + m10 * t1);
		impulses_N[1] = (float) (m10 * t0 + m11 * t1);
	}

	private void computeImpulses3x3(float elasticity, float[] deltaV_N, float[] bias, float[] impulses_N) {
		double m00 = coefficients[0];
		double m10 = coefficients[1];
		double m11 = coefficients[2];
		double m20 = coefficients[3];
		double m21 = coefficients[4];
		double m22 = coefficients[5];

		double t0 = -(deltaV_N[0] + bias[0]);
		double t1 = -(deltaV_N[1] + bias[1]);
		double t2 = -(deltaV_N[2] + bias[2]);

		impulses_N[0] = (float) (m00 * t0 + m10 * t1 + m20 * t2);
		impulses_N[1] = (float) (m10 * t0 + m11 * t1 + m21 * t2);
		impulses_N[2] = (float) (m20 * t0 + m21 * t1 + m22 * t2);
	}

	private void computeImpulses4x4(float elasticity, float[] deltaV_N, float[] bias, float[] impulses_N) {
		double m00 = coefficients[0];
		double m10 = coefficients[1];
		double m11 = coefficients[2];
		double m20 = coefficients[3];
		double m21 = coefficients[4];
		double m22 = coefficients[5];
		double m30 = coefficients[6];
		double m31 = coefficients[7];
		double m32 = coefficients[8];
		double m33 = coefficients[9];

		double t0 = -(deltaV_N[0] + bias[0]);
		double t1 = -(deltaV_N[1] + bias[1]);
		double t2 = -(deltaV_N[2] + bias[2]);
		double t3 = -(deltaV_N[3] + bias[3]);

		impulses_N[0] = (float) (m00 * t0 + m10 * t1 + m20 * t2 + m30 * t3);
		impulses_N[1] = (float) (m10 * t0 + m11 * t1 + m21 * t2 + m31 * t3);
		impulses_N[2] = (float) (m20 * t0 + m21 * t1 + m22 * t2 + m32 * t3);
		impulses_N[3] = (float) (m30 * t0 + m31 * t1 + m32 * t2 + m33 * t3);

		for (int i = 0; i < 4; i++) {
			if (impulses_N[i] > 100) {
				System.out.print("A: \n" + asString(init_coefficients));
				System.out.print("A^-1: \n" + asString(coefficients));
				System.out.print("ziopar");
			}
		}

	}

	@Override
	public String toString() {
		StringBuilder s = new StringBuilder("Mat" + (size.ordinal() + 1) + "[\n");
		s.append(asString(coefficients));
		s.append("]\n");
		return s.toString();
	}

	private String asString(double[] mat) {
		StringBuilder buf = new StringBuilder();
		buf.append(mat[0]).append(' ').append(mat[1]).append(' ').append(mat[3]).append(' ').append(mat[6])
				.append('\n');
		buf.append(mat[1]).append(' ').append(mat[2]).append(' ').append(mat[4]).append(' ').append(mat[7])
				.append('\n');
		buf.append(mat[3]).append(' ').append(mat[4]).append(' ').append(mat[5]).append(' ').append(mat[8])
				.append('\n');
		buf.append(mat[6]).append(' ').append(mat[7]).append(' ').append(mat[8]).append(' ').append(mat[9])
				.append('\n');
		return buf.toString();
	}
}
