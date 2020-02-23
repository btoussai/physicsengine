package math;

import java.util.Random;

import org.lwjgl.util.vector.Vector3f;

/**
 * Cette classe permet de g�n�rer des nombres al�atoires dans un certain
 * intervalle.
 * 
 * @author Briac
 *
 */
public class RandGen {

	private static Random rand = new Random();

	public static double nextDouble(double min, double max) {
		return min + rand.nextDouble() * (max - min);
	}

	public static float nextFloat(float min, float max) {
		return min + rand.nextFloat() * (max - min);
	}

	public static int nextInt(int min, int max) {
		return min + rand.nextInt(max - min);
	}

	public static long nextLong(long min, long max) {
		return min + rand.nextLong() % (max - min);
	}

	public static Vector3f newUnitVector3f() {
		Vector3f vector = new Vector3f(nextFloat(-1, 1), nextFloat(-1, 1), nextFloat(-1, 1));
		return vector.normalise(vector);
	}

	public static Vector3f nextVector3f(float minLength, float maxLength) {
		float length = nextFloat(minLength, maxLength);
		Vector3f vector = newUnitVector3f();
		vector.scale(length);
		return vector;
	}

	/**
	 * @return a random float in [-PI, PI]
	 */
	public static float nextAngle() {
		return (float) nextDouble(-Math.PI, Math.PI);
	}

	public static void seed(long seed) {
		rand.setSeed(seed);
	}

}
