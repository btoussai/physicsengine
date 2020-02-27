package math;

import math.vector.Vector3f;

/**
 * Permet de restreindre une valeur entre deux bornes.
 * 
 * @author Briac
 *
 */
public class Clamp {

	/**
	 * Restreint la valeur entre 0.0f et 1.0f.
	 * 
	 * @param value La valeur � restreindre.
	 * @return La valeur restreinte � l'intervalle [0.0f, 1.0f].
	 */
	public static float clamp(float value) {
		if (value < 0.0f)
			return 0.0f;
		if (value > 1.0f)
			return 1.0f;
		return value;
	}

	/**
	 * Restreint la valeur dans l'intervalle [minValue, maxValue].
	 * 
	 * @param value    La valeur � restreindre.
	 * @param minValue La borne inf�rieure de l'intervalle.
	 * @param maxValue La borne sup�rieure de l'intervalle.
	 * @return La valeur restreinte � l'intervalle [minValue, maxValue].
	 */
	public static float clamp(float value, float minValue, float maxValue) {
		if (value < minValue)
			return minValue;
		if (value > maxValue)
			return maxValue;
		return value;
	}

	/**
	 * Restreint la valeur dans l'intervalle [minValue, maxValue].
	 * 
	 * @param value    La valeur � restreindre.
	 * @param minValue La borne inf�rieure de l'intervalle.
	 * @param maxValue La borne sup�rieure de l'intervalle.
	 * @return La valeur restreinte � l'intervalle [minValue, maxValue].
	 */
	public static int clamp(int value, int minValue, int maxValue) {
		if (value < minValue)
			return minValue;
		if (value > maxValue)
			return maxValue;
		return value;
	}

	/**
	 * Restreint la valeur dans l'intervalle [-1.0f, 1.0f].
	 * 
	 * @param value La valeur � restreindre.
	 * @return La valeur restreinte � l'intervalle [-1.0f, 1.0f].
	 */
	public static float symetricClamp(float value) {
		if (value < -1.0f)
			return -1.0f;
		if (value > 1.0f)
			return 1.0f;
		return value;
	}

	/**
	 * Restreint les composantes du vecteur dans l'intervalle [minValue, maxValue].
	 * @param vector 
	 * @param minValue La borne inf�rieure de l'intervalle.
	 * @param maxValue La borne sup�rieure de l'intervalle.
	 */
	public static void clamp(Vector3f vector, float minValue, float maxValue) {
		vector.set(clamp(vector.x, minValue, maxValue), clamp(vector.y, minValue, maxValue),
				clamp(vector.z, minValue, maxValue));
	}

	/**
	 * Restreint la valeur dans l'intervalle [-maxValue, maxValue].
	 * 
	 * @param value    La valeur � restreindre.
	 * @param maxValue La valeur maximale en valeur absolue de l'intervalle.
	 * @return La valeur restreinte � l'intervalle [-maxValue, maxValue].
	 */
	public static float symetricClamp(float value, float maxValue) {
		if (value < -maxValue)
			return -maxValue;
		if (value > maxValue)
			return maxValue;
		return value;
	}

}
