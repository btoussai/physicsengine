package cataclysm.constraints;

import org.lwjgl.util.vector.Vector3f;

import cataclysm.Epsilons;
import math.Clamp;

/**
 * Définit une contrainte simple, en opposition à {@link CompoundConstraint}.
 * <br>
 * Quelques explications:<br>
 * Notations:<br>
 * X^T est la transposée de X. <br>
 * X[i] est la ième composante de X. <br>
 * <br>
 * 
 * Conceptuellement, une contrainte s'écrit ainsi:<br>
 * <br>
 * C(x) est l'erreur de position (vecteur colonne), avec x = (position,
 * orientation). On cherche généralement à résoudre C(x) = 0. <br>
 * En dérivant par rapport au temps:<br>
 * Cdot(x) = J*V + b = 0 <br>
 * J est la matrice jacobienne de la contrainte, V la vitesse et b le 'bias'.
 * <br>
 * Dans le cas d'une contrainte entre deux solides, on aura par exemple V = [ va
 * wa -vb -wb ]^T. <br>
 * <br>
 * V correspond à la vitesse satisfaisant la contrainte, on a donc: <br>
 * V = V0 + M^-1 * J^T * lambda<br>
 * Avec V0 la vitesse avant correction et M la matrice dite 'de masse'. Lambda
 * est un vecteur contenant la valeur des impulsions appliquées aux solides.<br>
 * Plus précisément, M est une matrice 12x12: <br>
 * [__I*ma__0____0_____0__]<br>
 * [__0_____Ia___0_____0__]<br>
 * [__0_____0____I*mb__0__]<br>
 * [__0_____0____0_____Ib_]<br>
 * 
 * <br>
 * Avec I la matrice identitée 3x3, ma et mb la masse des solides et Ia et Ib
 * leur tenseur d'inertie en world-space. <br>
 * 
 * <br>
 * En développant: <br>
 * <br>
 * Cdot(x) = J * V0 + J * M^-1 * J^T * lambda + beta / h * C(x) + gamma / h *
 * lambda = 0 <br>
 * On cherche donc à résoudre cette équation pour lambda.<br>
 * h est le pas de temps, typiquement 1/60 s.<br>
 * beta et gamma sont des facteurs d'amortissement, ils permettent de moduler le
 * comportemnt de la contrainte (comme un ressort +/- rigide). <br>
 * Le fait d'injecter C(x) dans l'équation de Cdot(x) participe à la modulation
 * du comportement de la contrainte. <br>
 * <br>
 * Pour simplifier la résolution, on définit quelques termes utiles:<br>
 * J*V0 est l'erreur de vitesse. inv_mass = J * M^-1 * J^T est 'la masse
 * inverse'. Elle s'exprime en kg^-1.<br>
 * <br>
 * On définit également omega la fréquence angulaire en [rad.s^-1] et zeta le
 * facteur d'ammortissement [sans unité]. <br>
 * <br>
 * k = omega² / inv_mass[i]<br>
 * c = 2*zeta*omega / inv_mass[i]<br>
 * <br>
 * beta = h * k / (c + h*k)<br>
 * gamma = 1 / (c + h*k)<br>
 * 
 * <br>
 * Une résolution directe pour obtenir lambda serait envisageable, cependant
 * lambda doit satisfaire des inégaltés du type a_i < lambda[i] < b_i. 
 * <br>
 * On résout donc le système itérativement par la méthode des impulsions
 * séquentielles.
 * 
 * 
 * @author Briac
 *
 */
public abstract class SimpleConstraint extends AbstractConstraint {

	/**
	 * Une borne inférieure sur les impulsions appliquées pour satisfaire la
	 * contrainte.
	 */
	private float lowerForceLimit = Float.NEGATIVE_INFINITY;

	/**
	 * Une borne supérieure sur les impulsions appliquées pour satisfaire la
	 * contrainte.
	 */
	private float upperForceLimit = Float.POSITIVE_INFINITY;

	/**
	 * La direction d'application des impulsions, du point de vue du corps A.
	 */
	private final Vector3f N = new Vector3f();

	/**
	 * Le point d'application des impulsions sur le corps A.
	 */
	private final Vector3f Ra = new Vector3f();

	/**
	 * Le point d'application des impulsions sur le corps B.
	 */
	private final Vector3f Rb = new Vector3f();

	private final Vector3f RaxN = new Vector3f();
	private final Vector3f RbxN = new Vector3f();

	private float impulse;
	private float position_error;
	private float inv_mass;
	private float velocity_error;

	public SimpleConstraint(AnchorPoint pointA, AnchorPoint pointB) {
		super(pointA, pointB);
	}

	@Override
	protected void solveVelocity(boolean firstIteration, float timeStep, Vector3f temp) {
		if (firstIteration) {
			impulse = 0;
			velocity_error = buildVelocityJacobian(N, Ra, Rb, RaxN, RbxN, temp);
			inv_mass = computeVelocityInvMass(N, Ra, Rb, RaxN, RbxN, temp);
		} else {
			//the velocity may have changed due to other constraints, we need to recompute it.
			velocity_error = computeVelocityError(N, RaxN, RbxN, temp);
		}

		float oldImpulse = impulse;
		float corrective_impulse = computeVelocityImpulse(velocity_error, inv_mass, timeStep);
		float newImpulse = oldImpulse + corrective_impulse;
		newImpulse = Clamp.clamp(newImpulse, getLowerForceLimit(), getUpperForceLimit());
		impulse = newImpulse;
		float applied_impulse = newImpulse - oldImpulse;

		applyImpulse(N, RaxN, RbxN, temp, applied_impulse);

	}

	@Override
	protected void solvePosition(boolean firstIteration, float timeStep, Vector3f temp) {
		if (!hasPositionCorrection()) {
			return;
		}

		if (firstIteration) {
			impulse = 0;
			position_error = buildPositionJacobian(N, Ra, Rb, RaxN, RbxN, temp);
			inv_mass = computePositionInvMass(N, Ra, Rb, RaxN, RbxN, temp);
		} else {
			position_error = computePositionError(N, RaxN, RbxN, temp);
		}

		float oldImpulse = impulse;
		float corrective_impulse = computePositionImpulse(position_error, inv_mass, timeStep);
		float newImpulse = oldImpulse + corrective_impulse;
		impulse = newImpulse;
		float applied_impulse = newImpulse - oldImpulse;

		applyPseudoImpulse(N, RaxN, RbxN, temp, applied_impulse);
	}

	/**
	 * Calcule la i-ème ligne de la matrice jacobienne de la contrainte. Puisqu'il
	 * est trop coûteux de calculer l'ensemble de la matrice et de l'inverser, on ne
	 * calcule que les éléments non nuls. Le système est résolu itérativement en
	 * appliquant des impulsions. Les arguments sont destinés a être remplis. Des
	 * arguments inutiles peuvent tout à fait être ignorés. <br>
	 * Par exemple, certaintes contraintes ne portent que sur la vitesse angulaire
	 * du solide, les impulsions correspondent alors à des couples selon la
	 * direction N et les vecteurs Ra, Rb, Raxn et Rbxn sont inutiles.
	 * 
	 * @param N              Le vecteur indiquant la direction d'application de
	 *                       l'impulsion.
	 * @param Ra             Le vecteur indiquant le point d'appliquation de
	 *                       l'impulsion sur le solide A. Ce point est exprimé en
	 *                       world-space, mais est relatif au centre de masse du
	 *                       solide.
	 * @param Rb             Le vecteur indiquant le point d'appliquation de
	 *                       l'impulsion sur le solide B. Ce point est exprimé en
	 *                       world-space, mais est relatif au centre de masse du
	 *                       solide.
	 * @param RaxN           Le produit vectoriel entre Ra et N.
	 * @param RbxN           Le produit vectoriel entre Rb et N.
	 * @param velocity_error Un scalaire indiquant l'erreur de vitesse. Par exemple,
	 *                       dans le cas d'une contrainte de distance, il s'agira de
	 *                       la vitesse selon la normale N.
	 * @param temp           Un vecteur servant de variable intermédiaire pour les
	 *                       calculs.
	 */
	protected abstract float buildVelocityJacobian(Vector3f N, Vector3f Ra, Vector3f Rb, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp);

	/**
	 * Calcule la masse inverse pour la ième ligne de la matrice jacobienne: <br>
	 * inv_mass[i] = J[i] * M^-1 * J[i]^T
	 * 
	 * @param N
	 * @param Ra
	 * @param Rb
	 * @param RaxN
	 * @param RbxN
	 * @param inv_mass
	 * @param temp
	 */
	protected abstract float computeVelocityInvMass(Vector3f N, Vector3f Ra, Vector3f Rb, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp);

	/**
	 * Calcule la ième erreur de vitesse corresondant à la ième ligne de la matrice
	 * jacobienne: <br>
	 * speed[i] = J[i] * V0
	 * 
	 * @param N
	 * @param RaxN
	 * @param RbxN
	 * @param velocity_error
	 * @param temp
	 * @param i
	 */
	protected abstract float computeVelocityError(Vector3f N, Vector3f RaxN, Vector3f RbxN, Vector3f temp);

	/**
	 * Applique l'impulsion correspondant à la ième ligne de la matrice jacobienne:
	 * <br>
	 * V = V0 + M^-1 * J[i]^T * applied_impulse
	 * 
	 * @param N
	 * @param RaxN
	 * @param RbxN
	 * @param temp
	 * @param applied_impulse L'impulsion appliquée, i.e. lambda[i] - lambda_prec[i]
	 *                        où lambda_prec correspond à l'impulsion de l'itération
	 *                        précédente.
	 * @param i
	 */
	protected abstract void applyImpulse(Vector3f N, Vector3f RaxN, Vector3f RbxN, Vector3f temp,
			float applied_impulse);

	/**
	 * L'équivalent de
	 * {@link #buildVelocityJacobian(Vector3f, Vector3f, Vector3f, Vector3f, Vector3f)}
	 * pour la position.
	 * 
	 * @param N
	 * @param Ra
	 * @param Rb
	 * @param RaxN
	 * @param RbxN
	 * @param temp
	 */
	protected abstract float buildPositionJacobian(Vector3f N, Vector3f Ra, Vector3f Rb, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp);

	/**
	 * L'équivalent de
	 * {@link #computeVelocityInvMass(Vector3f, Vector3f, Vector3f, Vector3f, Vector3f, Vector3f)}
	 * pour la position.
	 * 
	 * @param N
	 * @param Ra
	 * @param Rb
	 * @param RaxN
	 * @param RbxN
	 * @param temp
	 */
	protected abstract float computePositionInvMass(Vector3f N, Vector3f Ra, Vector3f Rb, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp);

	/**
	 * L'équivalent de
	 * {@link #computeVelocityError(Vector3f[], Vector3f[], Vector3f[], float[], Vector3f, int)}
	 * pour la position.
	 * 
	 * @param N
	 * @param RaxN
	 * @param RbxN
	 * @param temp
	 */
	protected abstract float computePositionError(Vector3f N, Vector3f RaxN, Vector3f RbxN, Vector3f temp);

	/**
	 * L'équivalent de {@link #applyImpulse(Vector3f, Vector3f, Vector3f, Vector3f)}
	 * pour la position.
	 * 
	 * @param N
	 * @param RaxN
	 * @param RbxN
	 * @param temp
	 * @param applied_impulse
	 */
	protected abstract void applyPseudoImpulse(Vector3f N, Vector3f RaxN, Vector3f RbxN, Vector3f temp,
			float applied_impulse);

	protected float computeVelocityImpulse(float velocity_error, float inv_mass, float timeStep) {
		return -velocity_error / inv_mass;
	}

	protected float computePositionImpulse(float position_error, float inv_mass, float timeStep) {
		return -Epsilons.PENETRATION_RECOVERY * position_error / (inv_mass * timeStep);
	}

	protected float getLowerForceLimit() {
		return lowerForceLimit;
	}

	protected float getUpperForceLimit() {
		return upperForceLimit;
	}

	public void setForceLimits(float lower, float upper) {
		this.lowerForceLimit = lower;
		this.upperForceLimit = upper;
	}
	
	/**
	 * @return L'impulsion appliquée lors de la résolution.
	 */
	public float getImpulse() {
		return impulse;
	}

	@Override
	public String toString() {
		return this.getClass().getSimpleName() + ":\n" + pointA + "\n" + pointB;
	}

}
