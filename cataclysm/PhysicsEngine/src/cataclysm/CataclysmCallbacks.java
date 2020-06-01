package cataclysm;

import java.util.function.BiConsumer;
import java.util.function.Consumer;

import cataclysm.wrappers.Wrapper;

/**
 * Cette classe permet de mettre en place des callbacks appelées à différents
 * moments de la résolution des contacts.
 * 
 * @author Briac
 *
 */
public class CataclysmCallbacks {

	/**
	 * Cette fonction est appelée dès que deux objets entrent en collision.
	 */
	private BiConsumer<Wrapper, Wrapper> onCollision;

	/**
	 * Cette fonction est appelée dès qu'un objet entre en collision avec un
	 * staticmesh.
	 */
	private Consumer<Wrapper> onCollisionWithGround;

	public CataclysmCallbacks() {

	}

	/**
	 * 
	 * @param onCollision
	 * @param onCollisionWithGround
	 */
	public CataclysmCallbacks(@Parallelizable BiConsumer<Wrapper, Wrapper> onCollision,
			@Parallelizable Consumer<Wrapper> onCollisionWithGround) {
		this.onCollision = onCollision;
		this.onCollisionWithGround = onCollisionWithGround;
	}

	public @Parallelizable BiConsumer<Wrapper, Wrapper> getOnCollision() {
		return onCollision;
	}

	public @Parallelizable Consumer<Wrapper> getOnCollisionWithGround() {
		return onCollisionWithGround;
	}

}
