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
	 * Cette fonction est appelée dès qu'un objet entre en collision avec un staticmesh.
	 */
	private Consumer<Wrapper> onCollisionWithGround;
	
	public CataclysmCallbacks() {
		
	}

	public CataclysmCallbacks(BiConsumer<Wrapper, Wrapper> onCollision, Consumer<Wrapper> onCollisionWithGround) {
		this.onCollision = onCollision;
		this.onCollisionWithGround = onCollisionWithGround;
	}

	public BiConsumer<Wrapper, Wrapper> getOnCollision() {
		return onCollision;
	}

	public Consumer<Wrapper> getOnCollisionWithGround() {
		return onCollisionWithGround;
	}
	
	

}
